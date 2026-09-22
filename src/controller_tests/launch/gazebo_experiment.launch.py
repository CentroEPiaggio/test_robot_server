# Copyright 2026 Giorgio Simonini
# Licensed under the Apache License, Version 2.0.

"""
MACT experiment in the Franka Gazebo simulation.

    ros2 launch controller_tests gazebo_experiment.launch.py controller:=local
    ros2 launch controller_tests gazebo_experiment.launch.py controller:=server

Everything outside the controller itself is identical between the two runs:
the same world, the same robot, the same 1 kHz Lissajous reference, the same
recorded topics. The only difference is which plugin is spawned, and whether
the Thunder Robot Server is running.
"""

import os
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from mact_launch_utils import (  # noqa: E402
    bag_recorder,
    controller_spawner,
    gazebo_reaper,
    joint_state_broadcaster_spawner,
    robot_server_node,
    stale_controller_manager_check,
    trajectory_node,
)


def robot_state_publisher(context: LaunchContext, arm_id):
    """
    Publish the robot description, with the effort command interface enabled.

    'gazebo_effort' is what adds <command_interface name="effort"/> to the
    ros2_control block; without it a torque controller cannot claim anything.
    """
    arm_id_value = context.perform_substitution(arm_id)
    xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', arm_id_value, f'{arm_id_value}.urdf.xacro')

    description = xacro.process_file(
        xacro_file,
        mappings={
            'arm_id': arm_id_value,
            'hand': 'false',
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': 'franka_hand',
            'gazebo_effort': 'true',
        },
    ).toxml()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': description, 'use_sim_time': True}],
        ),
    ]


def experiment_chain(context: LaunchContext, spawn):
    """
    Everything after the robot exists in the world, in the right order.

    The order matters and is enforced with event handlers rather than with
    hopeful delays:

      robot spawned -> joint_state_broadcaster -> MACT controller
                    -> bag recorder -> (2 s later) trajectory generator

    The reference must not start before the controller is active, or the robot
    would already be a fraction of the approach behind when it takes over; and
    the recorder needs a moment to open the bag before the first sample.
    """
    controller = context.perform_substitution(LaunchConfiguration('controller'))
    arm_id = context.perform_substitution(LaunchConfiguration('arm_id'))
    record = context.perform_substitution(LaunchConfiguration('record')).lower() == 'true'

    if controller not in ('local', 'server'):
        raise RuntimeError(f"'controller' must be 'local' or 'server', not '{controller}'")

    actions = []
    if controller == 'server':
        # Up front: the controller queries it for the initial parameter
        # estimate while configuring, and fails if it is not there.
        actions.append(robot_server_node(use_sim_time=True))

    joint_state = joint_state_broadcaster_spawner()
    initial_scale = context.perform_substitution(
        LaunchConfiguration('initial_scale'))
    mact = controller_spawner(
        controller, arm_id, use_sim_time=True,
        initial_scale=float(initial_scale) if initial_scale.strip() != '' else None)

    trajectory = trajectory_node(arm_id, use_sim_time=True)

    after_controller = []
    if record:
        after_controller.append(bag_recorder(controller, environment='gazebo'))
    after_controller.append(TimerAction(period=2.0, actions=[trajectory]))

    actions += [
        # The controller manager only exists once the robot is in the world.
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn, on_exit=[joint_state])),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=joint_state, on_exit=[mact])),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=mact, on_exit=after_controller)),
        # The generator shuts itself down when the motion is over
        # ('stop_when_finished' in lissajous.yaml); take the rest of the launch
        # down with it. Without this, Gazebo and the controller manager stay up
        # after every run, and an interrupted launch leaves them behind for the
        # next one to collide with -- which shows up as a spawner that cannot
        # configure controllers that are, confusingly, already loaded.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=trajectory,
                on_exit=[EmitEvent(event=Shutdown(reason='trajectory finished'))])),
    ]
    return actions


def generate_launch_description():
    arm_id = LaunchConfiguration('arm_id')

    # Gazebo has to find the meshes of franka_description.
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))

    # '-s' runs the simulation server without the GUI, which is what batch
    # experiments and CI want; without it Gazebo needs a display.
    gz_arguments = [
        LaunchConfiguration('world'), ' -r',
        PythonExpression(["' -s' if '", LaunchConfiguration('headless'), "'.lower() == 'true' else ''"]),
    ]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_arguments}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Gazebo drives the controller manager with simulated time through the
    # gz_ros2_control plugin, but it does not publish /clock by itself. Without
    # this bridge every other node running with use_sim_time is stuck at t = 0:
    # the trajectory generator's 1 kHz timer never fires, and the controller
    # judges every reference to be infinitely stale.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config',
            os.path.join(
                get_package_share_directory('franka_description'),
                'rviz', 'visualize_franka.rviz'),
            '-f', 'world',
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='local',
            description="Which MACT controller to run: 'local' (Thunder linked "
                        "in-process) or 'server' (model served over topics)."),
        DeclareLaunchArgument(
            'arm_id', default_value='fr3',
            description='Robot model: fr3, fer, fp3. The kinematics of all of them match '
                        'the Thunder configuration; their URDF inertias do not.'),
        DeclareLaunchArgument(
            'world', default_value='empty.sdf', description='Gazebo world to load.'),
        DeclareLaunchArgument(
            'record', default_value='true', description='Record a bag of the run.'),
        DeclareLaunchArgument(
            'initial_scale', default_value='',
            description='Scale applied to the nominal par_REG to obtain pi_hat(0). Empty '
                        'keeps the value in mact_common.yaml. Use e.g. 0.6 to start from a '
                        'deliberately wrong estimate, which is what makes the parameter '
                        'convergence visible in the torque residual.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false', description='Also start RViz.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo without its GUI (no display needed).'),

        # First, before anything of ours is on the graph: any controller manager
        # visible at this point belongs to a run that never shut down. It also
        # takes the snapshot of pre-existing Gazebos that the reaper below
        # subtracts, so the two have to stay in this order.
        OpaqueFunction(function=stale_controller_manager_check),
        # Last: ros_gz_sim's simulator is a grandchild of the launch and does
        # not reliably die with it, so make sure this run leaves nothing that
        # the next one would collide with.
        RegisterEventHandler(
            event_handler=OnShutdown(on_shutdown=[OpaqueFunction(function=gazebo_reaper)])),

        gazebo,
        clock_bridge,
        OpaqueFunction(function=robot_state_publisher, args=[arm_id]),
        spawn,
        rviz,
        OpaqueFunction(function=experiment_chain, kwargs={'spawn': spawn}),
    ])
