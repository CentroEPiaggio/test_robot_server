# Copyright 2026 Giorgio Simonini
# Licensed under the Apache License, Version 2.0.

"""
MACT experiment on the real Franka robot.

    ros2 launch controller_tests real_experiment.launch.py \
        controller:=local robot_ip:=172.16.0.3

The structure mirrors gazebo_experiment.launch.py exactly; only the bring-up
differs, so a simulation run and an experiment differ in the hardware and
nothing else.

Two things are genuinely different from simulation, and both are handled
automatically:

* libfranka adds the commanded torque on top of its own gravity compensation,
  so 'gravity.subtract' is turned on here (mact_launch_utils sets it from
  use_sim_time). The server-based controller therefore needs the server to
  publish reg_G: add it to the 'topics' list of franka_conf.yaml and
  regenerate, otherwise it will keep commanding gravity twice.

* The robot must already be in FCI mode and unlocked before this is launched.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.append(os.path.dirname(__file__))
from mact_launch_utils import (  # noqa: E402
    bag_recorder,
    controller_spawner,
    robot_server_node,
    trajectory_node,
)


def experiment_chain(context: LaunchContext):
    """
    Same order as in simulation, so the two are directly comparable:

      MACT controller -> bag recorder -> (2 s later) trajectory generator

    franka.launch.py already starts the controller manager and the joint state
    broadcaster, so only the MACT controller has to be spawned here.
    """
    controller = context.perform_substitution(LaunchConfiguration('controller'))
    arm_id = context.perform_substitution(LaunchConfiguration('arm_id'))
    record = context.perform_substitution(LaunchConfiguration('record')).lower() == 'true'

    if controller not in ('local', 'server'):
        raise RuntimeError(f"'controller' must be 'local' or 'server', not '{controller}'")

    actions = []
    if controller == 'server':
        # Up first: the controller queries it for the initial estimate while
        # configuring.
        actions.append(robot_server_node(use_sim_time=False))

    initial_scale = context.perform_substitution(
        LaunchConfiguration('initial_scale'))
    mact = controller_spawner(
        controller, arm_id, use_sim_time=False,
        initial_scale=float(initial_scale) if initial_scale else None)

    after_controller = []
    if record:
        after_controller.append(bag_recorder(controller, environment='real'))
    after_controller.append(
        TimerAction(period=2.0, actions=[trajectory_node(arm_id, use_sim_time=False)]))

    actions += [
        mact,
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=mact, on_exit=after_controller)),
    ]
    return actions


def generate_launch_description():
    franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('franka_bringup'), 'launch', 'franka.launch.py')),
        launch_arguments={
            'arm_id': LaunchConfiguration('arm_id'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': 'false',
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'namespace': '',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='local',
            description="Which MACT controller to run: 'local' or 'server'."),
        DeclareLaunchArgument(
            'arm_id', default_value='fr3', description='Robot model: fr3, fer, fp3.'),
        DeclareLaunchArgument(
            'robot_ip', default_value='172.16.0.3', description='Address of the robot.'),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false',
            description='Run against the mock hardware instead of the robot.'),
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

        franka,
        OpaqueFunction(function=experiment_chain),
    ])
