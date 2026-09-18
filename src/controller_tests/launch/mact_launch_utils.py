# Copyright 2026 Giorgio Simonini
# Licensed under the Apache License, Version 2.0.

"""
Building blocks shared by the simulation and the real-robot experiment.

Everything that is not the robot bring-up lives here, so that the two launch
files differ only in how the hardware is started: same controller, same
trajectory generator, same recorded topics, same parameter files.
"""

import os
import tempfile
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# Plugin type of each controller variant, keyed by the 'controller' argument.
CONTROLLER_TYPES = {
    'local': 'mact_controllers/MactLocalController',
    'server': 'mact_controllers/MactServerController',
}

# Name the controller is spawned under. It is deliberately the same for both
# variants, so the recorded topics and the analysis scripts do not have to know
# which one was running.
CONTROLLER_NAME = 'mact_controller'

# Topics recorded for every run. The controller state carries the tracking
# error, the torques and the parameter estimate, i.e. everything the figures of
# the paper are made of.
COMMON_BAG_TOPICS = [
    '/mact/state',
    '/mact/trajectory',
    '/joint_states',
]

# Recorded on top of those when the server is in the loop, to document the
# actual traffic between the controller and the server. Yr and Y are 7x100
# matrices at 1 kHz, so they are NOT recorded by default: add them explicitly
# if you want to inspect them.
SERVER_BAG_TOPICS = [
    '/controller/q',
    '/controller/dq',
    '/controller/ddq',
    '/controller/dqr',
    '/controller/ddqr',
    '/controller/par_REG',
    '/controller/par_DYN',
]


def _write_override_file(parameters):
    """
    Write a node-agnostic parameter file with the given overrides.

    Launch arguments (the arm id, for instance) have to reach the controller,
    but the spawner only takes parameter files. Writing a small file is the
    least surprising way to pass them through.
    """
    handle = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='mact_override_', delete=False)
    yaml.safe_dump({'/**': {'ros__parameters': parameters}}, handle)
    handle.close()
    return handle.name


def controller_parameter_files(controller, arm_id, use_sim_time):
    """Return the parameter files the controller is spawned with, in order."""
    share = get_package_share_directory('mact_controllers')
    files = [
        os.path.join(share, 'config', 'mact_common.yaml'),
        os.path.join(share, 'config', f'mact_{controller}.yaml'),
    ]
    # Last file wins, so launch arguments override the defaults in the configs.
    files.append(_write_override_file({
        'arm_id': arm_id,
        'use_sim_time': use_sim_time,
    }))
    return files


def controller_spawner(controller, arm_id, use_sim_time, controller_manager='/controller_manager'):
    """Spawn the MACT controller under test."""
    return Node(
        package='controller_manager',
        executable='spawner',
        name=f'{CONTROLLER_NAME}_spawner',
        output='screen',
        arguments=[
            CONTROLLER_NAME,
            '--controller-manager', controller_manager,
            '--controller-type', CONTROLLER_TYPES[controller],
            '--controller-manager-timeout', '60',
        ] + sum(
            [['--param-file', path]
             for path in controller_parameter_files(controller, arm_id, use_sim_time)],
            [],
        ),
    )


def joint_state_broadcaster_spawner(controller_manager='/controller_manager'):
    return Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', controller_manager,
            '--controller-manager-timeout', '60',
        ],
    )


def robot_server_node(use_sim_time):
    """
    The Thunder Robot Server.

    Started only for the server-based controller; the in-process one must run
    without it, which is the whole point of the comparison.
    """
    return Node(
        package='franka_server',
        executable='franka_server_node',
        name='franka_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )


def trajectory_node(arm_id, use_sim_time, trajectory_config=None):
    """The Lissajous reference generator, identical for both controllers."""
    if trajectory_config is None:
        trajectory_config = os.path.join(
            get_package_share_directory('controller_tests'), 'config', 'lissajous.yaml')
    return Node(
        package='controller_tests',
        executable='trajectory_publisher_node',
        name='lissajous_trajectory_publisher',
        output='screen',
        parameters=[trajectory_config, {'arm_id': arm_id, 'use_sim_time': use_sim_time}],
    )


def bag_recorder(controller, environment, output_root=None, extra_topics=()):
    """
    Record the run into bags/<environment>_<controller>_<timestamp>/.

    A fixed topic list rather than '-a': recording everything would put the
    7x100 regressor matrices at 1 kHz into the bag for the server run only,
    making the two runs unequal in I/O load and the comparison unfair.
    """
    if output_root is None:
        output_root = os.path.join(os.getcwd(), 'bags')
    os.makedirs(output_root, exist_ok=True)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    destination = os.path.join(output_root, f'{environment}_{controller}_{stamp}')

    topics = list(COMMON_BAG_TOPICS)
    if controller == 'server':
        topics += SERVER_BAG_TOPICS
    topics += list(extra_topics)

    return ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--output', destination] + topics,
        output='screen',
    )
