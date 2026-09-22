# Copyright 2026 Giorgio Simonini
# Licensed under the Apache License, Version 2.0.

"""
Building blocks shared by the simulation and the real-robot experiment.

Everything that is not the robot bring-up lives here, so that the two launch
files differ only in how the hardware is started: same controller, same
trajectory generator, same recorded topics, same parameter files.
"""

import os
import signal
import tempfile
import time
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


# Gazebo processes that were already running when this launch started. Anything
# matching _GAZEBO_PATTERNS that is NOT in here appeared during our own run and
# is ours to clean up; anything in here belongs to somebody else and is left
# strictly alone. Populated by stale_controller_manager_check().
_GAZEBO_PIDS_AT_START = frozenset()

# ros_gz_sim runs Gazebo as `ruby <path>/ign gazebo ...` under `shell=True`, so
# these are the shell, the ruby launcher and the simulator itself.
_GAZEBO_PATTERNS = ('ign gazebo', 'gz sim', 'ign-gazebo-server', 'gz-sim-server')


def _gazebo_pids():
    """PIDs of everything that looks like a Gazebo, from /proc."""
    if not os.path.isdir('/proc'):
        return set()          # not Linux: the reaper simply does nothing
    pids = set()
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(os.path.join('/proc', entry, 'cmdline'), 'rb') as handle:
                cmdline = handle.read().replace(b'\0', b' ').decode('utf-8', 'replace')
        except OSError:
            continue          # the process went away, or is not ours to read
        if any(pattern in cmdline for pattern in _GAZEBO_PATTERNS):
            pids.add(int(entry))
    return pids


def gazebo_reaper(context, *_args, **_kwargs):
    """
    Kill the Gazebo this launch started, if it outlived the launch.

    ros_gz_sim starts the simulator through a `ruby` wrapper under `/bin/sh -c`,
    so the process that actually simulates is a *grandchild* of the launch. The
    SIGINT and the SIGTERM that launch escalates to go to the shell, and about
    half the time the simulator survives them -- still holding a
    `/controller_manager` with the controllers loaded, which is exactly what
    makes the next run fail to configure them.

    Only PIDs that appeared after stale_controller_manager_check() took its
    snapshot are touched, so a Gazebo somebody else is running is never at risk.
    Run as an on_shutdown handler: the graceful path is given its chance first,
    and this only deals with what is left.
    """
    survivors = _gazebo_pids() - _GAZEBO_PIDS_AT_START
    if not survivors:
        return []

    # Let the launch's own SIGINT/SIGTERM finish the job if it is going to.
    for _ in range(40):
        time.sleep(0.1)
        survivors = {pid for pid in survivors if os.path.isdir(f'/proc/{pid}')}
        if not survivors:
            return []

    for number in (signal.SIGTERM, signal.SIGKILL):
        for pid in sorted(survivors):
            try:
                os.kill(pid, number)
            except OSError:
                pass
        time.sleep(0.5)
        survivors = {pid for pid in survivors if os.path.isdir(f'/proc/{pid}')}
        if not survivors:
            break

    print('[mact] cleaned up Gazebo processes that outlived the launch')
    if survivors:
        print(f'[mact] WARNING: these would not die: {sorted(survivors)}')
    return []


def stale_controller_manager_check(context, *_args, **_kwargs):
    """
    Refuse to start while a controller manager from a previous run is alive.

    A launch that was interrupted rather than shut down leaves Gazebo -- and
    with it the controller manager -- running. The next launch then finds the
    controllers already loaded on *that* manager and the spawner reports

        Controller already loaded, skipping load_controller
        Failed to configure controller

    which says nothing about the actual cause. This runs before anything of our
    own starts, so any controller manager it finds belongs to somebody else.

    It never fails the launch on its own account: if the graph cannot be
    inspected at all, the run goes ahead as before.
    """
    global _GAZEBO_PIDS_AT_START
    # Before anything of ours starts: whatever Gazebo is running now is not
    # ours, and gazebo_reaper() must never touch it.
    _GAZEBO_PIDS_AT_START = frozenset(_gazebo_pids())

    try:
        import rclpy
    except ImportError:
        return []

    # A private context, never the default one: launch_ros owns that, and
    # initialising or shutting it down from here would pull the rug out from
    # under the rest of the launch.
    found = []
    private = None
    node = None
    try:
        private = rclpy.Context()
        private.init()
        node = rclpy.create_node(f'mact_launch_precheck_{os.getpid()}', context=private)
        # Discovery is not instantaneous; give an existing manager a moment to
        # show up rather than declaring the graph empty on the first look.
        deadline = time.time() + 2.0
        while time.time() < deadline:
            found = [
                (namespace.rstrip('/') + '/' + name)
                for name, namespace in node.get_node_names_and_namespaces()
                if name == 'controller_manager'
            ]
            if found:
                break
            time.sleep(0.1)
    except Exception:  # noqa: BLE001 - a broken check must not break the launch
        return []
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if private is not None:
                private.try_shutdown()
        except Exception:  # noqa: BLE001
            pass

    if found:
        raise RuntimeError(
            'A controller manager is already running at ' + ', '.join(found) + '.\n'
            'This is almost always a previous run that was interrupted instead of '
            'shut down: its Gazebo is still alive, holds the controllers, and the '
            'spawner would fail to configure them against it.\n'
            "Kill it first --  pkill -9 -f 'i[g]n gazebo'  -- and launch again.\n"
            '(The [g] is deliberate: a plain pattern matches the pkill command '
            'itself and kills the shell before it kills Gazebo.)')
    return []


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


def controller_parameter_files(controller, arm_id, use_sim_time, initial_scale=None):
    """Return the parameter files the controller is spawned with, in order."""
    share = get_package_share_directory('mact_controllers')
    files = [
        os.path.join(share, 'config', 'mact_common.yaml'),
        os.path.join(share, 'config', f'mact_{controller}.yaml'),
    ]
    # Last file wins, so launch arguments override the defaults in the configs.
    #
    # The gravity source is the one thing that genuinely differs between the
    # two environments, because the thing that compensates gravity differs:
    # franka_ign_ros2_control computes it from the URDF with KDL, the real
    # robot uses its own model. Selecting the matching one makes the
    # cancellation exact on both sides; it is the same choice for both
    # controllers, so the comparison stays fair.
    overrides = {
        'arm_id': arm_id,
        'use_sim_time': use_sim_time,
        'gravity.source': 'urdf_kdl' if use_sim_time else 'franka_model',
    }
    # Starting the estimate away from the nominal value is how the parameter
    # convergence is made visible: with pi_hat(0) already nominal there is
    # little for the adaptation to do and the torque residual just sits on the
    # unmodelled-friction floor.
    if initial_scale is not None:
        overrides['adaptation.initial_scale'] = float(initial_scale)
    files.append(_write_override_file(overrides))
    return files


def controller_spawner(
        controller, arm_id, use_sim_time, controller_manager='/controller_manager',
        initial_scale=None):
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
             for path in controller_parameter_files(
                 controller, arm_id, use_sim_time, initial_scale)],
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
