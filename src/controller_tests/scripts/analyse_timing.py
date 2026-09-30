#!/usr/bin/env python3
# Copyright 2026 Giorgio Simonini
# Licensed under the Apache License, Version 2.0.

"""
Look at the control loop of a MACT run, to find out why the robot stopped.

    ros2 run controller_tests analyse_timing.py bags/real_local_20261001_100000
    ros2 run controller_tests analyse_timing.py bags/real_local_... bags/real_server_...
    ros2 run controller_tests analyse_timing.py bags/real_local_... --plot \\
        --log ~/.ros/log/latest/launch.log

The worst update() duration alone cannot explain a communication constraints
violation: libfranka raises it when the robot stops receiving commands, and
that also happens when the loop does not run at all -- stuck in read() or
write(), in another controller, or descheduled -- which update() never sees.
So besides the duration, this reports:

  * missed cycles. Every /mact/state carries the time of the control cycle it
    was sampled in and the number of cycles since the previous sample. At the
    loop rate, the time between two samples predicts how many cycles there
    should have been; the difference is cycles that did not run.
  * the stage breakdown of update(), where the bag has it.
  * the last windows before the recording ends, which is where a run that was
    stopped by the robot shows what happened.
  * libfranka's own view, when the bag has the robot state
    (/franka_robot_state_broadcaster/robot_state): control_command_success_rate
    -- the fraction of the last 100 commands the robot received -- the robot
    mode and the error flags.
  * with --log, the lines of a launch log that libfranka and the controller
    manager print about it, including the success rate and the packets lost
    in a row that libfranka reports when it aborts.

With --plot it draws the same as a timeline, <bag>_timing.png next to the bag.
"""

import argparse
import os
import re

import numpy as np

from analyse_run import has_stage_timing, read_topics, STATE_TOPIC

JOINT_STATE_TOPIC = '/joint_states'
ROBOT_STATE_TOPIC = '/franka_robot_state_broadcaster/robot_state'

# Lines of a launch log that are about the loop missing its deadline.
LOG_PATTERNS = [
    r'communication_constraints_violation',
    r'control_command_success_rate',
    r'packets lost',
    r'[Rr]eflex',
    r'libfranka',
    r'Failed to lock the realtime publisher',
    r'[Oo]verrun',
    r'Tracking error',
    r'Desired trajectory is stale',
    r'No usable regressor',
]


# ---------------------------------------------------------------------------
# Series
# ---------------------------------------------------------------------------

def stamp_seconds(header):
    return header.stamp.sec + 1e-9 * header.stamp.nanosec


def stage_names(message):
    """The STAGE_* constants of MactState, in index order."""
    constants = {name: getattr(message, name)
                 for name in dir(type(message)) if name.startswith('STAGE_')}
    return [name[len('STAGE_'):].lower()
            for name, _ in sorted(constants.items(), key=lambda item: item[1])]


def loop_series(samples, rate_hz):
    """Per-window series from /mact/state."""
    messages = [message for _, message in samples]
    stamp = np.array([stamp_seconds(m.header) for m in messages])
    cycles = np.array([m.cycles_in_window for m in messages], dtype=float)

    # The first window runs from the activation, whose time is not in the bag.
    expected = np.concatenate([[np.nan], np.diff(stamp) * rate_hz])
    missed = np.rint(expected) - cycles
    missed[0] = 0.0

    return {
        'stamp': stamp,
        'time': stamp - stamp[0],
        'receive': np.array([receive for receive, _ in samples]),
        'cycles': cycles,
        'missed': missed,
        'mean_us': np.array([m.update_duration_mean_us for m in messages]),
        'max_us': np.array([m.update_duration_max_us for m in messages]),
        'stage_names': stage_names(messages[0]),
        'has_stages': has_stage_timing(messages),
        'stage_mean_us': np.array([m.stage_duration_mean_us for m in messages]),
        'stage_max_us': np.array([m.stage_duration_max_us for m in messages]),
        'trajectory_time': np.array([m.trajectory_time for m in messages]),
        'trajectory_valid': np.array([m.trajectory_valid for m in messages]),
        'adaptation_enabled': np.array([m.adaptation_enabled for m in messages]),
        'model_valid': np.array([m.model_valid for m in messages]),
        'degraded_cycles': np.array([m.degraded_cycles for m in messages]),
    }


def robot_series(samples):
    """Series from franka_msgs/FrankaRobotState."""
    messages = [message for _, message in samples]
    error_fields = list(type(messages[0].current_errors).get_fields_and_field_types())
    return {
        'stamp': np.array([stamp_seconds(m.header) for m in messages]),
        'robot_time': np.array([m.time for m in messages]),
        'success_rate': np.array([m.control_command_success_rate for m in messages]),
        'mode': np.array([m.robot_mode for m in messages]),
        'error_fields': error_fields,
        'current_errors': np.array(
            [[getattr(m.current_errors, f) for f in error_fields] for m in messages]),
        'last_motion_errors': np.array(
            [[getattr(m.last_motion_errors, f) for f in error_fields] for m in messages]),
        'mode_names': {getattr(messages[0], name): name[len('ROBOT_MODE_'):].lower()
                       for name in dir(type(messages[0])) if name.startswith('ROBOT_MODE_')},
    }


def transitions(time, flags):
    """[(t, new_value)] every time a boolean series changes."""
    changes = np.flatnonzero(np.diff(flags.astype(int))) + 1
    return [(time[index], bool(flags[index])) for index in changes]


def percentile_line(values):
    points = np.percentile(values, [50, 90, 99, 99.9])
    return (f'p50 {points[0]:7.1f}  p90 {points[1]:7.1f}  p99 {points[2]:7.1f}  '
            f'p99.9 {points[3]:7.1f}  max {values.max():7.1f}')


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def report_loop(loop, rate_hz, budgets_us):
    windows = len(loop['time'])
    counted = int(loop['cycles'][1:].sum())
    span = loop['stamp'][-1] - loop['stamp'][0]
    missed = loop['missed']
    worst = int(np.argmax(missed))

    print(f'  windows              : {windows} over {span:.2f} s of control time, '
          f'{counted} cycles counted after the first')
    print(f'  expected at {rate_hz:.0f} Hz    : {span * rate_hz:.0f} cycles')
    print(f'  missed cycles        : {int(missed[missed > 0].sum())} in '
          f'{int((missed > 0).sum())} windows; worst {int(missed[worst])} '
          f'in the window ending at t = {loop["time"][worst]:.3f} s')
    if (missed < 0).any():
        print(f'                         ({int((missed < 0).sum())} windows counted more '
              'cycles than the time allows: the loop is not at the rate given with --rate)')

    receive_gap = np.diff(loop['receive'])
    print(f'  publication gaps     : largest {1e3 * receive_gap.max():.1f} ms between two '
          'samples reaching the bag')

    print('  update() [us]        :')
    print(f'    window mean        : {percentile_line(loop["mean_us"])}')
    print(f'    window max         : {percentile_line(loop["max_us"])}')
    for budget in budgets_us:
        over_max = int((loop['max_us'] > budget).sum())
        over_mean = int((loop['mean_us'] > budget).sum())
        print(f'    > {budget:6.0f} us       : worst cycle over it in {over_max} windows '
              f'({100.0 * over_max / windows:.2f} %), mean over it in {over_mean}')

    if loop['has_stages']:
        weights = loop['cycles']
        total_mean = np.average(loop['mean_us'], weights=weights)
        print('  stages [us]          :        mean   share   worst')
        for index, name in enumerate(loop['stage_names']):
            mean = np.average(loop['stage_mean_us'][:, index], weights=weights)
            worst_stage = loop['stage_max_us'][:, index].max()
            part = '  (part of model)' if name.startswith('regressor') else ''
            print(f'    {name:18s} : {mean:9.1f} {100.0 * mean / total_mean:6.1f} % '
                  f'{worst_stage:8.1f}{part}')
    else:
        print('  stages               : not in this bag (recorded before they existed)')


def report_events(loop):
    time = loop['time']
    events = []
    for name, key in (('reference', 'trajectory_valid'),
                      ('adaptation', 'adaptation_enabled'),
                      ('model', 'model_valid')):
        for when, value in transitions(time, loop[key]):
            events.append((when, f'{name} {"on" if value else "off"}'))
    degraded = np.flatnonzero(np.diff(loop['degraded_cycles']) > 0) + 1
    if len(degraded):
        events.append((time[degraded[0]], f'first degraded cycle ({len(degraded)} windows)'))
    print('  events               :' + ('' if events else ' none'))
    for when, what in sorted(events):
        print(f'    t = {when:8.3f} s  {what}')


def report_tail(loop, robot, tail):
    names = loop['stage_names']
    first = max(0, len(loop['time']) - tail)
    header = (f'    {"t [s]":>8s} {"motion":>7s} {"cycles":>6s} {"missed":>6s} '
              f'{"mean":>7s} {"max":>7s}')
    if loop['has_stages']:
        header += f'  {"slowest stage (max)":24s}'
    if robot is not None:
        header += f' {"success":>7s}'
    print(f'  last {len(loop["time"]) - first} windows:')
    print(header)
    for index in range(first, len(loop['time'])):
        line = (f'    {loop["time"][index]:8.3f} {loop["trajectory_time"][index]:7.2f} '
                f'{int(loop["cycles"][index]):6d} {int(loop["missed"][index]):6d} '
                f'{loop["mean_us"][index]:7.1f} {loop["max_us"][index]:7.1f}')
        if loop['has_stages']:
            # Skip the regressor entries, which are parts of 'model'.
            candidates = [i for i, n in enumerate(names) if not n.startswith('regressor')]
            slowest = max(candidates, key=lambda i: loop['stage_max_us'][index, i])
            line += f'  {names[slowest]:10s} {loop["stage_max_us"][index, slowest]:9.1f} us  '
        if robot is not None:
            begin = loop['stamp'][index - 1] if index > 0 else -np.inf
            inside = (robot['stamp'] > begin) & (robot['stamp'] <= loop['stamp'][index])
            rate = robot['success_rate'][inside]
            line += f' {100.0 * rate.min():6.1f}%' if len(rate) else f' {"-":>7s}'
        print(line)


def report_joint_states(samples, t0):
    stamp = np.array([stamp_seconds(m.header) for _, m in samples])
    gaps = np.diff(stamp)
    worst = int(np.argmax(gaps))
    print(f'  /joint_states        : {len(stamp)} samples; largest gap between stamps '
          f'{1e3 * gaps[worst]:.1f} ms, ending at t = {stamp[worst + 1] - t0:.3f} s '
          '(an upper bound on a stall: the broadcaster also drops samples)')


def report_robot(robot, t0):
    time = robot['stamp'] - t0
    rate = robot['success_rate']
    worst = int(np.argmin(rate))
    print(f'  robot state          : {len(time)} samples')
    print(f'    success rate       : min {100.0 * rate[worst]:.1f} % at t = {time[worst]:.3f} s; '
          f'below 100 % in {100.0 * (rate < 1.0).mean():.2f} % of samples, '
          f'below 95 % in {100.0 * (rate < 0.95).mean():.2f} %')
    robot_dt = 1e3 * np.diff(robot['robot_time'])
    print(f'    robot clock        : largest step between samples {robot_dt.max():.1f} ms '
          f'(1 ms per robot packet)')
    modes = robot['mode']
    changes = np.flatnonzero(np.diff(modes)) + 1
    print('    robot mode         : ' + robot['mode_names'].get(modes[0], str(modes[0])) +
          ''.join(f' -> {robot["mode_names"].get(modes[i], str(modes[i]))} at t = {time[i]:.3f} s'
                  for i in changes))
    for key in ('current_errors', 'last_motion_errors'):
        flags = robot[key]
        raised = [(int(np.argmax(flags[:, j])), name)
                  for j, name in enumerate(robot['error_fields']) if flags[:, j].any()]
        if raised:
            print(f'    {key:19s}: ' + ', '.join(f'{name} from t = {time[i]:.3f} s'
                                                for i, name in sorted(raised)))
        else:
            print(f'    {key:19s}: none')


def report_log(path):
    pattern = re.compile('|'.join(LOG_PATTERNS))
    with open(os.path.expanduser(path), errors='replace') as handle:
        lines = [line.rstrip() for line in handle if pattern.search(line)]
    print(f'  log {path}: {len(lines)} relevant lines')
    # Repeated warnings would drown the one line that matters: show each once,
    # with how many times it occurred.
    seen = {}
    for line in lines:
        key = re.sub(r'\[\d+\.\d+\]', '', line)
        seen.setdefault(key, [line, 0])[1] += 1
    for line, count in seen.values():
        print(f'    {line}' + (f'   (x{count})' if count > 1 else ''))


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

# Categorical slots 1-3 of the default palette, in fixed order, plus the ink
# and grid colours of its light surface.
SERIES = ['#2a78d6', '#eb6834', '#1baf7a']
INK = '#0b0b0b'
MUTED = '#52514e'
GRID = '#e4e3dc'
SURFACE = '#fcfcfb'


def plot(path, name, loop, robot, budgets_us):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    panels = ['duration', 'missed']
    if loop['has_stages']:
        panels.insert(1, 'stages')
    if robot is not None:
        panels.append('success')

    figure, axes = plt.subplots(
        len(panels), 1, sharex=True, figsize=(11, 2.3 * len(panels) + 0.6),
        facecolor=SURFACE, constrained_layout=True)
    axes = np.atleast_1d(axes)
    time = loop['time']

    def style(ax, ylabel):
        ax.set_facecolor(SURFACE)
        ax.set_ylabel(ylabel, color=MUTED)
        ax.grid(axis='y', color=GRID, linewidth=0.8)
        ax.tick_params(colors=MUTED, labelsize=8)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            ax.spines[side].set_color(GRID)
        # Phases of the run, on every panel, so a problem can be read against
        # them: when the reference started and when adaptation was switched.
        for when, value in transitions(time, loop['adaptation_enabled']):
            ax.axvline(when, color=MUTED, linewidth=0.8, linestyle=':')
        for when, value in transitions(time, loop['trajectory_valid']):
            ax.axvline(when, color=MUTED, linewidth=0.8, linestyle='--')

    def label_end(ax, y, text, color):
        ax.annotate(text, (time[-1], y), xytext=(4, 0), textcoords='offset points',
                    color=INK, fontsize=8, va='center',
                    bbox={'boxstyle': 'round,pad=0.15', 'fc': SURFACE, 'ec': color, 'lw': 1})

    for ax, panel in zip(axes, panels):
        if panel == 'duration':
            ax.plot(time, loop['max_us'], color=SERIES[0], linewidth=1.2, label='worst cycle')
            ax.plot(time, loop['mean_us'], color=SERIES[1], linewidth=1.2, label='mean')
            for budget in budgets_us:
                ax.axhline(budget, color=MUTED, linewidth=0.8, linestyle='--')
                ax.annotate(f'{budget:.0f} us', (0, budget), xycoords=('axes fraction', 'data'),
                            xytext=(2, 2), textcoords='offset points', color=MUTED, fontsize=7)
            style(ax, 'update() [us]')
            ax.legend(loc='upper left', fontsize=8, frameon=False, ncol=2)
            ax.set_title(f'{name}: control loop per /mact/state window', loc='left',
                         color=INK, fontsize=10)
        elif panel == 'stages':
            names = loop['stage_names']
            index = {n: i for i, n in enumerate(names)}
            regressor_r = loop['stage_max_us'][:, index['regressor_r']]
            regressor = loop['stage_max_us'][:, index['regressor']]
            others = [i for i, n in enumerate(names)
                      if not n.startswith('regressor') and n != 'model']
            # Everything outside the regressors, including the model stage's own
            # overhead around them. A sum of per-stage maxima is an upper bound
            # on any single cycle, which is what matters here.
            rest = (loop['stage_max_us'][:, others].sum(axis=1) +
                    np.maximum(loop['stage_max_us'][:, index['model']] - regressor_r - regressor,
                               0.0))
            for values, color, label in ((regressor_r, SERIES[0], 'Y_r'),
                                         (regressor, SERIES[1], 'Y'),
                                         (rest, SERIES[2], 'everything else')):
                ax.plot(time, values, color=color, linewidth=1.2, label=label)
            style(ax, 'worst stage [us]')
            ax.legend(loc='upper left', fontsize=8, frameon=False, ncol=3)
        elif panel == 'missed':
            ax.vlines(time, 0, np.maximum(loop['missed'], 0), color=SERIES[0], linewidth=1.5)
            style(ax, 'missed cycles')
            ax.set_ylim(bottom=0, top=max(1.0, loop['missed'].max() * 1.15))
        elif panel == 'success':
            ax.plot(robot['stamp'] - loop['stamp'][0], 100.0 * robot['success_rate'],
                    color=SERIES[0], linewidth=1.2)
            style(ax, 'success rate [%]')
            label_end(ax, 100.0 * robot['success_rate'][-1],
                      f'{100.0 * robot["success_rate"][-1]:.0f} %', SERIES[0])

    axes[-1].set_xlabel('control time since the first sample [s]   '
                        '(dashed: reference on/off, dotted: adaptation on/off)', color=MUTED)
    figure.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(figure)
    print(f'  plot written to {path}')


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bags', nargs='+', help='one or more recorded runs')
    parser.add_argument('--rate', type=float, default=1000.0,
                        help='control loop rate [Hz] (default 1000)')
    parser.add_argument('--budget-us', type=float, nargs='+', default=[500.0, 1000.0],
                        help='update() durations to count the windows above (default 500 1000)')
    parser.add_argument('--tail', type=int, default=25,
                        help='how many of the last windows to list (default 25)')
    parser.add_argument('--plot', action='store_true',
                        help='also write <bag>_timing.png next to each bag')
    parser.add_argument('--log', help='launch log to scan for libfranka/controller messages')
    arguments = parser.parse_args()

    rows = []
    for bag in arguments.bags:
        name = os.path.basename(os.path.normpath(bag))
        samples = read_topics(bag, [STATE_TOPIC, JOINT_STATE_TOPIC, ROBOT_STATE_TOPIC],
                              required=[STATE_TOPIC])
        if len(samples[STATE_TOPIC]) < 2:
            print(f'{bag}: fewer than two messages on {STATE_TOPIC}')
            continue
        loop = loop_series(samples[STATE_TOPIC], arguments.rate)
        robot = robot_series(samples[ROBOT_STATE_TOPIC]) \
            if samples.get(ROBOT_STATE_TOPIC) else None

        print(f'\n=== {name} ===')
        report_loop(loop, arguments.rate, arguments.budget_us)
        if samples.get(JOINT_STATE_TOPIC):
            report_joint_states(samples[JOINT_STATE_TOPIC], loop['stamp'][0])
        if robot is not None:
            report_robot(robot, loop['stamp'][0])
        else:
            print(f'  robot state          : {ROBOT_STATE_TOPIC} not recorded')
        report_events(loop)
        report_tail(loop, robot, arguments.tail)
        if arguments.plot:
            plot(os.path.normpath(bag) + '_timing.png', name, loop, robot, arguments.budget_us)

        rows.append((name, loop, robot))

    if arguments.log:
        print()
        report_log(arguments.log)

    if len(rows) > 1:
        print('\n=== comparison ===')
        print(f'{"run":40s} {"missed":>7s} {"worst":>6s} {"p99 max":>8s} {"max":>8s} '
              f'{"mean":>7s} {"min success":>11s}')
        for name, loop, robot in rows:
            success = f'{100.0 * robot["success_rate"].min():10.1f}%' if robot is not None \
                else f'{"-":>11s}'
            print(f'{name:40s} {int(loop["missed"][loop["missed"] > 0].sum()):7d} '
                  f'{int(loop["missed"].max()):6d} {np.percentile(loop["max_us"], 99):8.1f} '
                  f'{loop["max_us"].max():8.1f} {loop["mean_us"].mean():7.1f} {success}')


if __name__ == '__main__':
    main()
