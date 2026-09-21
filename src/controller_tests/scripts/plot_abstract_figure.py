#!/usr/bin/env python3
# Copyright 2026 Giorgio Simonini
# Licensed under the Apache License, Version 2.0.

r"""
Figures for "Thunder Dynamics: A Robot Server for ROS".

Produces the two data panels of Fig. 2 from a pair of recorded runs:

  (b) the tracking error ||e||, e = q_d - q;
  (c) a measure of how well the parameter estimate explains the robot, namely
      the torque prediction residual.

Why a residual and not the estimates themselves: the regressor has a null
space, so many parameter vectors reproduce the same dynamics and plotting
individual entries of pi_hat says little about whether the estimate is *good*.
The residual does:

    r_Y (t)  = tau_meas - Y (q, qd, qdd) * pi_hat
    r_Yr(t)  = tau_meas - Y_r(q, qd, qd_d, qdd_d) * pi_hat

`tau_meas` is the torque actually applied, read from the effort state
interface; both regressors are re-evaluated here from the recorded state using
the same generated code the controller ran. r_Y uses the measured
acceleration, which is the noisiest signal in the loop; r_Yr replaces it with
the reference motion, so it is much cleaner whenever the tracking error is
small, at the cost of being exact only in that limit.

IMPORTANT: use the measured torque, not the commanded one. The control law is
tau_model = Y_r * pi_hat + PD, so tau_model - Y_r * pi_hat is identically the
PD term and carries no information about the estimate. What makes the residual
meaningful is that tau_meas is an independent measurement of what the robot
did. In these Gazebo runs the two differ by up to 0.5 Nm, which is essentially
the joint friction the model leaves out — and that friction is also the floor
the residual cannot go below.

The time axis is the trajectory generator's own clock (`trajectory_time` in
the recorded state), so the panels of two runs line up exactly: 0 is the start
of the approach and 5 s the start of the Lissajous with the shipped
configuration. Nothing is detected from the data.

Usage
-----
    ros2 run controller_tests plot_abstract_figure.py \
        --server bags/gazebo_server_20260918_201326 \
        --local  bags/gazebo_local_20260918_201211 \
        --output fig2

writes fig2.pdf (all panels) plus fig2_error.pdf / fig2_residual_y.pdf /
fig2_residual_yr.pdf for including the panels separately in LaTeX.
"""

import argparse
import os
import sys

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyse_run import read_state  # noqa: E402
from thunder_ctypes import ThunderModel  # noqa: E402


# MATLAB's default colour order: the blue of the first plotted line, and the
# red-orange of the second.
MATLAB_BLUE = '#0072BD'
MATLAB_RED = '#D95319'


def default_library():
    """Locate libthunder_franka.so through the sourced workspace."""
    for variable in ('AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH'):
        for prefix in os.environ.get(variable, '').split(':'):
            if not prefix:
                continue
            for candidate in (
                os.path.join(prefix, 'lib', 'libthunder_franka.so'),
                os.path.join(prefix, 'libthunder_franka.so'),
            ):
                if os.path.exists(candidate):
                    return candidate
    return 'libthunder_franka.so'


def default_gains_file():
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
        candidate = os.path.join(
            prefix, 'share', 'mact_controllers', 'config', 'mact_common.yaml')
        if os.path.exists(candidate):
            return candidate
    return ''


def load_gains(path):
    """Read k_p and k_v from the controller configuration, for the self-check."""
    if not path or not os.path.exists(path):
        return None
    import yaml
    with open(path) as handle:
        config = yaml.safe_load(handle)
    try:
        gains = config['/**']['ros__parameters']['gains']
        return np.asarray(gains['k_p']), np.asarray(gains['k_v'])
    except (KeyError, TypeError):
        return None


def default_parameter_file():
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
        candidate = os.path.join(
            prefix, 'share', 'thunder_franka', 'config', 'franka_par.yaml')
        if os.path.exists(candidate):
            return candidate
    return ''


def motion_clock(messages, time, dq_d, logger=print):
    """
    The time axis: the generator's own motion clock.

    Every sample carries `trajectory_time`, which the trajectory generator
    reports as `time_from_start` and the controller forwards: 0 while the
    start pose is being held, then counting up from the beginning of the
    approach. Nothing has to be detected, and two runs are directly
    comparable.

    Older bags recorded before that field existed fall back to locating the
    Lissajous by its speed, which is only approximate -- and unreliable for
    comparing two runs, because the amount of the approach that reached the
    controller depended on how long discovery took. Re-record rather than rely
    on it.
    """
    clock = np.array([getattr(m, 'trajectory_time', 0.0) for m in messages], dtype=float)
    if np.any(clock > 0.0):
        return clock

    logger('  WARNING: no trajectory_time in this bag (recorded before the motion clock '
           'was added); falling back to detecting the Lissajous, which is not reliable '
           'for comparing runs. Re-record if the figure matters.')
    peak = np.abs(dq_d).max()
    if peak > 0.0:
        fast = np.flatnonzero(np.abs(dq_d).max(axis=1) > 0.05 * peak)
        if fast.size:
            # Put the Lissajous where the motion clock would put it.
            return time - time[fast[0]] + 5.0
    return time - time[0]


def load_run(bag, model, torque, t0, t1, rezero=False):
    """Read a run and compute everything the figure needs."""
    messages = read_state(bag)
    if not messages:
        raise SystemExit(f'{bag}: no samples on /mact/state')

    time = np.array(
        [m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for m in messages])
    q = np.array([m.q for m in messages])
    dq = np.array([m.dq for m in messages])
    ddq = np.array([m.ddq for m in messages])
    dq_d = np.array([m.dq_d for m in messages])
    ddq_d = np.array([m.ddq_d for m in messages])
    error = np.array([m.e for m in messages])
    estimate = np.array([m.pi_hat for m in messages])
    tau_meas = np.array([m.tau_meas for m in messages])
    tau_model = np.array([m.tau_model for m in messages])

    time = motion_clock(messages, time, dq_d)

    # Everything recorded while the generator was holding the start pose
    # carries a clock of exactly 0, so those samples would all pile up on the
    # left edge of the plot and skew the summary. Keep only the motion, plus
    # the last held sample so the curve starts at t = 0.
    held = np.flatnonzero(time <= 0.0)
    first = held[-1] if held.size else 0

    window = (time >= t0) & (time <= t1)
    window[:first] = False
    if not window.any():
        raise SystemExit(f'{bag}: no samples in [{t0}, {t1}] s')
    if rezero:
        # Put t = 0 at the start of the window, e.g. to plot the Lissajous
        # alone with --t0 5 --rezero.
        time = time - t0

    reference_torque = tau_meas if torque == 'measured' else tau_model

    residual_y = np.empty((int(window.sum()), q.shape[1]))
    residual_yr = np.empty_like(residual_y)
    for out_index, index in enumerate(np.flatnonzero(window)):
        pi = estimate[index]
        residual_y[out_index] = (
            reference_torque[index] - model.regressor(q[index], dq[index], ddq[index]) @ pi)
        residual_yr[out_index] = (
            reference_torque[index] -
            model.regressor_r(q[index], dq[index], dq_d[index], ddq_d[index]) @ pi)

    return {
        'name': os.path.basename(os.path.normpath(bag)),
        'time': time[window],
        'error_norm': np.linalg.norm(error[window], axis=1),
        'residual_y': np.linalg.norm(residual_y, axis=1),
        'residual_yr': np.linalg.norm(residual_yr, axis=1),
        # Kept for the self-check below.
        '_q': q[window], '_dq': dq[window], '_dq_d': dq_d[window],
        '_ddq_d': ddq_d[window], '_pi': estimate[window],
        '_tau_model': tau_model[window], '_e': error[window],
        '_de': np.array([m.de for m in messages])[window],
    }


def self_check(run, model, gains):
    """
    Verify that the regressors evaluated here match the ones the controller used.

    For the in-process controller the control law is exactly
    tau_model = Y_r * pi_hat + k_p e + k_v edot, so recomputing Y_r from the
    recorded state reproduces tau_model almost exactly. "Almost" because the
    recorded pi_hat is the estimate *after* that cycle's update while
    tau_model was computed with the previous one, which leaves a residual of
    order |dpi/dt| * dt * |Y_r| -- micro- to milli-newton-metres. Anything much
    larger means the ctypes wiring, the column-major ordering or the parameter
    file is wrong, and the residual plot cannot be trusted.
    """
    if gains is None:
        return None
    k_p, k_v = gains
    worst = 0.0
    step = max(1, len(run['time']) // 200)
    for index in range(0, len(run['time']), step):
        predicted = (
            model.regressor_r(
                run['_q'][index], run['_dq'][index],
                run['_dq_d'][index], run['_ddq_d'][index]) @ run['_pi'][index]
            + k_p * run['_e'][index] + k_v * run['_de'][index])
        worst = max(worst, np.abs(predicted - run['_tau_model'][index]).max())
    return worst


def style_axes(axes, ylabel, xlabel=None):
    axes.grid(True, which='major', linewidth=0.4, alpha=0.5)
    axes.set_ylabel(ylabel)
    if xlabel:
        axes.set_xlabel(xlabel)
    axes.margins(x=0)


def draw(axes, runs, key, ylabel, xlabel, colours, styles):
    for label, run in runs.items():
        axes.plot(
            run['time'], run[key],
            color=colours[label], linestyle=styles[label],
            linewidth=1.4 if styles[label] == '-' else 1.6,
            label=label)
    style_axes(axes, ylabel, xlabel)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--server', required=True, help='bag of the server-based run')
    parser.add_argument('--local', required=True, help='bag of the in-process run')
    parser.add_argument('--output', default='fig2', help='output basename (no extension)')
    parser.add_argument('--t0', type=float, default=0.0, help='window start [s]')
    parser.add_argument('--t1', type=float, default=30.0, help='window end [s]')
    parser.add_argument(
        '--rezero', action='store_true',
        help='put t = 0 at --t0; the approach occupies the first 5 s of the motion '
             'clock, so --t0 5 --t1 35 --rezero plots the Lissajous alone')
    parser.add_argument(
        '--torque', choices=('measured', 'model'), default='measured',
        help="torque in the residual; 'model' makes it identically the PD term")
    parser.add_argument('--library', default=default_library())
    parser.add_argument('--parameter-file', default=default_parameter_file())
    parser.add_argument(
        '--gains-file', default=default_gains_file(),
        help='controller config, used only to verify the recomputed regressors')
    parser.add_argument(
        '--server-label', default='server', help='legend entry for the server run')
    parser.add_argument(
        '--local-label', default='in-process', help='legend entry for the linked run')
    parser.add_argument(
        '--local-colour', default=MATLAB_RED,
        help=f"colour of the in-process curve (use {MATLAB_BLUE} for a one-colour figure)")
    parser.add_argument('--width', type=float, default=7.16, help='figure width [in]')
    parser.add_argument('--height', type=float, default=2.1, help='figure height [in]')
    parser.add_argument('--png', action='store_true', help='also write PNG')
    parser.add_argument(
        '--no-separate', action='store_true', help='only the combined figure')
    arguments = parser.parse_args()

    plt.rcParams.update({
        'font.size': 8,
        'axes.labelsize': 8,
        'axes.titlesize': 8,
        'legend.fontsize': 7,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'axes.linewidth': 0.6,
        'pdf.fonttype': 42,   # editable text in the PDF
        'ps.fonttype': 42,
    })

    model = ThunderModel(arguments.library, arguments.parameter_file)
    print(f'model: {arguments.library}')

    runs = {}
    for label, bag in (
            (arguments.server_label, arguments.server),
            (arguments.local_label, arguments.local)):
        print(f'reading {bag} ...')
        runs[label] = load_run(
            bag, model, arguments.torque, arguments.t0, arguments.t1, arguments.rezero)

    # The in-process run is the one whose control law can be reproduced
    # exactly, so it validates the whole ctypes path.
    gains = load_gains(arguments.gains_file)
    check = self_check(runs[arguments.local_label], model, gains)
    if check is None:
        print('self-check skipped: no gains found (pass --gains-file)')
    else:
        # The expected residual is one adaptation step, not zero; see
        # self_check(). A wrong column order would be off by whole newton-metres.
        verdict = 'OK' if check < 1e-2 else 'SUSPECT - check --library and --parameter-file'
        print(f'self-check  max |tau_model - (Yr*pi_hat + PD)| = {check:.3e} Nm  [{verdict}]')

    colours = {arguments.server_label: MATLAB_BLUE,
               arguments.local_label: arguments.local_colour}
    styles = {arguments.server_label: '-', arguments.local_label: ':'}

    panels = [
        ('error_norm', r'$\|e\|$  [rad]', 'error'),
        ('residual_y', r'$\|\tau - Y\hat{\pi}\|$  [Nm]', 'residual_y'),
        ('residual_yr', r'$\|\tau - Y_r\hat{\pi}\|$  [Nm]', 'residual_yr'),
    ]

    figure, all_axes = plt.subplots(
        1, len(panels), figsize=(arguments.width, arguments.height), constrained_layout=True)
    for axes, (key, ylabel, _) in zip(np.atleast_1d(all_axes), panels):
        draw(axes, runs, key, ylabel, 'time  [s]', colours, styles)
    np.atleast_1d(all_axes)[0].legend(frameon=False, loc='best')
    figure.savefig(arguments.output + '.pdf')
    if arguments.png:
        figure.savefig(arguments.output + '.png', dpi=300)
    plt.close(figure)
    print(f'wrote {arguments.output}.pdf')

    if not arguments.no_separate:
        for key, ylabel, suffix in panels:
            single, axes = plt.subplots(
                figsize=(arguments.width / len(panels), arguments.height),
                constrained_layout=True)
            draw(axes, runs, key, ylabel, 'time  [s]', colours, styles)
            axes.legend(frameon=False, loc='best')
            name = f'{arguments.output}_{suffix}'
            single.savefig(name + '.pdf')
            if arguments.png:
                single.savefig(name + '.png', dpi=300)
            plt.close(single)
            print(f'wrote {name}.pdf')

    print('\nsummary over the plotted window:')
    header = f'{"run":24s} {"rms |e|":>10s} {"rms r_Y":>10s} {"rms r_Yr":>10s}'
    print(header)
    for label, run in runs.items():
        print(f'{label:24s} '
              f'{np.sqrt((run["error_norm"] ** 2).mean()):10.5f} '
              f'{np.sqrt((run["residual_y"] ** 2).mean()):10.4f} '
              f'{np.sqrt((run["residual_yr"] ** 2).mean()):10.4f}')


if __name__ == '__main__':
    main()
