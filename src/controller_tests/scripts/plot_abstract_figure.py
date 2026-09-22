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
the recorded state), so the panels of two runs line up exactly and nothing is
detected from the data. On that clock the shipped configuration gives

    [0, 5)    approach, from wherever the arm was to the centre pose
    [5, 35)   Lissajous, 2 s of quintic ramp in and out
    [35, ..)  holding the centre pose

and the plot shows **the Lissajous alone, shifted so that 0 is its start**:
`--t0 5 --t1 35`, re-zeroed. That window is exactly the one in which the
controller is adapting -- the generator enables the update law at 5 s and
disables it at 35 s -- so t = 0 on the figure is also the start of the
adaptation. It also removes the approach, whose initial step in q_d puts a
spike at the origin that says nothing about either controller.

35, not 30: the Lissajous really does run for the full `run_duration` of 30 s,
the last 2 s of which are the ramp-down. Stopping earlier would cut valid data
and would no longer match the interval the estimate was produced over.

Layout: the combined figure is one column wide and stacks its panels
vertically, sharing the time axis, so that it drops into a two-column paper as
a single float. The two curves are a MATLAB-blue solid line and a black dashed
one drawn on top of it, which is what makes "the two are indistinguishable"
visible rather than merely asserted.

Usage
-----
    ros2 run controller_tests plot_abstract_figure.py \
        --server bags/gazebo_server_20260918_201326 \
        --local  bags/gazebo_local_20260918_201211 \
        --output fig2

writes fig2.pdf (the stacked figure) plus fig2_error.pdf / fig2_residual_y.pdf /
fig2_residual_yr.pdf for including the panels separately in LaTeX.
"""

import argparse
import os
import sys

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import MultipleLocator  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyse_run import read_state  # noqa: E402
from thunder_ctypes import ThunderModel  # noqa: E402


# MATLAB's default colour order: the blue of the first plotted line, and the
# red-orange of the second.
MATLAB_BLUE = '#0072BD'
MATLAB_RED = '#D95319'

# One column of a two-column page (IEEE, A4 or letter) is about 3.5 in. The
# figure is built to that width so it needs no \includegraphics scaling, which
# is what keeps the font sizes here equal to the ones in the body text.
COLUMN_WIDTH_IN = 3.5
# Height of ONE panel. Two stacked panels plus the shared axis come to roughly
# 2 * PANEL_HEIGHT_IN, which is a comfortable third of a column.
PANEL_HEIGHT_IN = 1.30

# Motion-clock instants of the phase boundaries, with the shipped lissajous.yaml.
LISSAJOUS_START_S = 5.0
LISSAJOUS_END_S = 35.0


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
    order |dpi/dt| * dt * |Y_r|, so it grows with the adaptation gains and with
    how wrong pi_hat(0) was: milli-newton-metres with a nominal start, a few
    hundredths while the estimate is still moving fast. What it is testing for
    is not that -- a wrong column order, library or parameter file is off by
    whole newton-metres, comparable to the residual itself.
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


def latex_is_available():
    """Whether matplotlib could actually drive a real LaTeX for the labels."""
    from shutil import which
    return all(which(tool) for tool in ('latex', 'dvipng')) and which('gs') is not None


def apply_style(usetex):
    """
    Paper styling: LaTeX maths, serif text, hairlines.

    With a LaTeX installation the labels are typeset by LaTeX itself and match
    the document exactly. Without one -- which is the case in the development
    container -- matplotlib's own Computer Modern is used instead: the same
    typeface, rendered by matplotlib, which is indistinguishable at figure
    sizes and needs nothing installed.
    """
    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['cmr10', 'CMU Serif', 'DejaVu Serif'],
        'mathtext.fontset': 'cm',
        # cmr10 has no real minus sign; letting the formatter typeset tick
        # labels as maths avoids the missing-glyph warnings on negative ticks.
        'axes.formatter.use_mathtext': True,
        'font.size': 8,
        'axes.labelsize': 8,
        'axes.titlesize': 8,
        'legend.fontsize': 7,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'axes.linewidth': 0.6,
        'grid.linewidth': 0.4,
        'xtick.major.width': 0.6,
        'ytick.major.width': 0.6,
        'xtick.major.size': 2.5,
        'ytick.major.size': 2.5,
        'legend.handlelength': 2.2,
        'legend.borderaxespad': 0.3,
        'legend.labelspacing': 0.25,
        'pdf.fonttype': 42,   # editable text in the PDF
        'ps.fonttype': 42,
    })
    if usetex:
        plt.rcParams.update({
            'text.usetex': True,
            'text.latex.preamble': r'\usepackage{amsmath}\usepackage{amssymb}',
        })


def style_axes(axes, ylabel, xlabel=None, xlim=None):
    axes.grid(True, which='major', linewidth=0.4, alpha=0.4)
    axes.set_ylabel(ylabel)
    if xlabel:
        axes.set_xlabel(xlabel)
    else:
        # Stacked panels share the axis: no label, and no tick labels either,
        # which is where the vertical space for a compact figure comes from.
        axes.tick_params(labelbottom=False)
    axes.margins(x=0)
    # The requested window, not the data's: the last sample lands a millisecond
    # short of the end, which is enough for the locator to drop the tick there
    # and leave the axis looking as if it stopped early.
    if xlim is not None:
        axes.set_xlim(*xlim)
    # A tick every 5 s, so the axis actually shows the start and the end of the
    # window rather than whatever round numbers happen to fall inside it.
    axes.xaxis.set_major_locator(MultipleLocator(5.0))
    # Headroom at the top so the curve never touches the spine, but a hard
    # floor at zero: every quantity plotted here is a norm.
    axes.margins(y=0.08)
    axes.set_ylim(bottom=0.0)


def draw(axes, runs, key, ylabel, xlabel, styling, xlim=None):
    """
    One panel.

    The dashed curve is drawn last and on top on purpose: the claim the figure
    makes is that the two runs lie on each other, and that is only legible if
    the one underneath can be seen through the gaps of the one above.
    """
    for label, run in runs.items():
        colour, linestyle, width, zorder = styling[label]
        axes.plot(
            run['time'], run[key],
            color=colour, linestyle=linestyle, linewidth=width,
            dash_capstyle='butt', solid_capstyle='round',
            zorder=zorder, label=label)
    style_axes(axes, ylabel, xlabel, xlim)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--server', required=True, help='bag of the server-based run')
    parser.add_argument('--local', required=True, help='bag of the in-process run')
    parser.add_argument('--output', default='fig2', help='output basename (no extension)')
    parser.add_argument(
        '--t0', type=float, default=LISSAJOUS_START_S,
        help='window start on the generator clock [s]; the default 5 is the start of '
             'the Lissajous, and of the adaptation')
    parser.add_argument(
        '--t1', type=float, default=LISSAJOUS_END_S,
        help='window end on the generator clock [s]; the default 35 is the end of the '
             'Lissajous, and of the adaptation')
    parser.add_argument(
        '--no-rezero', action='store_true',
        help='keep the generator clock on the axis instead of putting 0 at --t0')
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
    # Solid blue underneath, black dashed on top: swap the four options below
    # to reverse which run is which.
    parser.add_argument(
        '--local-colour', default=MATLAB_BLUE, help='colour of the in-process curve')
    parser.add_argument(
        '--local-style', default='-', help='line style of the in-process curve')
    parser.add_argument(
        '--server-colour', default='black', help='colour of the server curve')
    parser.add_argument(
        '--server-style', default=(0, (4.0, 2.2)),
        help="line style of the server curve; '--' or a dash pattern")
    parser.add_argument(
        '--panels', default='error,residual_y',
        help='panels of the stacked figure, in order, comma separated: '
             'error, residual_y, residual_yr')
    parser.add_argument(
        '--xlabel', default=r'$t$  [s]', help='label of the shared time axis')
    parser.add_argument(
        '--width', type=float, default=COLUMN_WIDTH_IN,
        help='figure width [in]; the default is one column of a two-column page')
    parser.add_argument(
        '--height', type=float, default=PANEL_HEIGHT_IN,
        help='height of ONE panel [in]; the stacked figure is a multiple of it')
    parser.add_argument(
        '--usetex', dest='usetex', action='store_true', default=None,
        help='typeset the labels with a real LaTeX (needs latex and dvipng)')
    parser.add_argument(
        '--no-usetex', dest='usetex', action='store_false',
        help="use matplotlib's own Computer Modern instead")
    parser.add_argument('--png', action='store_true', help='also write PNG')
    parser.add_argument(
        '--no-separate', action='store_true', help='only the stacked figure')
    arguments = parser.parse_args()

    usetex = arguments.usetex
    if usetex is None:
        usetex = latex_is_available()
    elif usetex and not latex_is_available():
        print('--usetex asked for but latex/dvipng were not found; '
              "falling back to matplotlib's Computer Modern")
        usetex = False
    apply_style(usetex)
    print(f'fonts: {"LaTeX" if usetex else "matplotlib Computer Modern"}')

    model = ThunderModel(arguments.library, arguments.parameter_file)
    print(f'model: {arguments.library}')

    runs = {}
    for label, bag in (
            (arguments.server_label, arguments.server),
            (arguments.local_label, arguments.local)):
        print(f'reading {bag} ...')
        runs[label] = load_run(
            bag, model, arguments.torque, arguments.t0, arguments.t1,
            not arguments.no_rezero)

    # The in-process run is the one whose control law can be reproduced
    # exactly, so it validates the whole ctypes path.
    gains = load_gains(arguments.gains_file)
    check = self_check(runs[arguments.local_label], model, gains)
    if check is None:
        print('self-check skipped: no gains found (pass --gains-file)')
    else:
        # The expected residual is one adaptation step, not zero; see
        # self_check(). The threshold is loose on purpose: what it has to catch
        # is a wiring error, which is of the order of the residual itself.
        verdict = 'OK' if check < 0.1 else 'SUSPECT - check --library and --parameter-file'
        print(f'self-check  max |tau_model - (Yr*pi_hat + PD)| = {check:.3e} Nm  [{verdict}]')

    # zorder 3 over 2: the server's dashed line sits on top of the in-process
    # solid one, so the overlap is something the reader can see.
    styling = {
        arguments.local_label: (arguments.local_colour, arguments.local_style, 1.1, 2),
        arguments.server_label: (arguments.server_colour, arguments.server_style, 1.0, 3),
    }

    panels = {
        'error': ('error_norm', r'$\|e\|$  [rad]'),
        'residual_y': ('residual_y', r'$\|\tau - Y\hat{\pi}\|$  [Nm]'),
        'residual_yr': ('residual_yr', r'$\|\tau - Y_r\hat{\pi}\|$  [Nm]'),
    }
    wanted = [name.strip() for name in arguments.panels.split(',') if name.strip()]
    unknown = [name for name in wanted if name not in panels]
    if unknown:
        raise SystemExit(
            f"--panels: unknown panel(s) {', '.join(unknown)}; "
            f"choose from {', '.join(panels)}")

    # Stacked in one column, sharing the time axis: only the bottom panel
    # carries tick labels and the axis label, which is most of what makes two
    # panels fit where one and a half would otherwise go.
    figure, all_axes = plt.subplots(
        len(wanted), 1, sharex=True,
        figsize=(arguments.width, arguments.height * len(wanted)),
        constrained_layout=True)
    all_axes = np.atleast_1d(all_axes)
    # What the axis should span, in plot coordinates.
    xlim = ((arguments.t0, arguments.t1) if arguments.no_rezero
            else (0.0, arguments.t1 - arguments.t0))

    for index, name in enumerate(wanted):
        key, ylabel = panels[name]
        last = index == len(wanted) - 1
        draw(all_axes[index], runs, key, ylabel,
             arguments.xlabel if last else None, styling, xlim)
    all_axes[0].legend(frameon=False, loc='upper right', ncol=len(runs))
    # constrained_layout leaves a little slack between stacked panels; taking
    # it out is free vertical space in the paper.
    figure.set_constrained_layout_pads(hspace=0.02, wspace=0.02, h_pad=0.02, w_pad=0.02)
    figure.savefig(arguments.output + '.pdf')
    if arguments.png:
        figure.savefig(arguments.output + '.png', dpi=300)
    plt.close(figure)
    print(f'wrote {arguments.output}.pdf  ({len(wanted)} panels stacked, '
          f'{arguments.width:.2f} x {arguments.height * len(wanted):.2f} in)')

    if not arguments.no_separate:
        for name, (key, ylabel) in panels.items():
            single, axes = plt.subplots(
                figsize=(arguments.width, arguments.height * 1.25),
                constrained_layout=True)
            draw(axes, runs, key, ylabel, arguments.xlabel, styling, xlim)
            axes.legend(frameon=False, loc='upper right', ncol=len(runs))
            single.set_constrained_layout_pads(h_pad=0.02, w_pad=0.02)
            output = f'{arguments.output}_{name}'
            single.savefig(output + '.pdf')
            if arguments.png:
                single.savefig(output + '.png', dpi=300)
            plt.close(single)
            print(f'wrote {output}.pdf')

    span = 'the Lissajous' if (
        arguments.t0 == LISSAJOUS_START_S and arguments.t1 == LISSAJOUS_END_S) else 'the window'
    print(f'\nsummary over {span} '
          f'({arguments.t0:g} to {arguments.t1:g} s on the generator clock'
          f'{", plotted from 0" if not arguments.no_rezero else ""}):')
    header = f'{"run":24s} {"rms |e|":>10s} {"rms r_Y":>10s} {"rms r_Yr":>10s}'
    print(header)
    for label, run in runs.items():
        print(f'{label:24s} '
              f'{np.sqrt((run["error_norm"] ** 2).mean()):10.5f} '
              f'{np.sqrt((run["residual_y"] ** 2).mean()):10.4f} '
              f'{np.sqrt((run["residual_yr"] ** 2).mean()):10.4f}')


if __name__ == '__main__':
    main()
