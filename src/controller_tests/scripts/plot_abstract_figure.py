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

# Both columns of a two-column page (IEEE) are 7.16 in across. The figure is
# built to that width so it needs no \includegraphics scaling, which is what
# keeps the font sizes here equal to the ones in the body text. Use --width 3.5
# for a single-column float.
TEXT_WIDTH_IN = 3.5 #7.16
# Height of ONE row of panels.
ROW_HEIGHT_IN = 1.3 #2.0

# Motion-clock instants of the phase boundaries, used only when a bag is too
# old to carry the adaptation flag; see adaptation_window().
LISSAJOUS_START_S = 5.0
LISSAJOUS_END_S = 20.0


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


def read_run(bag):
    """Read one bag into plain arrays, on the generator's motion clock."""
    messages = read_state(bag)
    if not messages:
        raise SystemExit(f'{bag}: no samples on /mact/state')

    stamps = np.array(
        [m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for m in messages])
    dq_d = np.array([m.dq_d for m in messages])

    return {
        'name': os.path.basename(os.path.normpath(bag)),
        'time': motion_clock(messages, stamps, dq_d),
        'q': np.array([m.q for m in messages]),
        'dq': np.array([m.dq for m in messages]),
        'ddq': np.array([m.ddq for m in messages]),
        'dq_d': dq_d,
        'ddq_d': np.array([m.ddq_d for m in messages]),
        'e': np.array([m.e for m in messages]),
        'de': np.array([m.de for m in messages]),
        'pi': np.array([m.pi_hat for m in messages]),
        'tau_meas': np.array([m.tau_meas for m in messages]),
        'tau_model': np.array([m.tau_model for m in messages]),
        'adapting': np.array(
            [bool(getattr(m, 'adaptation_enabled', False)) for m in messages]),
    }


def adaptation_window(raw):
    """
    The interval in which the controller was adapting, on the motion clock.

    This is the window worth plotting, and the bag states it rather than
    leaving it to be assumed: the generator switches the update law on at the
    start of the Lissajous and off again when it ends, and the controller
    records the flag in every sample. Reading it back means the figure follows
    `run_duration` automatically -- shorten the trajectory and the axis
    shortens with it, with nothing to keep in step by hand.

    The bounds are rounded to a tenth of a second. The diagnostics are
    published every tenth control cycle, so the flag is only ever observed to
    within 10 ms of the instant the service call landed, and 5.008 to 19.998 is
    a 14.99 s axis on which the locator will not put a tick at 15. The true
    boundaries are whole multiples of the configured phase durations, so
    rounding recovers them rather than inventing them.

    None when the bag predates the flag; the caller then falls back to the
    phase durations of the shipped configuration.
    """
    adapting = np.flatnonzero(raw['adapting'])
    if adapting.size == 0:
        return None
    return (round(float(raw['time'][adapting[0]]), 1),
            round(float(raw['time'][adapting[-1]]), 1))


def window_run(raw, model, torque, t0, t1, rezero=True):
    """Cut a run down to [t0, t1] and compute everything the figure needs."""
    time = raw['time']

    # Everything recorded while the generator was holding the start pose
    # carries a clock of exactly 0, so those samples would all pile up on the
    # left edge of the plot and skew the summary.
    held = np.flatnonzero(time <= 0.0)
    first = held[-1] if held.size else 0

    window = (time >= t0) & (time <= t1)
    window[:first] = False
    if not window.any():
        raise SystemExit(f"{raw['name']}: no samples in [{t0}, {t1}] s")
    if rezero:
        time = time - t0

    reference_torque = raw['tau_meas'] if torque == 'measured' else raw['tau_model']
    indices = np.flatnonzero(window)

    residual_y = np.empty((indices.size, raw['q'].shape[1]))
    residual_yr = np.empty_like(residual_y)
    for out_index, index in enumerate(indices):
        pi = raw['pi'][index]
        residual_y[out_index] = (
            reference_torque[index]
            - model.regressor(raw['q'][index], raw['dq'][index], raw['ddq'][index]) @ pi)
        residual_yr[out_index] = (
            reference_torque[index]
            - model.regressor_r(
                raw['q'][index], raw['dq'][index],
                raw['dq_d'][index], raw['ddq_d'][index]) @ pi)

    return {
        'name': raw['name'],
        'time': time[window],
        'error_norm': np.linalg.norm(raw['e'][window], axis=1),
        'residual_y': np.linalg.norm(residual_y, axis=1),
        'residual_yr': np.linalg.norm(residual_yr, axis=1),
        # Kept for the self-check below.
        '_q': raw['q'][window], '_dq': raw['dq'][window], '_dq_d': raw['dq_d'][window],
        '_ddq_d': raw['ddq_d'][window], '_pi': raw['pi'][window],
        '_tau_model': raw['tau_model'][window], '_e': raw['e'][window],
        '_de': raw['de'][window],
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
        'axes.labelpad': 2.0,    # default 4.0
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
        '--t0', type=float, default=None,
        help='window start on the generator clock [s]; by default the instant the '
             'controller started adapting, read from the bag')
    parser.add_argument(
        '--t1', type=float, default=None,
        help='window end on the generator clock [s]; by default the instant the '
             'controller stopped adapting, read from the bag')
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
    # A thick blue line with a thin black dashed one on top of it: the wide
    # curve underneath stays visible through the gaps of the narrow one, which
    # is what makes the two agreeing something the reader sees rather than
    # something the caption claims. Every attribute is an option, so the roles
    # can be swapped without touching the code.
    parser.add_argument(
        '--server-colour', default=MATLAB_BLUE, help='colour of the server curve')
    parser.add_argument(
        '--server-style', default='-', help='line style of the server curve')
    parser.add_argument(
        '--server-width', type=float, default=2.0,
        help='line width of the server curve [pt]')
    parser.add_argument(
        '--local-colour', default='black', help='colour of the in-process curve')
    parser.add_argument(
        '--local-style', default=(0, (4.0, 2.4)),
        help="line style of the in-process curve; '--' or a dash pattern")
    parser.add_argument(
        '--local-width', type=float, default=1.0,
        help='line width of the in-process curve [pt]')
    parser.add_argument(
        '--panels', default='error,residual_y',
        help='panels of the combined figure, in order, comma separated: '
             'error, residual_y, residual_yr')
    parser.add_argument(
        '--layout', choices=('row', 'column'), default='row',
        help="'row' puts the panels side by side (default), 'column' stacks them "
             'and shares the time axis')
    parser.add_argument(
        '--xlabel', default=r'$t$  [s]', help='label of the time axis')
    parser.add_argument(
        '--width', type=float, default=TEXT_WIDTH_IN,
        help='TOTAL figure width [in]; the default spans both columns of a '
             'two-column page. Use 3.5 for a single-column float.')
    parser.add_argument(
        '--height', type=float, default=ROW_HEIGHT_IN,
        help='height of one ROW of panels [in]')
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

    raw = {}
    for label, bag in (
            (arguments.server_label, arguments.server),
            (arguments.local_label, arguments.local)):
        print(f'reading {bag} ...')
        raw[label] = read_run(bag)

    # The window: what the bags say the adaptation interval was, unless it was
    # asked for explicitly. Taking the latest start and the earliest end keeps
    # both runs fully covered when they differ by a sample or two.
    windows = [adaptation_window(entry) for entry in raw.values()]
    if all(window is not None for window in windows):
        detected = (max(w[0] for w in windows), min(w[1] for w in windows))
        source = 'from the adaptation flag in the bags'
    else:
        detected = (LISSAJOUS_START_S, LISSAJOUS_END_S)
        source = ('from the shipped lissajous.yaml; these bags predate the '
                  'adaptation flag, so check they match the run')
    t0 = arguments.t0 if arguments.t0 is not None else detected[0]
    t1 = arguments.t1 if arguments.t1 is not None else detected[1]
    if arguments.t0 is None and arguments.t1 is None:
        print(f'window: {t0:.3f} to {t1:.3f} s on the generator clock ({source})')
    else:
        print(f'window: {t0:.3f} to {t1:.3f} s on the generator clock (asked for)')

    runs = {
        label: window_run(
            entry, model, arguments.torque, t0, t1, not arguments.no_rezero)
        for label, entry in raw.items()
    }

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

    # zorder 3 over 2: the thin dashed curve goes on top of the thick one, so
    # the wide line stays visible through its gaps.
    styling = {
        arguments.server_label: (
            arguments.server_colour, arguments.server_style, arguments.server_width, 2),
        arguments.local_label: (
            arguments.local_colour, arguments.local_style, arguments.local_width, 3),
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

    # What the time axis should span, in plot coordinates.
    xlim = (t0, t1) if arguments.no_rezero else (0.0, t1 - t0)

    # Side by side by default: every panel carries its own time axis, because
    # they are read left to right and each has to stand on its own. Stacked
    # (--layout column) they share it instead, and only the bottom one is
    # labelled, which buys vertical space at the cost of the panels no longer
    # being independent.
    stacked = arguments.layout == 'column'
    rows, columns = (len(wanted), 1) if stacked else (1, len(wanted))
    figure, all_axes = plt.subplots(
        rows, columns, sharex=stacked,
        figsize=(arguments.width, arguments.height * rows),
        constrained_layout=True)
    all_axes = np.atleast_1d(all_axes)

    for index, name in enumerate(wanted):
        key, ylabel = panels[name]
        labelled = (index == len(wanted) - 1) if stacked else True
        draw(all_axes[index], runs, key, ylabel,
             arguments.xlabel if labelled else None, styling, xlim)
    all_axes[0].legend(frameon=False, loc='upper right', ncol=len(runs))
    figure.set_constrained_layout_pads(hspace=0.02, wspace=0.02, h_pad=0.03, w_pad=0.03)
    figure.savefig(arguments.output + '.pdf')
    if arguments.png:
        figure.savefig(arguments.output + '.png', dpi=300)
    plt.close(figure)
    print(f'wrote {arguments.output}.pdf  ({len(wanted)} panels in a '
          f'{"column" if stacked else "row"}, '
          f'{arguments.width:.2f} x {arguments.height * rows:.2f} in)')

    if not arguments.no_separate:
        # One panel alone gets the width a panel of the combined figure has,
        # so a single one dropped into the text matches the rest.
        single_width = arguments.width if stacked else arguments.width / len(wanted)
        for name, (key, ylabel) in panels.items():
            single, axes = plt.subplots(
                figsize=(single_width, arguments.height),
                constrained_layout=True)
            draw(axes, runs, key, ylabel, arguments.xlabel, styling, xlim)
            axes.legend(frameon=False, loc='upper right', ncol=1)
            single.set_constrained_layout_pads(h_pad=0.03, w_pad=0.03)
            output = f'{arguments.output}_{name}'
            single.savefig(output + '.pdf')
            if arguments.png:
                single.savefig(output + '.png', dpi=300)
            plt.close(single)
            print(f'wrote {output}.pdf')

    print(f'\nsummary over {t0:g} to {t1:g} s on the generator clock'
          f'{", plotted from 0" if not arguments.no_rezero else ""}:')
    header = f'{"run":24s} {"rms |e|":>10s} {"rms r_Y":>10s} {"rms r_Yr":>10s}'
    print(header)
    for label, run in runs.items():
        print(f'{label:24s} '
              f'{np.sqrt((run["error_norm"] ** 2).mean()):10.5f} '
              f'{np.sqrt((run["residual_y"] ** 2).mean()):10.4f} '
              f'{np.sqrt((run["residual_yr"] ** 2).mean()):10.4f}')


if __name__ == '__main__':
    main()
