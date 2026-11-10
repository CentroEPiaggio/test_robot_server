#!/usr/bin/env python3
"""
Figures of the ERF 2027 paper, from the real-robot runs.

  fig_timing.pdf      controller update() duration, in-process vs server, and
                      the age of the regressor the server-fed loop used;
  fig_trajectory.pdf  difference between the joint trajectories executed with
                      the server and in-process, and between the two parameter
                      estimates, against how far each estimate travels.

Run inside the workspace container after compare_runs.py:

    python3 paper_erf/plot_paper_figures.py --local bags/real_local_p06_new \
        --server bags/real_server_p06_new --compare paper_erf/out_p06 \
        --out paper_erf/figures
"""

import argparse
import os
import sys

import numpy as np

SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'src', 'controller_tests', 'scripts')
sys.path.insert(0, SCRIPTS)
from plot_abstract_figure import (  # noqa: E402
    MATLAB_BLUE, adaptation_window, apply_style, read_run)
from analyse_run import read_state  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

# LLNCS text width.
WIDTH_IN = 4.8
JOINT_COLOURS = ['#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', '#4DBEEE', '#A2142F']


def ecdf(values):
    values = np.sort(np.asarray(values, dtype=float))
    return values, np.arange(1, values.size + 1) / values.size


def figure_timing(local_bag, server_bag, output):
    local = read_state(local_bag)
    server = read_state(server_bag)
    fig, (ax_update, ax_age) = plt.subplots(1, 2, figsize=(WIDTH_IN, 1.85))

    for messages, label, style in ((server, 'Robot Server', dict(color=MATLAB_BLUE, lw=1.4)),
                                   (local, 'in-process', dict(color='k', lw=0.8, ls='--'))):
        worst = [m.update_duration_max_us for m in messages if m.cycles_in_window > 0]
        x, y = ecdf(worst)
        ax_update.plot(x, y, label=label, **style)
    ax_update.axvline(1000.0, color='0.5', lw=0.6, ls=':')
    ax_update.text(1050.0, 0.05, 'period', color='0.4', fontsize=7)
    ax_update.set_xscale('log')
    ax_update.set_xlim(10, 3000)
    ax_update.set_xlabel(r'worst update() in 10 cycles [$\mu$s]')
    ax_update.set_ylabel('empirical CDF')
    ax_update.legend(loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=2, frameon=False)
    ax_update.grid(True, which='major', alpha=0.4)

    age = [m.model_age_ms for m in server if m.model_valid]
    x, y = ecdf(age)
    ax_age.plot(x, y, color=MATLAB_BLUE, lw=1.4)
    ax_age.set_xlabel(r'age of $Y_r$ at use [ms]')
    ax_age.set_xlim(0, 4)
    ax_age.grid(True, alpha=0.4)
    for axis in (ax_update, ax_age):
        axis.set_ylim(0, 1.02)
    fig.tight_layout(pad=0.3, w_pad=1.0)
    fig.savefig(output)
    plt.close(fig)


def figure_trajectory(local_bag, server_bag, compare_dir, output):
    data = np.load(os.path.join(compare_dir, 'trajectory_difference.npz'))
    local, server = read_run(local_bag), read_run(server_bag)
    t0, t1 = adaptation_window(local)
    grid = data['time'] + t0
    pi_local = np.column_stack([np.interp(grid, local['time'], local['pi'][:, k])
                                for k in range(local['pi'].shape[1])])
    pi_server = np.column_stack([np.interp(grid, server['time'], server['pi'][:, k])
                                 for k in range(server['pi'].shape[1])])

    fig, (ax_q, ax_p) = plt.subplots(2, 1, figsize=(WIDTH_IN, 2.9), sharex=True)
    for joint in range(7):
        ax_q.plot(data['time'], 1e3 * data['q'][:, joint], color=JOINT_COLOURS[joint],
                  lw=0.7, label=f'$q_{joint + 1}$')
    ax_q.set_ylabel(r'$q_{\mathrm{srv}} - q_{\mathrm{loc}}$ [mrad]')
    ax_q.legend(ncol=7, loc='upper center', frameon=False, handlelength=1.2,
                columnspacing=0.8, bbox_to_anchor=(0.5, 1.22))
    ax_q.grid(True, alpha=0.4)

    ax_p.plot(data['time'], np.linalg.norm(pi_server - pi_server[0], axis=1),
              color=MATLAB_BLUE, lw=1.4, label=r'$\|\hat{\pi}_{\mathrm{srv}} - \hat{\pi}(0)\|$')
    ax_p.plot(data['time'], np.linalg.norm(pi_local - pi_local[0], axis=1),
              color='k', lw=0.7, ls='--', label=r'$\|\hat{\pi}_{\mathrm{loc}} - \hat{\pi}(0)\|$')
    ax_p.plot(data['time'], np.linalg.norm(pi_server - pi_local, axis=1),
              color='#D95319', lw=1.0, label=r'$\|\hat{\pi}_{\mathrm{srv}} - \hat{\pi}_{\mathrm{loc}}\|$')
    ax_p.set_ylabel('parameter estimate')
    ax_p.set_xlabel('time [s]')
    ax_p.set_xlim(0, t1 - t0)
    ax_p.legend(loc='upper left', frameon=False)
    ax_p.grid(True, alpha=0.4)
    fig.tight_layout(pad=0.3, h_pad=0.4)
    fig.savefig(output)
    plt.close(fig)
    print('max |pi_srv - pi_loc| =', np.linalg.norm(pi_server - pi_local, axis=1).max(),
          ' final travel', np.linalg.norm(pi_local[-1] - pi_local[0]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--local', required=True)
    parser.add_argument('--server', required=True)
    parser.add_argument('--compare', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    apply_style(usetex=False)
    figure_timing(args.local, args.server, os.path.join(args.out, 'fig_timing.pdf'))
    figure_trajectory(args.local, args.server, args.compare,
                      os.path.join(args.out, 'fig_trajectory.pdf'))


if __name__ == '__main__':
    main()
