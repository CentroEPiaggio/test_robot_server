#!/usr/bin/env python3
"""
Benchmark figure of the ERF 2027 paper, from the CSVs of run_benchmarks.bash.

  left:  tail of the round trip of the regressor through a topic and through
         a service, one client;
  right: server cycle duration and topic round trip against the number of
         clients subscribed to Y_r and Y, with the mean of every single run.

    python3 paper_erf/plot_benchmarks.py paper_erf/bench_pinned paper_erf/figures
"""

import csv
import glob
import os
import sys

import numpy as np

SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'src', 'controller_tests', 'scripts')
sys.path.insert(0, SCRIPTS)
from plot_abstract_figure import MATLAB_BLUE, MATLAB_RED, apply_style  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

WIDTH_IN = 4.8


def client_samples(directory, case, kind):
    values = []
    for path in sorted(glob.glob(os.path.join(directory, f'{case}_rep*_client.csv'))):
        with open(path) as handle:
            values += [float(r['value_us']) for r in csv.DictReader(handle) if r['kind'] == kind]
    return np.asarray(values)


def client_samples_rep(path):
    with open(path) as handle:
        values = [float(r['value_us']) for r in csv.DictReader(handle) if r['kind'] == 'topic_rtt']
    return np.mean(values) / 1e3


def server_cycles(directory, case):
    values = []
    for path in sorted(glob.glob(os.path.join(directory, f'{case}_rep*_server.csv'))):
        data = np.genfromtxt(path, delimiter=',', names=True)
        values.append(data['duration_us'][data['start_us'] > 5e6])
    return np.concatenate(values)


def ecdf(values):
    values = np.sort(values)
    return values, np.arange(1, values.size + 1) / values.size


def main(directory, out):
    apply_style(usetex=False)
    fig, (ax_rtt, ax_load) = plt.subplots(1, 2, figsize=(WIDTH_IN, 1.8))

    for case, kind, label, style in (
            ('topic_load0', 'topic_rtt', 'topic', dict(color=MATLAB_BLUE, lw=1.4)),
            ('service_load0', 'service_rtt', 'service', dict(color=MATLAB_RED, lw=1.0, ls='--'))):
        x, y = ecdf(client_samples(directory, case, kind) / 1e3)
        ax_rtt.step(x, 1.0 - y + 1.0 / y.size, where='post', label=label, **style)
    ax_rtt.axvline(1.0, color='0.5', lw=0.6, ls=':')
    ax_rtt.set_yscale('log')
    ax_rtt.set_xlabel('round trip of $Y_r$ [ms]')
    ax_rtt.set_ylabel('fraction exceeding')
    ax_rtt.set_xlim(0, 12)
    ax_rtt.set_ylim(1e-5, 1.5)
    ax_rtt.legend(loc='upper right', frameon=False)
    ax_rtt.grid(True, alpha=0.4)

    clients = [1, 2, 4, 8]
    cases = [f'topic_load{n - 1}' for n in clients]
    cycle = [server_cycles(directory, case) for case in cases]
    ax_load.plot(clients, [v.mean() / 1e3 for v in cycle], color='k', marker='s', ms=3,
                 lw=1.0, label='server cycle, mean')
    ax_load.plot(clients, [np.percentile(v, 99) / 1e3 for v in cycle], color='k', marker='s',
                 ms=3, lw=0.8, ls=':', mfc='none', label='server cycle, p99')
    rtt = [client_samples(directory, case, 'topic_rtt') for case in cases]
    ax_load.plot(clients, [v.mean() / 1e3 for v in rtt], color=MATLAB_BLUE, marker='o', ms=3,
                 lw=1.0, label='topic round trip, mean')
    for n, case in zip(clients, cases):
        means = [client_samples_rep(path) for path in
                 sorted(glob.glob(os.path.join(directory, f'{case}_rep*_client.csv')))]
        ax_load.scatter([n] * len(means), means, s=8, facecolors='none',
                        edgecolors=MATLAB_BLUE, lw=0.6,
                        label='single run' if n == clients[0] else None)
    ax_load.set_xscale('log', base=2)
    ax_load.set_xticks(clients)
    ax_load.set_xticklabels([str(n) for n in clients])
    ax_load.set_xlabel('clients of $Y_r$, $Y$')
    ax_load.set_ylabel('[ms]')
    ax_load.set_ylim(0, 2.4)
    ax_load.legend(loc='upper center', ncol=2, frameon=False, fontsize=6,
                   columnspacing=0.6, handlelength=1.2, bbox_to_anchor=(0.53, 1.02))
    ax_load.grid(True, alpha=0.4)

    fig.tight_layout(pad=0.3, w_pad=1.0)
    os.makedirs(out, exist_ok=True)
    fig.savefig(os.path.join(out, 'fig_benchmarks.pdf'))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
