#!/usr/bin/env python3
"""
Server vs in-process comparison for the ERF 2027 paper.

From one pair of real-robot runs (same reference, same gains, same pi_hat(0)):

  * the difference between the two executed joint trajectories, on the
    generator's motion clock;
  * the torque deviation the server introduces in its own run, i.e. the
    commanded model torque against the one the in-process controller would
    have computed from the same recorded state;
  * the part of that deviation due to float32 transport alone;
  * the model age distribution.

Run inside the workspace container, with install/setup.bash sourced:

    python3 paper_erf/compare_runs.py --local bags/real_local_p06_new \
        --server bags/real_server_p06_new --out paper_erf/out
"""

import argparse
import json
import os
import sys

import numpy as np

SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'src', 'controller_tests', 'scripts')
sys.path.insert(0, SCRIPTS)
from plot_abstract_figure import (  # noqa: E402
    adaptation_window, default_library, default_parameter_file, read_run)
from analyse_run import read_state  # noqa: E402
from thunder_ctypes import ThunderModel  # noqa: E402


def percentiles(values):
    values = np.asarray(values, dtype=float)
    return {'mean': float(values.mean()), 'p50': float(np.percentile(values, 50)),
            'p99': float(np.percentile(values, 99)), 'max': float(values.max())}


def trajectory_difference(local, server, t0, t1, step=0.01):
    """Interpolate both runs on a common motion-clock grid and subtract."""
    grid = np.arange(t0, t1, step)
    out = {'time': grid - t0}
    for key in ('q', 'tau_cmd', 'q_d'):
        a = np.column_stack([np.interp(grid, local['time'], local[key][:, j])
                             for j in range(7)])
        b = np.column_stack([np.interp(grid, server['time'], server[key][:, j])
                             for j in range(7)])
        out[key] = b - a
    out['e_local'] = np.column_stack([np.interp(grid, local['time'], local['e'][:, j])
                                      for j in range(7)])
    return out


def torque_deviation(run, model, gains, t0, t1):
    """tau_model against Y_r(x(t)) pi_hat + PD, from the run's own recorded state."""
    k_p, k_v = gains
    # Cycles that fell back to PD + G are reported separately, not here.
    window = np.flatnonzero((run['time'] >= t0) & (run['time'] <= t1) & run['valid'])
    total, quantisation, reference = [], [], []
    for i in window:
        x = (run['q'][i], run['dq'][i], run['dq_d'][i], run['ddq_d'][i])
        pi = run['pi'][i]
        y_r = model.regressor_r(*x)
        ideal = y_r @ pi + k_p * run['e'][i] + k_v * run['de'][i]
        total.append(np.linalg.norm(run['tau_model'][i] - ideal))
        # float32 transport: inputs and output rounded as on the wire.
        x32 = [np.asarray(v, dtype=np.float32).astype(np.float64) for v in x]
        y_r32 = model.regressor_r(*x32).astype(np.float32).astype(np.float64)
        quantisation.append(np.linalg.norm((y_r32 - y_r) @ pi))
        reference.append(np.linalg.norm(y_r @ pi))
    return np.array(total), np.array(quantisation), np.array(reference)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--local', required=True)
    parser.add_argument('--server', required=True)
    parser.add_argument('--out', required=True)
    # The gains of the recorded runs, fitted back from tau_model (debug_gains.py);
    # the config file in the workspace has since been changed.
    parser.add_argument('--kp', type=float, nargs=7, default=[50, 50, 50, 50, 15, 15, 4])
    parser.add_argument('--kv', type=float, nargs=7, default=[5, 5, 5, 5, 1.5, 1.5, 0.4])
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    model = ThunderModel(default_library(), default_parameter_file())
    gains = (np.asarray(args.kp), np.asarray(args.kv))

    runs = {}
    for name, bag in (('local', args.local), ('server', args.server)):
        run = read_run(bag)
        messages = read_state(bag)
        run['tau_cmd'] = np.array([m.tau_cmd for m in messages])
        run['q_d'] = run['q'] + run['e']
        run['age'] = np.array([m.model_age_ms for m in messages])
        run['valid'] = np.array([m.model_valid for m in messages])
        runs[name] = run

    t0, t1 = adaptation_window(runs['local'])
    s0, s1 = adaptation_window(runs['server'])
    t0, t1 = max(t0, s0), min(t1, s1)

    diff = trajectory_difference(runs['local'], runs['server'], t0, t1)
    np.savez(os.path.join(args.out, 'trajectory_difference.npz'), **diff)

    summary = {'window': [t0, t1]}
    dq = np.linalg.norm(diff['q'], axis=1)
    summary['q_diff_norm_rad'] = percentiles(dq)
    summary['q_diff_rms_per_joint_rad'] = np.sqrt((diff['q'] ** 2).mean(axis=0)).tolist()
    summary['q_diff_max_per_joint_rad'] = np.abs(diff['q']).max(axis=0).tolist()
    summary['q_d_diff_max_rad'] = float(np.abs(diff['q_d']).max())
    summary['e_local_norm_rad'] = percentiles(np.linalg.norm(diff['e_local'], axis=1))
    summary['tau_cmd_diff_norm_Nm'] = percentiles(np.linalg.norm(diff['tau_cmd'], axis=1))

    for name, run in runs.items():
        total, quant, ref = torque_deviation(run, model, gains, t0, t1)
        summary[f'{name}_torque_deviation_Nm'] = percentiles(total)
        summary[f'{name}_float32_deviation_Nm'] = percentiles(quant)
        summary[f'{name}_model_torque_norm_Nm'] = percentiles(ref)
        window = (run['time'] >= t0) & (run['time'] <= t1)
        if name == 'server':
            age = run['age'][window & run['valid']]
            summary['server_model_age_ms'] = percentiles(age)
            summary['server_invalid_samples_in_window'] = int((~run['valid'][window]).sum())
        np.save(os.path.join(args.out, f'{name}_torque_deviation.npy'), total)

    with open(os.path.join(args.out, 'summary.json'), 'w') as handle:
        json.dump(summary, handle, indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
