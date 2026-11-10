#!/usr/bin/env python3
"""Summarise the CSVs written by run_benchmarks.bash into one JSON."""
import csv
import glob
import json
import os
import sys

import numpy as np


def stats(values):
    v = np.asarray(values, dtype=float)
    if v.size == 0:
        return None
    return {'n': int(v.size), 'mean': float(v.mean()), 'p50': float(np.percentile(v, 50)),
            'p99': float(np.percentile(v, 99)), 'p999': float(np.percentile(v, 99.9)),
            'max': float(v.max())}


def main(directory):
    summary = {'eval': {}}
    with open(os.path.join(directory, 'eval.csv')) as handle:
        for row in csv.DictReader(handle):
            summary['eval'][row['function']] = {k: float(v) for k, v in row.items() if k != 'function'}
    # Repetitions of a case (<case>_rep<k>) are pooled; the per-repetition
    # means are kept to show the spread between them.
    pooled = {}
    for client in sorted(glob.glob(os.path.join(directory, '*_client.csv'))):
        name = os.path.basename(client)[:-len('_client.csv')]
        case_name = name.rsplit('_rep', 1)[0]
        case = pooled.setdefault(case_name, {'kinds': {}, 'extra': {}, 'cycle': [], 'period': [], 'cpu': [], 'rep_means': {}})
        kinds = {}
        with open(client) as handle:
            for row in csv.DictReader(handle):
                if row['kind'].startswith('summary_'):
                    case['extra'][row['kind'][8:]] = case['extra'].get(row['kind'][8:], 0.0) + float(row['value_us'])
                else:
                    kinds.setdefault(row['kind'], []).append(float(row['value_us']))
        for kind, values in kinds.items():
            case['kinds'].setdefault(kind, []).extend(values)
            case['rep_means'].setdefault(kind, []).append(float(np.mean(values)))
        server = np.genfromtxt(os.path.join(directory, name + '_server.csv'), delimiter=',', names=True)
        # Discard the first 5 s, before the client is up.
        steady = server[server['start_us'] > 5e6]
        case['cycle'].extend(steady['duration_us'].tolist())
        case['period'].extend(np.diff(steady['start_us']).tolist())
        case['rep_means'].setdefault('server_cycle_us', []).append(float(steady['duration_us'].mean()))
        with open(os.path.join(directory, name + '_cpu.csv')) as handle:
            case['cpu'].append(float(handle.read().split(',')[1]))
    for case_name, case in pooled.items():
        out = {kind: stats(values) for kind, values in case['kinds'].items()}
        out.update(case['extra'])
        out['server_cycle_us'] = stats(case['cycle'])
        out['server_period_us'] = stats(case['period'])
        out['server_cpu_percent'] = float(np.mean(case['cpu']))
        out['rep_means'] = case['rep_means']
        summary[case_name] = out
    with open(os.path.join(directory, 'summary.json'), 'w') as handle:
        json.dump(summary, handle, indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main(sys.argv[1])
