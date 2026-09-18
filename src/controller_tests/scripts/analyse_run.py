#!/usr/bin/env python3
# Copyright 2026 Giorgio Simonini
# Licensed under the Apache License, Version 2.0.

"""
Summarise a recorded MACT run, or compare two of them.

    ros2 run controller_tests analyse_run.py bags/gazebo_local_20260918_150000
    ros2 run controller_tests analyse_run.py bags/gazebo_local_... bags/gazebo_server_...

It reads /mact/state and reports the quantities the comparison is about: the
tracking error, the loop timing, how often the controller had to fall back to
the PD term, and how far the parameter estimate travelled.

With --csv it writes the per-sample series out, so the figures can be drawn
with whatever plotting tool you prefer.
"""

import argparse
import csv
import os
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

STATE_TOPIC = '/mact/state'


def read_state(bag_path, topic=STATE_TOPIC):
    """Return the deserialised messages of `topic`, in recording order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f"'{topic}' is not in {bag_path}; recorded: {sorted(types)}")

    message_type = get_message(types[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    messages = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        messages.append(deserialize_message(data, message_type))
    return messages


def summarise(messages):
    """Collect the series the report and the CSV are built from."""
    time = np.array([m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for m in messages])
    time -= time[0] if len(time) else 0.0

    error = np.array([m.e for m in messages])              # N x 7
    torque = np.array([m.tau_cmd for m in messages])       # N x 7
    estimate = np.array([m.pi_hat for m in messages])      # N x P

    return {
        'time': time,
        'error': error,
        'error_norm': np.linalg.norm(error, axis=1),
        'torque': torque,
        'estimate': estimate,
        'duration_mean_us': np.array([m.update_duration_mean_us for m in messages]),
        'duration_max_us': np.array([m.update_duration_max_us for m in messages]),
        'model_valid': np.array([m.model_valid for m in messages]),
        'model_age_ms': np.array([m.model_age_ms for m in messages]),
        'degraded_cycles': np.array([m.degraded_cycles for m in messages]),
        'cycles': np.array([m.cycles_in_window for m in messages]),
    }


def report(name, data):
    total_cycles = int(data['cycles'].sum())
    degraded = int(data['degraded_cycles'][-1]) if len(data['degraded_cycles']) else 0
    estimate = data['estimate']

    print(f'\n=== {name} ===')
    print(f'  samples            : {len(data["time"])} over {data["time"][-1]:.1f} s')
    print(f'  control cycles     : {total_cycles}')
    print(f'  tracking error |e| : rms {np.sqrt(np.mean(data["error_norm"]**2)):.5f} rad, '
          f'max {data["error_norm"].max():.5f} rad')
    print('  per joint max |e|  : ' +
          ' '.join(f'{v:.4f}' for v in np.abs(data['error']).max(axis=0)))
    print(f'  torque |tau|_inf   : {np.abs(data["torque"]).max():.2f} Nm')
    print(f'  update() duration  : mean {data["duration_mean_us"].mean():.1f} us, '
          f'worst {data["duration_max_us"].max():.1f} us')
    print(f'  model available    : {100.0 * data["model_valid"].mean():.2f} % of samples')
    if data['model_age_ms'].max() > 0:
        finite = data['model_age_ms'][np.isfinite(data['model_age_ms'])]
        print(f'  model age          : mean {finite.mean():.3f} ms, max {finite.max():.3f} ms')
    print(f'  degraded cycles    : {degraded}'
          f'{"" if total_cycles == 0 else f" ({100.0 * degraded / total_cycles:.3f} %)"}')
    if len(estimate) > 1:
        drift = np.linalg.norm(estimate[-1] - estimate[0])
        print(f'  |pi_hat(T)-pi_hat(0)|: {drift:.6f}')


def write_csv(path, data):
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ['t', 'error_norm'] +
            [f'e{i + 1}' for i in range(data['error'].shape[1])] +
            [f'tau{i + 1}' for i in range(data['torque'].shape[1])] +
            ['duration_mean_us', 'duration_max_us', 'model_valid', 'model_age_ms'] +
            [f'pi{i}' for i in range(data['estimate'].shape[1])])
        for index in range(len(data['time'])):
            writer.writerow(
                [data['time'][index], data['error_norm'][index]] +
                list(data['error'][index]) + list(data['torque'][index]) +
                [data['duration_mean_us'][index], data['duration_max_us'][index],
                 int(data['model_valid'][index]), data['model_age_ms'][index]] +
                list(data['estimate'][index]))
    print(f'  csv written to {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bags', nargs='+', help='one or more recorded runs')
    parser.add_argument('--topic', default=STATE_TOPIC)
    parser.add_argument('--csv', action='store_true',
                        help='also write <bag>.csv next to each bag')
    arguments = parser.parse_args()

    summaries = {}
    for bag in arguments.bags:
        messages = read_state(bag, arguments.topic)
        if not messages:
            print(f'{bag}: no messages on {arguments.topic}', file=sys.stderr)
            continue
        data = summarise(messages)
        name = os.path.basename(os.path.normpath(bag))
        summaries[name] = data
        report(name, data)
        if arguments.csv:
            write_csv(os.path.normpath(bag) + '.csv', data)

    if len(summaries) > 1:
        print('\n=== comparison ===')
        print(f'{"run":40s} {"rms |e| [rad]":>14s} {"max |e| [rad]":>14s} '
              f'{"mean update [us]":>18s} {"worst [us]":>12s}')
        for name, data in summaries.items():
            print(f'{name:40s} {np.sqrt(np.mean(data["error_norm"]**2)):14.5f} '
                  f'{data["error_norm"].max():14.5f} '
                  f'{data["duration_mean_us"].mean():18.1f} '
                  f'{data["duration_max_us"].max():12.1f}')


if __name__ == '__main__':
    main()
