#!/usr/bin/env python3
# Copyright 2026 Giorgio Simonini
# Licensed under the Apache License, Version 2.0.

"""
Summarise a recorded MACT run, or compare two of them.

    ros2 run controller_tests analyse_run.py bags/gazebo_local_20260918_150000
    ros2 run controller_tests analyse_run.py bags/gazebo_local_... bags/gazebo_server_...

It reads /mact/state and reports the quantities the comparison is about: the
tracking error, the loop timing, how often the controller had to fall back to
the PD term, and how far the parameter estimate travelled. For a closer look at
the loop timing -- why a run lost the robot -- see analyse_timing.py.

With --csv it writes the per-sample series out, so the figures can be drawn
with whatever plotting tool you prefer.
"""

import argparse
import csv
import os
import struct
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

STATE_TOPIC = '/mact/state'
STATE_TYPE = 'mact_msgs/msg/MactState'


class _CdrReader:
    """Just enough of a CDR decoder for decode_legacy_state()."""

    def __init__(self, data):
        # 4-byte encapsulation header; its second byte says the endianness.
        self._prefix = '<' if data[1] == 1 else '>'
        self._body = memoryview(data)[4:]
        self._offset = 0

    def _take(self, code, size):
        self._offset = (self._offset + size - 1) // size * size
        value = struct.unpack_from(self._prefix + code, self._body, self._offset)[0]
        self._offset += size
        return value

    def double(self):
        return self._take('d', 8)

    def doubles(self, count):
        return [self.double() for _ in range(count)]

    def int32(self):
        return self._take('i', 4)

    def uint32(self):
        return self._take('I', 4)

    def boolean(self):
        return bool(self._take('B', 1))

    def string(self):
        length = self.uint32()
        raw = bytes(self._body[self._offset:self._offset + length])
        self._offset += length
        return raw.rstrip(b'\0').decode()


def decode_legacy_state(data, message_type):
    """
    Decode a MactState recorded before the per-stage timing existed.

    Humble's bags keep the type's name but not its definition, so those
    messages no longer match the current one: stage_duration_mean_us and
    stage_duration_max_us were added in the middle of it. They are decoded here
    field by field, in the old order, and the stage arrays are left at zero.
    """
    reader = _CdrReader(data)
    message = message_type()
    message.header.stamp.sec = reader.int32()
    message.header.stamp.nanosec = reader.uint32()
    message.header.frame_id = reader.string()
    for field in ('q', 'dq', 'ddq', 'q_d', 'dq_d', 'ddq_d', 'e', 'de',
                  'tau_model', 'tau_cmd', 'tau_meas', 'tau_gravity'):
        setattr(message, field, reader.doubles(7))
    message.pi_hat = reader.doubles(reader.uint32())
    message.update_duration_min_us = reader.double()
    message.update_duration_mean_us = reader.double()
    message.update_duration_max_us = reader.double()
    message.cycles_in_window = reader.uint32()
    message.model_valid = reader.boolean()
    message.model_age_ms = reader.double()
    message.degraded_cycles = reader.uint32()
    message.trajectory_valid = reader.boolean()
    message.adaptation_enabled = reader.boolean()
    message.trajectory_time = reader.double()
    return message


def has_stage_timing(messages):
    """False for runs recorded before the per-stage timing existed."""
    return any(any(m.stage_duration_max_us) for m in messages)


def read_topics(bag_path, topics, required=()):
    """
    Return {topic: [(receive_time_s, message), ...]} for the topics present.

    The receive time is the recorder's clock, i.e. when the sample reached the
    bag, not when it was produced; the header stamps are the producer's. Topics
    in `required` that are missing abort; the others are simply left out.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in required:
        if topic not in types:
            raise SystemExit(f"'{topic}' is not in {bag_path}; recorded: {sorted(types)}")
    present = [topic for topic in topics if topic in types]
    if not present:
        return {}

    message_types = {topic: get_message(types[topic]) for topic in present}
    reader.set_filter(rosbag2_py.StorageFilter(topics=present))

    samples = {topic: [] for topic in present}
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        message_type = message_types[topic]
        try:
            message = deserialize_message(data, message_type)
        except Exception:  # noqa: BLE001 -- rclpy raises a bare RMWError
            if types[topic] != STATE_TYPE:
                raise
            message = decode_legacy_state(data, message_type)
        samples[topic].append((receive_ns * 1e-9, message))
    return samples


def read_state(bag_path, topic=STATE_TOPIC):
    """Return the deserialised messages of `topic`, in recording order."""
    samples = read_topics(bag_path, [topic], required=[topic])
    return [message for _, message in samples[topic]]


def summarise(messages, rate_hz=1000.0):
    """Collect the series the report and the CSV are built from."""
    time = np.array([m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for m in messages])
    time -= time[0] if len(time) else 0.0
    cycles = np.array([m.cycles_in_window for m in messages], dtype=float)
    # Cycles the time between two samples calls for but that were not counted,
    # i.e. that did not run. The first window starts at the activation, whose
    # time is not in the bag.
    missed = np.zeros(len(time))
    missed[1:] = np.maximum(np.rint(np.diff(time) * rate_hz) - cycles[1:], 0.0)

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
        'cycles': cycles,
        'missed': missed,
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
    print(f'  missed cycles      : {int(data["missed"].sum())} '
          f'(worst window {int(data["missed"].max())})')
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
              f'{"mean update [us]":>18s} {"worst [us]":>12s} {"missed":>8s}')
        for name, data in summaries.items():
            print(f'{name:40s} {np.sqrt(np.mean(data["error_norm"]**2)):14.5f} '
                  f'{data["error_norm"].max():14.5f} '
                  f'{data["duration_mean_us"].mean():18.1f} '
                  f'{data["duration_max_us"].max():12.1f} '
                  f'{int(data["missed"].sum()):8d}')


if __name__ == '__main__':
    main()
