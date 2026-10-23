#!/usr/bin/env python3
"""Bounded, passive ROS sample. Each invocation destroys its node and exits."""
import argparse
import datetime as dt
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def summarize(directory, until):
    records = [json.loads(line) for line in (directory / 'samples.jsonl').read_text().splitlines()]
    lines = ['# TF / costmap sampling', '',
             f'End time: {until.isoformat()}. Each sample is passive and releases its ROS node.',
             'Intermittent samples cannot exclude faults between sampling windows.', '',
             '| Time (Asia/Shanghai) | Result | TF stamp | Local footprint stamp | Max local age (s) |',
             '| --- | --- | ---: | ---: | ---: |']
    for record in records:
        streams = record.get('streams', {})
        tf = streams.get('odom_tf', {})
        local = streams.get('local_footprint', {})
        lines.append(f"| {record['time']} | {', '.join(record['issues']) or 'OK'} | "
                     f"{tf.get('last', '-')} | {local.get('last', '-')} | {local.get('max_age', '-')} |")
    (directory / 'SUMMARY.md').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--until', required=True)
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    until = dt.datetime.fromisoformat(args.until)
    directory = Path(args.output)
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'sample.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if dt.datetime.now(dt.timezone.utc) >= until:
            if (directory / 'samples.jsonl').exists():
                summarize(directory, until)
            print('Monitoring deadline reached; no ROS node started.')
            return
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import PolygonStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import PointCloud2

        record = {'time': dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat(),
                  'pid': os.getpid(), 'streams': {}, 'issues': []}
        latest_clock = [None]
        stop = [False]
        node = None
        rclpy.init()
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.__setitem__(0, True))
        started = time.monotonic()
        try:
            node = rclpy.create_node('bounded_nav_tf_sample')

            def observe(name, stamp):
                value = stamp.sec + stamp.nanosec / 1e9
                item = record['streams'].setdefault(name, {'count': 0, 'first': value,
                                                         'duplicates': 0, 'backwards': 0})
                if item['count']:
                    item['duplicates'] += value == item['last']
                    item['backwards'] += value < item['last']
                item['count'] += 1
                item['last'] = value
                if latest_clock[0] is not None and name != 'clock':
                    age = latest_clock[0] - value
                    item['max_age'] = round(max(item.get('max_age', age), age), 6)
                    item['min_age'] = round(min(item.get('min_age', age), age), 6)

            def clock_callback(message):
                latest_clock[0] = message.clock.sec + message.clock.nanosec / 1e9
                observe('clock', message.clock)

            def tf_callback(message):
                for transform in message.transforms:
                    if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint':
                        observe('odom_tf', transform.header.stamp)

            node.create_subscription(Clock, '/clock', clock_callback, qos_profile_sensor_data)
            node.create_subscription(TFMessage, '/tf', tf_callback, qos_profile_sensor_data)
            topics = [(PolygonStamped, '/local_costmap/published_footprint', 'local_footprint'),
                      (PolygonStamped, '/global_costmap/published_footprint', 'global_footprint'),
                      (Odometry, '/odom', 'odom'),
                      (PointCloud2, '/points2_1', 'front_cloud'),
                      (PointCloud2, '/points2_2', 'rear_cloud')]
            for msg_type, topic, key in topics:
                node.create_subscription(msg_type, topic,
                                         lambda message, name=key: observe(name, message.header.stamp),
                                         qos_profile_sensor_data)
            duration = min(max(args.seconds, 2), 30)
            while not stop[0] and time.monotonic() - started < duration and dt.datetime.now(dt.timezone.utc) < until:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            record['node_released'] = True
        record['elapsed_wall_seconds'] = round(time.monotonic() - started, 3)
        for name in ('clock', 'odom_tf', 'local_footprint', 'global_footprint'):
            item = record['streams'].get(name)
            if not item:
                record['issues'].append(name + ': no data')
            elif item['count'] > 1 and item['first'] == item['last']:
                record['issues'].append(name + ': timestamp frozen')
            if item and (item.get('max_age', 0) > 1 or item.get('min_age', 0) < -1):
                record['issues'].append(name + ': timestamp offset >1s')
            if item and item['backwards']:
                record['issues'].append(name + ': timestamp moved backwards')
        record['packages'] = subprocess.run(
            ['dpkg-query', '-W', 'ros-humble-tf2', 'ros-humble-tf2-ros'],
            capture_output=True, text=True, timeout=5).stdout.strip()
        processes = []
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                cmd = path.read_bytes().replace(b'\0', b' ').decode(errors='replace')
                if '/nav2_controller/controller_server ' not in cmd and '/nav2_behaviors/behavior_server ' not in cmd:
                    continue
                maps = (path.parent / 'maps').read_text()
                processes.append({'pid': int(path.parent.name), 'cmd': cmd,
                                  'old_tf2_mapping': any('libtf2' in line and '(deleted)' in line
                                                        for line in maps.splitlines())})
            except (OSError, ValueError):
                continue
        record['nav2_processes'] = processes
        if any(p['old_tf2_mapping'] for p in processes):
            record['issues'].append('Nav2 still maps replaced TF2 libraries')
        with (directory / 'samples.jsonl').open('a') as output:
            output.write(json.dumps(record, ensure_ascii=False) + '\n')
        summarize(directory, until)
        print(json.dumps(record, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
