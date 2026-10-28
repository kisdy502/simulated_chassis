#!/usr/bin/env python3
"""Read-only live Lightning/DDS/timestamp snapshot. Never calls robot services."""
import argparse
import json
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def stamp(value):
    return value.sec + value.nanosec * 1e-9


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def points(message):
    fields = {field.name: field.offset for field in message.fields}
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': [endian+'f4']*3,
                      'offsets': [fields[k] for k in ('x', 'y', 'z')], 'itemsize': message.point_step})
    values = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                        strides=(message.row_step, message.point_step))
    xyz = np.column_stack([values[k].ravel() for k in ('x', 'y', 'z')])
    return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('lightning_readonly_diagnostics')
    buffer = Buffer()
    # Keep the listener alive throughout sampling (also owns optional executor state).
    listener = TransformListener(buffer, node)
    stamps, walls = defaultdict(list), defaultdict(list)
    latest, imu_values, speeds = {}, [], []

    def receive(topic, message):
        value = message.clock if topic == '/clock' else message.header.stamp
        stamps[topic].append(stamp(value))
        walls[topic].append(time.monotonic())
        latest[topic] = message
        if topic == '/imu':
            a, g = message.linear_acceleration, message.angular_velocity
            imu_values.append([a.x, a.y, a.z, g.x, g.y, g.z])
        if topic == '/odom':
            v = message.twist.twist
            speeds.append([v.linear.x, v.linear.y, v.angular.z])

    types = {'/clock': Clock, '/imu': Imu, '/odom': Odometry, '/tracked_pose': PoseStamped,
             '/points2_1': PointCloud2, '/points2_2': PointCloud2, '/points2_merged': PointCloud2,
             '/map': OccupancyGrid}
    subscriptions = []
    for topic, kind in types.items():
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic == '/map' else qos_profile_sensor_data
        subscriptions.append(node.create_subscription(kind, topic, lambda m, topic=topic: receive(topic, m), qos))
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        report = {'topics': {}, 'transforms': {}}
        arrays = {}
        for topic in types:
            times = np.array(stamps[topic])
            info = {'samples': len(times), 'publishers': [f'{p.node_namespace}/{p.node_name}'
                    for p in node.get_publishers_info_by_topic(topic)]}
            if len(times) > 1:
                dt = np.diff(times)
                info.update(sim_dt_median=float(np.median(dt)), backwards=int((dt < 0).sum()),
                            repeats=int((dt == 0).sum()), sim_dt_max=float(dt.max()),
                            wall_hz=(len(times)-1)/(walls[topic][-1]-walls[topic][0]))
            if topic in latest and topic.startswith('/points2'):
                cloud = latest[topic]
                info.update(frame=cloud.header.frame_id, shape=[cloud.height, cloud.width],
                            fields=[f.name for f in cloud.fields])
                xyz = points(cloud)
                if topic != '/points2_merged':
                    try:
                        tf = buffer.lookup_transform('base_link', cloud.header.frame_id, Time()).transform
                        xyz = xyz @ rotation(tf.rotation).T + np.array([tf.translation.x, tf.translation.y, tf.translation.z])
                    except Exception as error:
                        info['transform_error'] = str(error)
                arrays[topic.strip('/')] = xyz
                info['finite_points'] = len(xyz)
            report['topics'][topic] = info
        for parent, child in [('base_link', 'front_lidar_link'), ('base_link', 'rear_lidar_link'),
                              ('base_link', 'imu_link'), ('map', 'odom'), ('odom', 'base_link')]:
            try:
                tf = buffer.lookup_transform(parent, child, Time()).transform
                report['transforms'][parent+'<-'+child] = {'xyz': [tf.translation.x, tf.translation.y, tf.translation.z],
                    'xyzw': [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]}
            except Exception as error:
                report['transforms'][parent+'<-'+child] = str(error)
        front, rear = np.array(stamps['/points2_1']), np.array(stamps['/points2_2'])
        if len(front) and len(rear):
            skew = np.min(np.abs(front[:, None] - rear[None, :]), axis=1)
            report['front_rear_nearest_stamp_delta_ms'] = {'median': float(np.median(skew)*1000), 'max': float(skew.max()*1000)}
            report['stamp_delta_note'] = 'Nearest monitor samples, not actual synchronizer pairs; boundary samples/drops can inflate the maximum.'
        if imu_values:
            imu = np.array(imu_values)
            report['imu_mean_acc_gyro'] = imu.mean(axis=0).tolist()
            report['imu_std_acc_gyro'] = imu.std(axis=0).tolist()
        if speeds:
            report['odom_mean_abs_vx_vy_wz'] = np.abs(speeds).mean(axis=0).tolist()
        if '/tracked_pose' in latest:
            pose = latest['/tracked_pose'].pose
            report['tracked_pose'] = {'xyz': [pose.position.x, pose.position.y, pose.position.z],
                                     'xyzw': [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]}
        args.output.mkdir(parents=True, exist_ok=True)
        if '/map' in latest:
            import cv2
            grid = latest['/map']
            occupancy = np.array(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
            arrays['map'] = occupancy
            report['map'] = {'resolution': grid.info.resolution, 'shape': list(occupancy.shape),
                'origin_xy': [grid.info.origin.position.x, grid.info.origin.position.y]}
            view = np.full(occupancy.shape, 160, dtype=np.uint8)
            view[occupancy == 0], view[occupancy >= 65] = 255, 0
            cv2.imwrite(str(args.output / 'map.png'), np.flipud(view))
            stationary = bool(speeds) and np.max(np.abs(speeds)) < 0.02
            if not stationary:
                report['scan_alignment_note'] = 'Robot moved during sampling; unsynchronized latest-sample distance is intentionally omitted.'
            if stationary and '/points2_merged' in latest and '/tracked_pose' in latest:
                xyz = arrays['points2_merged']
                xyz = xyz[(xyz[:, 2] > 0.15) & (xyz[:, 2] < 1.5) & (np.linalg.norm(xyz[:, :2], axis=1) > 1)]
                pose = latest['/tracked_pose'].pose
                xyz = xyz @ rotation(pose.orientation).T + np.array([pose.position.x, pose.position.y, pose.position.z])
                ij = np.floor((xyz[:, :2] - np.array(report['map']['origin_xy']))/grid.info.resolution).astype(int)
                ij = ij[(ij[:, 0] >= 0) & (ij[:, 0] < grid.info.width) & (ij[:, 1] >= 0) & (ij[:, 1] < grid.info.height)]
                if len(ij):
                    distance = cv2.distanceTransform((occupancy < 65).astype('uint8'), cv2.DIST_L2, 5) * grid.info.resolution
                    d = distance[ij[:, 1], ij[:, 0]]
                    report['latest_scan_to_occupied_map_m'] = {'median': float(np.median(d)), 'p90': float(np.percentile(d, 90))}
        np.savez_compressed(args.output / 'snapshot.npz', **arrays)
        (args.output / 'report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
