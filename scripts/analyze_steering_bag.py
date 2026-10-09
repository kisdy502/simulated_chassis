#!/usr/bin/env python3
"""Read-only analysis of the steering probe rosbag (ROS 2 Humble Python)."""
import argparse
import csv
import math
from pathlib import Path
import sqlite3
import statistics

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--start', type=float, default=0)
    parser.add_argument('--end', type=float, default=float('inf'))
    parser.add_argument('--csv', default='/tmp/agv_steering_samples.csv')
    args = parser.parse_args()
    database = next(Path(args.bag).glob('*.db3'))
    db = sqlite3.connect(f'file:{database}?mode=ro', uri=True)
    topics = {i: (name, get_message(kind)) for i, name, kind in db.execute('select id,name,type from topics') if name != '/clock'}
    latest = {}
    map_odom_yaw = 0.0
    rows = []
    prefixes = ['wheel_front', 'wheel_left', 'wheel_right']
    locations = [(0.3, 0), (-0.15, 0.2), (-0.15, -0.2)]
    for topic_id, timestamp, blob in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
        if topic_id not in topics: continue
        topic, cls = topics[topic_id]
        message = deserialize_message(blob, cls)
        t = timestamp * 1e-9
        if topic == '/tf':
            for transform in message.transforms:
                if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                    map_odom_yaw = yaw(transform.transform.rotation)
        latest[topic] = (t, message)
        if topic != '/joint_states' or not args.start <= t <= args.end: continue
        if any(name not in latest for name in ['/cmd_vel_nav','/cmd_vel','/three_wheel_base_controller/cmd_vel','/odom']): continue
        nav = latest['/cmd_vel_nav'][1]
        smooth = latest['/cmd_vel'][1]
        command = latest['/three_wheel_base_controller/cmd_vel'][1]
        odom = latest['/odom'][1]
        states = {name: (message.position[i],message.velocity[i]) for i,name in enumerate(message.name)}
        if any(p+'_steering_joint' not in states or p+'_wheel_joint' not in states for p in prefixes): continue
        row = {'wall': t, 'sim': odom.header.stamp.sec+odom.header.stamp.nanosec*1e-9,
               'nav_vx':nav.linear.x,'nav_vy':nav.linear.y,'nav_wz':nav.angular.z,
               'vx':smooth.linear.x,'vy':smooth.linear.y,'wz':smooth.angular.z,
               'input_vx':command.linear.x,'input_vy':command.linear.y,'input_wz':command.angular.z,
               'odom_vx':odom.twist.twist.linear.x,'odom_vy':odom.twist.twist.linear.y,'odom_wz':odom.twist.twist.angular.z,
               'odom_yaw':math.degrees(yaw(odom.pose.pose.orientation)),
               'map_yaw':math.degrees(math.atan2(math.sin(map_odom_yaw+yaw(odom.pose.pose.orientation)), math.cos(map_odom_yaw+yaw(odom.pose.pose.orientation))))}
        for i,(prefix,(x,y)) in enumerate(zip(prefixes, locations)):
            actual = states[prefix+'_steering_joint'][0]
            angle = math.atan2(command.linear.y+command.angular.z*x, command.linear.x-command.angular.z*y)
            candidates = [angle+k*math.pi for k in range(-2,3) if -math.pi/2-1e-5 <= angle+k*math.pi <= math.pi/2+1e-5]
            # Equivalent inverse-kinematics target. The controller's 5-degree
            # reversal hysteresis is not reconstructed, so this is a diagnostic
            # comparison, not a directly recorded command-interface value.
            target = min(candidates, key=lambda a:abs(a-actual)) if candidates else float('nan')
            row[f'steer{i}'] = math.degrees(actual)
            row[f'target{i}'] = math.degrees(target)
            row[f'error{i}'] = math.degrees(target-actual)
            row[f'wheel{i}'] = states[prefix+'_wheel_joint'][1]
        rows.append(row)
    db.close()
    if not rows:
        print('No matching samples')
        return
    with open(args.csv,'w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
    active = [r for r in rows if abs(r['vx']) > .15]
    print(f'Samples={len(rows)} translating={len(active)} CSV={args.csv}')
    buckets = {}
    start = rows[0]['wall']
    for row in active:
        key=int((row['wall']-start)//2)*2
        buckets.setdefault(key,[]).append(row)
    keys=['nav_vx','nav_vy','nav_wz','vx','vy','wz','odom_vx','odom_vy','odom_wz','map_yaw','steer0','steer1','steer2','error0','error1','error2']
    print('seconds '+' '.join(keys))
    for key,group in buckets.items():
        print(f'{key:3d} '+' '.join(f'{statistics.mean(r[k] for r in group):+.3f}' for k in keys))
    for key in ['nav_vy','vy','wz','odom_wz','error0','error1','error2']:
        values=[r[key] for r in active if math.isfinite(r[key])]
        if values: print(f'{key}: mean={statistics.mean(values):+.4f} min={min(values):+.4f} max={max(values):+.4f}')


if __name__ == '__main__':
    main()
