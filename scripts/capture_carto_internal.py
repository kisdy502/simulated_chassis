#!/usr/bin/env python3
"""Read-only Cartographer state snapshots and synchronous topic observations.

Requires Cartographer protobuf Python modules on PYTHONPATH, numpy and scipy.
write_state serializes current state; it does not finish trajectories or optimize.
Local SLAM data is the latest inserted node, not every scan matching result.
The reconstructed local published pose is already projected to 2D by Cartographer.
"""
import argparse
import collections
import gzip
import json
import struct
import time
import sys
from pathlib import Path

dependency_parser = argparse.ArgumentParser(add_help=False)
dependency_parser.add_argument("--proto-path")
dependency_args, _ = dependency_parser.parse_known_args()
if dependency_args.proto_path:
    sys.path.insert(0, dependency_args.proto_path)

import numpy as np
import rclpy
from cartographer.mapping.proto import serialization_pb2
from cartographer_ros_msgs.srv import GetTrajectoryStates, WriteState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu


def matrix(pose):
    m = np.eye(4)
    q = pose.rotation if hasattr(pose, "rotation") else pose.orientation
    t = pose.translation if hasattr(pose, "translation") else pose.position
    m[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def describe(m):
    return {"xyz": m[:3, 3].tolist(),
            "rpy_deg": Rotation.from_matrix(m[:3, :3]).as_euler("xyz", degrees=True).tolist()}


def varint(raw, offset):
    value = shift = 0
    while True:
        byte = raw[offset]
        offset += 1
        value |= (byte & 127) << shift
        if byte < 128:
            return value, offset
        shift += 7


def trajectory_id_from_record(raw):
    # Submap/Node payloads start with their ID message. Inspect only that small
    # message, avoiding decoding frozen-map point clouds and voxel grids.
    key, offset = varint(raw, 0)
    assert key == 10
    size, offset = varint(raw, offset)
    stop = offset + size
    while offset < stop:
        key, offset = varint(raw, offset)
        assert key & 7 == 0
        value, offset = varint(raw, offset)
        if key >> 3 == 1:
            return value
    return 0  # protobuf omits the default trajectory ID


def decode(filename, trajectory_id):
    locals_, nodes, trajectory_data = {}, {}, None
    with open(filename, "rb") as stream:
        assert struct.unpack("<Q", stream.read(8))[0] == 0x7b1d1f7b5bf501db
        index = 0
        graph = None
        while size := stream.read(8):
            raw = gzip.decompress(stream.read(struct.unpack("<Q", size)[0]))
            if index == 0:
                index += 1
                continue
            key, offset = varint(raw, 0)
            field = key >> 3
            if field not in (1, 3, 4, 5):
                continue
            if field in (3, 4):
                size, offset = varint(raw, offset)
                if trajectory_id_from_record(raw[offset:offset + size]) != trajectory_id:
                    continue
            data = serialization_pb2.SerializedData.FromString(raw)
            index += 1
            kind = data.WhichOneof("data")
            if kind == "pose_graph":
                graph = data.pose_graph
            elif kind == "submap" and data.submap.submap_id.trajectory_id == trajectory_id:
                locals_[data.submap.submap_id.submap_index] = matrix(data.submap.submap_3d.local_pose)
            elif kind == "node" and data.node.node_id.trajectory_id == trajectory_id:
                nodes[data.node.node_id.node_index] = data.node.node_data
            elif kind == "trajectory_data" and data.trajectory_data.trajectory_id == trajectory_id:
                trajectory_data = data.trajectory_data
    trajectory = next(t for t in graph.trajectory if t.trajectory_id == trajectory_id)
    submap = max(trajectory.submap, key=lambda s: s.submap_index)
    # Same transform as PoseGraph3D::ComputeLocalToGlobalTransform. Unoptimized
    # submap global poses are extrapolated by this same transform during serialization.
    local_to_map = matrix(submap.pose) @ np.linalg.inv(locals_[submap.submap_index])
    node_index = max(nodes)
    node = nodes[node_index]
    global_node = next(n for n in trajectory.node if n.node_index == node_index)
    result = {"trajectory_id": trajectory_id, "submap_index": submap.submap_index,
              "node_index": node_index,
              "node_sim_time": (node.timestamp - 621355968000000000) / 1e7,
              "local_slam_node": describe(matrix(node.local_pose)),
              "global_node": describe(matrix(global_node.pose)),
              "local_to_map": describe(local_to_map),
              "gravity_alignment": describe(np.block([
                  [Rotation.from_quat([node.gravity_alignment.x, node.gravity_alignment.y,
                                      node.gravity_alignment.z, node.gravity_alignment.w]).as_matrix(), np.zeros((3, 1))],
                  [np.zeros((1, 3)), np.ones((1, 1))]])),
              "cross_trajectory_constraints": sum(
                  c.node_id.trajectory_id == trajectory_id and
                  c.submap_id.trajectory_id != trajectory_id for c in graph.constraint)}
    if trajectory_data:
        result["optimized_gravity_constant"] = trajectory_data.gravity_constant
    return result, local_to_map


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--interval", type=float, default=30.)
    parser.add_argument("--snapshots", type=int, default=5)
    parser.add_argument("--proto-path", help="Directory containing generated Cartographer protobuf modules")
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("cartographer_internal_state_probe")
    states = node.create_client(GetTrajectoryStates, "/get_trajectory_states")
    writer = node.create_client(WriteState, "/write_state")
    samples = collections.defaultdict(list)
    latest_pose = None
    start = time.monotonic()
    stream = (out / "topics.jsonl").open("w")

    def record(topic, msg, values):
        row = {"wall": time.monotonic() - start,
               "sim": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
               "values": values}
        samples[topic].append(row)
        stream.write(json.dumps({"topic": topic, **row}) + "\n")

    def pose_callback(msg):
        nonlocal latest_pose
        latest_pose = msg
        record("tracked_pose", msg, describe(matrix(msg.pose)))

    subscriptions = [
        node.create_subscription(PoseStamped, "/tracked_pose", pose_callback, qos_profile_sensor_data),
        node.create_subscription(Imu, "/imu", lambda m: record("imu", m, {
            "gyro": [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z],
            "accel": [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]}), qos_profile_sensor_data),
        node.create_subscription(Odometry, "/odom", lambda m: record("odom", m, {
            "pose": describe(matrix(m.pose.pose)),
            "velocity": [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z]}), qos_profile_sensor_data)]

    def call(client, request):
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError("Cartographer service unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=40.)
        if not future.done():
            raise RuntimeError("Cartographer service timed out")
        result = future.result()
        if result.status.code != 0:
            raise RuntimeError(result.status.message)
        return result

    results = []
    try:
        state = call(states, GetTrajectoryStates.Request()).trajectory_states
        active = [i for i, s in zip(state.trajectory_id, state.trajectory_state) if s == 0]
        if len(active) != 1:
            raise RuntimeError(f"Expected one active trajectory, got {active}")
        for index in range(args.snapshots):
            deadline = start + index * args.interval
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            before = latest_pose
            request = WriteState.Request()
            request.filename = str(out / f"state_{index}.pbstream")
            request.include_unfinished_submaps = True
            call(writer, request)
            result, local_to_map = decode(request.filename, active[0])
            result["wall_seconds"] = time.monotonic() - start
            result["tracked_pose_before_snapshot"] = describe(matrix(before.pose)) if before else None
            if latest_pose:
                result["tracked_sim_time"] = latest_pose.header.stamp.sec + latest_pose.header.stamp.nanosec * 1e-9
                result["tracked_pose"] = describe(matrix(latest_pose.pose))
                result["published_local_projected_pose"] = describe(
                    np.linalg.inv(local_to_map) @ matrix(latest_pose.pose))
            results.append(result)
            (out / "snapshots.json").write_text(json.dumps(results, indent=2))
            stream.flush()
            print(json.dumps(result), flush=True)
    finally:
        stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
