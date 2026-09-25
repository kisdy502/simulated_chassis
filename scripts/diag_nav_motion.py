#!/usr/bin/env python3
"""
导航运动一致性诊断：兜圈子/速度不一致问题一键取证。

同时采样五路数据并对比速度：
  1. /cmd_vel      最终下发速度（MPPI -> velocity_smoother 后）
  2. /cmd_vel_nav  MPPI 原始输出（未经 smoother，定位 vy 钳位源头）
  3. /odom         轮式里程计 twist
  4. TF map->base  Cartographer 修正后的位姿差分
  5. Gazebo 真值   ign dynamic_pose/info（物理真实运动，最终裁判）

用法（导航复现兜圈子时运行 20 秒）：
  python3 scripts/diag_nav_motion.py [时长s] [gazebo模型名]
  模型名默认 three_wheel_agv，可用 `ign topic -e -t /world/test_world/dynamic_pose/info`
  先看一眼实际的 name: 字段。

判读：
  - 真值 ≈ odom 但 ≈ 1/3 cmd   → 轮子滑移/机体被拖拽（查 gazebo contact、摩擦、自碰撞）
  - 真值 ≈ cmd，map->base 变慢 → Cartographer map 系畸变（定位问题，不是执行问题）
  - /cmd_vel_nav 的 vy 也钉死  → vy 钳位在 MPPI；否则在 velocity_smoother
"""
import math
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
MODEL_NAME = sys.argv[2] if len(sys.argv) > 2 else "three_wheel_agv"

gz_poses = []      # (t, x, y, yaw)
gz_lock = threading.Lock()


def yaw_from_quat(w, x, y, z):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def gz_thread_fn():
    """解析 ign topic 文本 protobuf：position/orientation 字段重名，按小节区分。"""
    proc = subprocess.Popen(
        ["ign", "topic", "-e", "-t", "/world/test_world/dynamic_pose/info"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    rec = {"name": None, "pos": {}, "ori": {}, "sec": 0, "nan": 0}
    section = None

    def flush():
        name = rec["name"]
        if (name and MODEL_NAME in name and rec["pos"] and rec["ori"]):
            t = rec["sec"] + rec["nan"] * 1e-9
            with gz_lock:
                gz_poses.append((t, rec["pos"].get("x", 0.0), rec["pos"].get("y", 0.0),
                                 yaw_from_quat(rec["ori"].get("w", 1.0),
                                               rec["ori"].get("x", 0.0),
                                               rec["ori"].get("y", 0.0),
                                               rec["ori"].get("z", 0.0))))
        rec["name"], rec["pos"], rec["ori"] = None, {}, {}

    for line in proc.stdout:
        s = line.strip()
        if s.startswith("time {"):
            flush()  # 新消息开始
            section = "time"
        elif s.startswith("position {"):
            section = "pos"
        elif s.startswith("orientation {"):
            section = "ori"
        elif s.startswith("name:"):
            rec["name"] = s.split('"')[1] if '"' in s else None
        else:
            m = re.match(r"(\w+): (-?[\d.e+-]+)$", s)
            if m and section:
                key, val = m.group(1), m.group(2)
                if section == "time" and key in ("sec", "nan"):
                    rec[key] = int(val)
                elif section == "pos" and key in ("x", "y"):
                    rec["pos"][key] = float(val)
                elif section == "ori" and key in ("x", "y", "z", "w"):
                    rec["ori"][key] = float(val)


def fit_speed(samples):
    """[(t, x, y, yaw)] -> (线速度均值, 角速度均值)；首尾差分。"""
    if len(samples) < 3:
        return None
    dist = 0.0
    for i in range(1, len(samples)):
        dist += math.hypot(samples[i][1] - samples[i - 1][1],
                           samples[i][2] - samples[i - 1][2])
    dt = samples[-1][0] - samples[0][0]
    dyaw = samples[-1][3] - samples[0][3]
    while dyaw > math.pi:
        dyaw -= 2 * math.pi
    while dyaw < -math.pi:
        dyaw += 2 * math.pi
    return dist / dt if dt > 0 else 0.0, dyaw / dt if dt > 0 else 0.0


def main():
    rclpy.init()
    node = Node("diag_nav_motion")
    cmd, cmd_nav, odom = [], [], []

    node.create_subscription(Twist, "/cmd_vel",
                             lambda m: cmd.append((time.time(), m)), 20)
    node.create_subscription(Twist, "/cmd_vel_nav",
                             lambda m: cmd_nav.append((time.time(), m)), 20)
    node.create_subscription(Odometry, "/odom",
                             lambda m: odom.append((time.time(), m)), 20)

    tfb = Buffer()
    TransformListener(tfb, node)
    map_poses = []

    gz_thread = threading.Thread(target=gz_thread_fn, daemon=True)
    gz_thread.start()

    t0 = time.time()
    while time.time() - t0 < DURATION and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tr = tfb.lookup_transform("map", "base_footprint", rclpy.time.Time())
            map_poses.append((time.time(), tr.transform.translation.x,
                              tr.transform.translation.y,
                              yaw_from_quat(tr.transform.rotation.w, 0, 0,
                                            tr.transform.rotation.z)))
        except Exception:
            pass
        time.sleep(0.02)

    def twist_stats(samples, label):
        if not samples:
            print(f"{label:<14} 无数据")
            return
        vxs = [m.linear.x for _, m in samples]
        vys = [m.linear.y for _, m in samples]
        wzs = [m.angular.z for _, m in samples]
        v = [math.hypot(a, b) for a, b in zip(vxs, vys)]
        n = len(samples)
        print(f"{label:<14} n={n} |v|={sum(v)/n:.3f} vx={sum(vxs)/n:+.3f} "
              f"vy={sum(vys)/n:+.3f} wz={sum(wzs)/n:+.3f}")
        uniq = sorted({round(y, 4) for y in vys})
        if len(uniq) == 1:
            print(f"{'':<14} ⚠ vy 恒等于 {uniq[0]}（钳位指纹！）")

    print(f"\n===== {DURATION:.0f}s 采样结果 =====")
    twist_stats(cmd, "/cmd_vel")
    twist_stats(cmd_nav, "/cmd_vel_nav")
    twist_stats([(t, m.twist.twist) for t, m in odom], "/odom twist")

    for label, arr in (("map->base", map_poses), ("Gazebo真值", gz_poses)):
        r = fit_speed(sorted(arr))
        if r:
            print(f"{label:<14} 线速度={r[0]:.3f} m/s 角速度={r[1]:+.3f} rad/s "
                  f"(样本 {len(arr)})")
        else:
            print(f"{label:<14} 样本不足（{len(arr)}）——{'检查 ign topic / 模型名' if label == 'Gazebo真值' else '检查 TF'}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
