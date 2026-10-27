#!/usr/bin/env python3
"""Isolated ROS test: no Gazebo, navigation commands or production state files."""
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def main():
    rclpy.init()
    node = rclpy.create_node('battery_node_integration_test')
    odom = node.create_publisher(Odometry, '/odom', 10)
    cmd = node.create_publisher(Twist, '/cmd_vel', 10)
    readings, contacts = [], []
    node.create_subscription(BatteryState, '/battery_state', readings.append, 10)
    node.create_subscription(Bool, '/battery/charging_connected', contacts.append, 10)
    client = node.create_client(SetBool, '/battery/set_charging')
    temporary = tempfile.TemporaryDirectory()
    file = str(Path(temporary.name) / 'soc.state')
    process = None

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            odom.publish(Odometry())
            rclpy.spin_once(node, timeout_sec=0.03)

    def start():
        return subprocess.Popen([sys.argv[1], '--ros-args', '-p', 'state_file:='+file,
            '-p', 'initial_soc:=50.0', '-p', 'idle_discharge_pct_per_hour:=3600.0',
            '-p', 'moving_discharge_pct_per_hour:=7200.0', '-p', 'charge_pct_per_hour:=7200.0'])

    def call(enabled):
        assert client.wait_for_service(timeout_sec=3), 'service unavailable'
        req = SetBool.Request(); req.data = enabled
        future = client.call_async(req)
        end = time.monotonic()+3
        while not future.done() and time.monotonic()<end:
            spin(.05)
        assert future.done(), 'service timeout'
        result = future.result()
        assert result.success, result.message

    try:
        process = start()
        spin(2)
        assert readings, 'no battery telemetry'
        before = readings[-1].percentage
        spin(1.2)
        assert readings[-1].percentage < before, 'idle battery did not discharge'
        call(True)
        spin(.2)
        before = readings[-1].percentage
        spin(1.5)
        assert readings[-1].percentage > before, 'charging did not increase SOC'
        assert readings[-1].power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        # A manual motion command is a simulation failsafe, not a real undock.
        moving = Twist(); moving.linear.x = .1
        cmd.publish(moving)
        spin(.3)
        assert contacts and not contacts[-1].data, 'motion did not disconnect simulated charger'
        call(False)
        spin(.2)
        before = readings[-1].percentage
        spin(1.2)
        assert readings[-1].percentage < before, 'stop charging did not resume discharge'
        process.terminate(); process.wait(timeout=4); process=None
        saved = float(Path(file).read_text()) / 100
        readings.clear(); contacts.clear()
        process=start(); spin(1.3)
        assert readings and abs(readings[-1].percentage-saved) < .03, 'SOC reset after restart'
        assert contacts and not contacts[-1].data, 'charging connection restored unexpectedly'
        print('PASS: discharge, charge, disconnect on command, stop, persisted SOC, safe restart')
    finally:
        if process is not None:
            process.terminate()
            try: process.wait(timeout=4)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        node.destroy_node(); rclpy.shutdown(); temporary.cleanup()


if __name__ == '__main__':
    main()
