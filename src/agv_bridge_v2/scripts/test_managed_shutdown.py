#!/usr/bin/env python3
"""Regression for interruptible shutdown and orphaned process-group cleanup.

Run after sourcing the build. Uses fake launch children and DDS domain 94.
"""
import os
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '94'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose, Spin, FollowPath
from ament_index_python.packages import get_package_prefix


def owned(token):
    marker = ('MANAGED_SHUTDOWN_TEST=' + token).encode()
    result = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if marker in (entry / 'environ').read_bytes().split(b'\0'):
                result.append(int(entry.name))
        except (OSError, ProcessLookupError):
            pass
    return result


def main():
    rclpy.init()
    node = Node('shutdown_test_fake_actions')
    servers = [ActionServer(node, kind, name, lambda handle: None) for kind, name in (
        (NavigateToPose, '/navigate_to_pose'), (Spin, '/spin'), (FollowPath, '/follow_path'))]
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    executable = Path(get_package_prefix('agv_bridge_v2')) / 'lib/agv_bridge_v2/agv_nav_server'
    try:
        for stubborn in (False, True):
            with tempfile.TemporaryDirectory(prefix='managed-shutdown-') as directory:
                folder = Path(directory)
                (folder / 'bin').mkdir(); (folder / 'maps').mkdir()
                (folder / 'fake_map.pbstream').write_bytes(b'fake: no real map is loaded')
                launcher = folder / 'bin' / 'ros2'
                launcher.write_text('''#!/usr/bin/python3
import os, signal, subprocess, sys, time
stubborn = os.environ['STUBBORN_TEST'] == 'true'
child = subprocess.Popen([sys.executable, '-c',
    'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(120)'
    if stubborn else 'import time; time.sleep(120)'])
open(os.environ['CHILD_MARKER'], 'w').write(str(child.pid))
if stubborn:
    time.sleep(.2)
    sys.exit(0)  # leave the grandchild behind deliberately
def stop(sig, frame):
    if child.poll() is None:
        child.send_signal(sig)
    child.wait()
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
while True: time.sleep(.1)
''')
                launcher.chmod(0o755)
                token = folder.name
                env = dict(os.environ, PATH=str(folder / 'bin') + ':' + os.environ['PATH'],
                    MANAGED_SHUTDOWN_TEST=token, STUBBORN_TEST=str(stubborn).lower(),
                    CHILD_MARKER=str(folder / 'child.pid'), AGV_PROCESS_LOG_DIR=directory)
                log = open(folder / 'bridge.log', 'w+')
                process = subprocess.Popen([str(executable), '--ros-args',
                    '-p', 'robot_package:=simulated_chassis', '-p', 'manage_navigation:=true',
                    '-p', f'pbstream_file:={folder}/fake_map.pbstream', '-p', f'maps_dir:={folder}/maps'],
                    env=env, stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 30
                    while not (folder / 'child.pid').exists() and time.monotonic() < deadline:
                        if process.poll() is not None:
                            raise AssertionError('Bridge exited during initialization')
                        time.sleep(0.05)
                    assert (folder / 'child.pid').exists(), 'Managed localization never spawned'
                    child_pid = int((folder / 'child.pid').read_text())
                    time.sleep(0.3)
                    started = time.monotonic()
                    process.send_signal(signal.SIGINT)  # only bridge, not its child group
                    process.wait(timeout=15)
                    assert process.returncode == 0, process.returncode
                    deadline = time.monotonic() + 3
                    while owned(token) and time.monotonic() < deadline:
                        time.sleep(0.05)
                    assert not owned(token), owned(token)
                    assert not Path(f'/proc/{child_pid}').exists(), 'Grandchild was not reaped'
                    log.flush(); log.seek(0)
                    assert 'managed processes reaped' in log.read()
                    print(f'PASS {"orphan/stubborn grandchild" if stubborn else "waiting for unavailable service"}: '
                          f'all owned children exited in {time.monotonic()-started:.2f}s', flush=True)
                except Exception:
                    log.flush(); log.seek(0); print(log.read(), flush=True)
                    raise
                finally:
                    for pid in owned(token):
                        try: os.kill(pid, signal.SIGKILL)
                        except ProcessLookupError: pass
                    if process.poll() is None: process.wait(timeout=5)
                    log.close()
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        for server in servers: server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
