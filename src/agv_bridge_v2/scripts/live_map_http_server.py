#!/usr/bin/env python3
"""Serve the latest ROS occupancy grid on demand, outside rosbridge."""
import argparse
import gzip
import json
import sys
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.utilities import remove_ros_args
from rclpy.qos import DurabilityPolicy, QoSProfile


class LatestGrid:
    def __init__(self):
        self.lock = threading.Lock()
        self.instance = uuid.uuid4().hex
        self.version = 0
        self.message = None
        self.encoded = None

    def update(self, message):
        if not message.data or len(message.data) != message.info.width * message.info.height:
            return
        with self.lock:
            self.message = message
            self.version += 1
            self.encoded = None

    def get(self, known_etag):
        with self.lock:
            if self.message is None:
                return 503, None, b'{"error":"map not available"}'
            etag = f'"{self.instance}-{self.version}"'
            if known_etag == etag:
                return 304, etag, b""
            if self.encoded is None:
                msg = self.message
                info = msg.info
                pos, rot = info.origin.position, info.origin.orientation
                payload = {
                    "header": {"frame_id": msg.header.frame_id},
                    "info": {
                        "width": info.width, "height": info.height, "resolution": info.resolution,
                        "origin": {
                            "position": {"x": pos.x, "y": pos.y, "z": pos.z},
                            "orientation": {"x": rot.x, "y": rot.y, "z": rot.z, "w": rot.w},
                        },
                    },
                    "data": list(msg.data),
                }
                self.encoded = gzip.compress(json.dumps(payload, separators=(",", ":")).encode(),
                                             compresslevel=1, mtime=0)
            return 200, etag, self.encoded


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/maps/live":
            self.send_error(404)
            return
        status, etag, body = self.server.grid.get(self.headers.get("If-None-Match"))
        self.send_response(status)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Type", "application/json")
        if etag is not None:
            self.send_header("ETag", etag)
        if status == 200:
            self.send_header("Content-Encoding", "gzip")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if body:
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def log_message(self, *_args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8092)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    grid = LatestGrid()
    with ThreadingHTTPServer((args.address, args.port), Handler) as server:
        server.grid = grid
        rclpy.init()
        node = rclpy.create_node("live_map_http_server")
        node.create_subscription(
            OccupancyGrid, "/map", grid.update,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            server.shutdown()
            worker.join()
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
