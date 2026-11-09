"""Smoke-test mesh and live-map routes on the unified production HTTP server."""
import gzip
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import tempfile
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

SERVER = str(Path(__import__("sys").argv.pop(1)).resolve())


class UnifiedRoutes(unittest.TestCase):
    def setUp(self):
        self.temp = Path(tempfile.mkdtemp(prefix="robot-http-routes-"))
        prefix = self.temp / "prefix"
        package_index = prefix / "share/ament_index/resource_index/packages"
        assets = prefix / "share/test_mesh/meshes"
        package_index.mkdir(parents=True)
        assets.mkdir(parents=True)
        (package_index / "test_mesh").touch()
        (assets / "model.dae").write_bytes(b"<COLLADA/>\n")
        env = os.environ.copy()
        env["AMENT_PREFIX_PATH"] = str(prefix) + ":" + env.get("AMENT_PREFIX_PATH", "")
        env["ROS_DOMAIN_ID"] = "97"
        env.pop("NATIVE_MAP_TOKEN", None)
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        self.base = f"http://127.0.0.1:{port}"
        self.server = subprocess.Popen([
            SERVER, "--maps-dir", str(self.temp / "maps"), "--address", "127.0.0.1",
            "--port", str(port), "--enable-map-3d", "false"], env=env,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.publisher = subprocess.Popen([
            "ros2", "topic", "pub", "--rate", "5", "/map", "nav_msgs/msg/OccupancyGrid",
            "{header: {frame_id: map}, info: {width: 2, height: 2, resolution: 0.5, "
            "origin: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}, "
            "data: [0, 100, -1, 50]}"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def tearDown(self):
        self.publisher.terminate()
        self.publisher.wait(timeout=5)
        self.server.send_signal(signal.SIGTERM)
        self.server.wait(timeout=5)
        self.server.stdout.close()
        self.server.stderr.close()
        self.temp and shutil.rmtree(self.temp)

    def test_mesh_and_live_map_keep_existing_wire_contract(self):
        response = None
        for _ in range(200):
            if self.server.poll() is not None:
                self.fail(self.server.stderr.read())
            try:
                response = urlopen(self.base + "/test_mesh/meshes/model.dae", timeout=0.25)
                break
            except Exception:
                time.sleep(0.05)
        self.assertIsNotNone(response, "unified server did not become ready")
        with response:
            self.assertEqual(response.read(), b"<COLLADA/>\n")
            self.assertEqual(response.headers["Content-Type"], "model/vnd.collada+xml")
            self.assertEqual(response.headers["Access-Control-Allow-Origin"], "*")

        live_response = None
        for _ in range(100):
            try:
                live_response = urlopen(self.base + "/maps/live", timeout=2)
                break
            except HTTPError as error:
                if error.code != 503:
                    raise
                time.sleep(0.05)
        self.assertIsNotNone(live_response, "ROS /map was not received")
        with live_response as response:
            encoded = response.read()
            etag = response.headers["ETag"]
            self.assertEqual(response.headers["Content-Encoding"], "gzip")
        payload = json.loads(gzip.decompress(encoded))
        self.assertEqual(payload["header"]["frame_id"], "map")
        self.assertEqual(payload["info"]["width"], 2)
        self.assertEqual(payload["data"], [0, 100, -1, 50])

        request = Request(self.base + "/maps/live", headers={"If-None-Match": etag})
        with self.assertRaises(HTTPError) as error:
            urlopen(request, timeout=2)
        self.assertEqual(error.exception.code, 304)
        self.assertEqual(error.exception.read(), b"")


if __name__ == "__main__":
    unittest.main()
