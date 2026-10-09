import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.request import urlopen
from urllib.error import HTTPError
from http.server import ThreadingHTTPServer
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('map3d', Path(__file__).parents[1] / 'scripts/map_3d_server.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class AssetsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.source = self.root / 'sample.pbstream'
        self.source.write_bytes(b'state')
        self.assets = module.Assets(self.root, 'unused')

    def tearDown(self):
        self.assets.pool.shutdown(wait=True)
        self.directory.cleanup()

    def test_paths_cannot_escape(self):
        for name in ('../sample', 'a/b', '..', 'a\\b', '\x00'):
            with self.assertRaises(ValueError):
                self.assets.source(name)
        outside = tempfile.TemporaryDirectory()
        try:
            p = Path(outside.name) / 'secret.pbstream'
            p.write_bytes(b'state')
            try:
                (self.root / 'secret.pbstream').symlink_to(p)
            except OSError:
                return  # Windows without symlink privilege
            with self.assertRaises(ValueError):
                self.assets.source('secret')
        finally:
            outside.cleanup()

    def test_async_export_cached_then_invalidated(self):
        calls = []
        def export(args, **kwargs):
            calls.append(args)
            Path(args[2]).write_bytes(b'ply\nformat binary_little_endian 1.0\nelement vertex 1\nend_header\n' + bytes(12))
            return type('Result', (), {'returncode': 0})()
        with patch.object(module.subprocess, 'run', side_effect=export):
            self.assertEqual(self.assets.status('sample')['status'], 'generating')
            for _ in range(100):
                state = self.assets.status('sample')
                if state['status'] == 'ready': break
                time.sleep(0.01)
            self.assertEqual(state['point_count'], 1)
            self.assertEqual(len(calls), 1)
            self.source.write_bytes(b'new state')
            self.assertEqual(self.assets.status('sample')['status'], 'generating')
            self.assets.pool.shutdown(wait=True)
            self.assertEqual(len(calls), 2)

    def test_http_status_and_binary(self):
        ply, meta = self.assets.paths(self.source)
        payload = b'ply test binary\x00'
        ply.write_bytes(payload)
        meta.write_text(json.dumps({'status': 'ready', 'version': self.assets.signature(self.source)}))
        server = ThreadingHTTPServer(('127.0.0.1', 0), module.handler_for(self.assets))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            with urlopen(base + '/maps/sample/status') as response:
                self.assertEqual(json.load(response)['status'], 'ready')
            with urlopen(base + '/maps/sample/cloud.ply') as response:
                self.assertEqual(response.read(), payload)
                self.assertEqual(int(response.headers['Content-Length']), len(payload))
            with self.assertRaises(HTTPError) as exc:
                urlopen(base + '/maps/missing/status')
            self.assertEqual(exc.exception.code, 404)
            exc.exception.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
