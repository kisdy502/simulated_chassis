"""Black-box tests of the production C++ HTTP server; Python is test code only."""
import concurrent.futures
import json
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from urllib.error import HTTPError
from urllib.request import build_opener, ProxyHandler, Request

SERVER = str(Path(sys.argv.pop(1)).resolve())
OPENER = build_opener(ProxyHandler({}))
EXPORTER = r'''#!/usr/bin/env python3
from pathlib import Path
import json, os, struct, subprocess, sys, time
source, target = map(Path, sys.argv[1:3])
root = Path(__file__).parent
data = source.read_bytes()
with (root/'calls').open('a') as stream:
    stream.write(source.name+'\n')
if b'BLOCK' in data:
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    (root/'pids').write_text(json.dumps([os.getpid(), child.pid]))
    time.sleep(60)
if b'FAIL_ONCE' in data and not (root/'attempted').exists():
    (root/'attempted').touch()
    sys.exit('temporary exporter failure')
if b'SLOW' in data:
    time.sleep(.35)
header = b'ply\nformat binary_little_endian 1.0\ncomment frame_id map\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
target.write_bytes(header+struct.pack('<fff', len(data), 2., 3.))
'''


class AssetAPI(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.maps = self.root/'maps'
        self.maps.mkdir()
        self.exporter = self.root/'exporter.py'
        self.exporter.write_text(EXPORTER)
        self.exporter.chmod(0o755)
        self.log = (self.root/'server.log').open('w+')
        self.process = subprocess.Popen([
            SERVER, '--maps-dir', str(self.maps), '--address', '127.0.0.1', '--port', '0',
            '--exporter', str(self.exporter), '--exporter-pcd', str(self.exporter),
            '--scan-seconds', '.05', '--settle-seconds', '.05',
            '--export-timeout', '2', '--retry-seconds', '.1'],
            stdout=subprocess.PIPE, stderr=self.log, text=True)
        line = self.process.stdout.readline()
        match = re.search(r'listening on 127.0.0.1:(\d+)', line)
        if not match:
            self.process.wait(timeout=3)
            self.log.seek(0)
            self.fail('Server startup failed: '+line+self.log.read())
        self.base = 'http://127.0.0.1:'+match.group(1)

    def tearDown(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
                self.fail('C++ server did not stop promptly')
        self.process.stdout.close()
        self.log.close()
        self.directory.cleanup()

    def source(self, name='sample', data=b'state', kind='flat'):
        if kind != 'flat':
            (self.maps/name).mkdir(exist_ok=True)
        path = (self.maps/(name+'.pbstream') if kind == 'flat' else
                self.maps/name/('global.pcd' if kind == 'pcd' else name+'.pbstream'))
        path.write_bytes(data)
        return path

    def request(self, path, method='GET'):
        return OPENER.open(Request(self.base+path, method=method), timeout=2)

    def state(self, name='sample'):
        with self.request('/maps/'+name+'/status') as response:
            return json.load(response)

    def wait(self, predicate, timeout=5):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.02)
        self.fail('Condition timed out')

    def ready(self, name='sample'):
        self.wait(lambda: self.state(name)['status'] == 'ready')
        return self.state(name)

    def calls(self):
        path = self.root/'calls'
        return path.read_text().splitlines() if path.exists() else []

    def test_lightning_is_exported_automatically_under_parent_map_name(self):
        source = self.source('station01', kind='pcd')
        meta = source.with_suffix('.3d.json')
        self.wait(lambda: meta.exists())  # no HTTP request starts the export
        state = json.loads(meta.read_text())
        self.assertEqual(state['name'], 'station01')
        self.assertEqual(state['status'], 'ready')
        self.assertFalse((self.maps/'global.3d.json').exists())

    def test_async_http_binary_cors_head_and_cache_invalidation(self):
        source = self.source(data=b'SLOW state')
        begin = time.monotonic()
        self.assertEqual(self.state()['status'], 'generating')
        self.assertLess(time.monotonic()-begin, .25)
        with self.assertRaises(HTTPError) as error:
            self.request('/maps/sample/cloud.ply')
        self.assertEqual(error.exception.code, 409)
        error.exception.close()
        first = self.ready()
        with self.request('/maps/sample/cloud.ply') as response:
            payload = response.read()
            self.assertEqual(int(response.headers['Content-Length']),len(payload))
            self.assertEqual(response.headers['Access-Control-Allow-Origin'],'*')
            self.assertEqual(struct.unpack('<fff',payload.split(b'end_header\n')[1]),(10.,2.,3.))
        with self.request('/maps/sample/cloud.ply','HEAD') as response:
            self.assertEqual(response.read(),b'')
            self.assertEqual(int(response.headers['Content-Length']),first['size_bytes'])
        with self.request('/maps/sample/status','OPTIONS') as response:
            self.assertEqual(response.status,204)
        self.assertEqual(len(self.calls()),1)
        source.write_bytes(b'new state with a different length')
        self.wait(lambda:self.state()['version'] != first['version'])
        self.ready()
        self.assertEqual(len(self.calls()),2)

    def test_paths_and_all_source_symlinks_cannot_escape(self):
        outside = self.root/'secret.pbstream'
        outside.write_bytes(b'secret')
        (self.maps/'flat.pbstream').symlink_to(outside)
        (self.maps/'nested').mkdir()
        (self.maps/'nested/nested.pbstream').symlink_to(outside)
        (self.maps/'pcd').mkdir()
        (self.maps/'pcd/global.pcd').symlink_to(outside)
        for name in ('flat','nested','pcd','..','%2e%2e','%00','%FF'):
            with self.assertRaises(HTTPError) as error:
                self.request('/maps/'+name+'/status')
            self.assertIn(error.exception.code,(400,404))
            error.exception.close()
        self.assertEqual(self.calls(),[])

    def test_output_symlink_is_rejected_without_writing_outside(self):
        self.source()
        outside = self.root/'outside.ply'
        outside.write_bytes(b'keep')
        (self.maps/'sample.ply').symlink_to(outside)
        with self.assertRaises(HTTPError) as error:
            self.request('/maps/sample/status')
        self.assertEqual(error.exception.code,400)
        error.exception.close()
        self.assertEqual(outside.read_bytes(),b'keep')

    def test_corrupt_cached_ply_is_regenerated(self):
        self.source()
        self.ready()
        (self.maps/'sample.ply').write_bytes(b'broken')
        self.ready()
        self.assertEqual(len(self.calls()),2)

    def test_source_change_during_export_never_publishes_stale_cloud(self):
        source = self.source(data=b'SLOW first')
        self.wait(lambda: len(self.calls()) == 1)
        source.write_bytes(b'second version')
        self.ready()
        with self.request('/maps/sample/cloud.ply') as response:
            xyz=struct.unpack('<fff',response.read().split(b'end_header\n')[1])
        self.assertEqual(xyz[0],len(b'second version'))
        self.assertFalse(list(self.maps.glob('*.export-*')))

    def test_transient_failure_retries_without_deleting_metadata(self):
        self.source(data=b'FAIL_ONCE')
        self.ready()
        self.assertEqual(len(self.calls()),2)

    def test_parallel_requests_share_one_serial_export(self):
        self.source(data=b'SLOW parallel')
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            states=list(pool.map(lambda _:self.state(),range(16)))
        self.assertTrue(all(state['status']=='generating' for state in states))
        self.ready()
        self.assertEqual(len(self.calls()),1)

    def test_shutdown_terminates_exporter_and_descendant(self):
        self.source(data=b'BLOCK')
        self.wait(lambda:(self.root/'pids').exists())
        pids=json.loads((self.root/'pids').read_text())
        begin=time.monotonic()
        self.process.send_signal(signal.SIGTERM)
        self.assertEqual(self.process.wait(timeout=3),0)
        self.assertLess(time.monotonic()-begin,3)
        for pid in pids:
            status=Path('/proc')/str(pid)/'stat'
            self.wait(lambda: not status.exists() or status.read_text().split()[2]=='Z')

    def test_timeout_releases_job_and_kills_process_tree(self):
        source=self.source(data=b'BLOCK')
        self.wait(lambda:self.state()['status']=='failed')
        self.assertIn('timed out',self.state()['message'])
        pids=json.loads((self.root/'pids').read_text())
        source.write_bytes(b'recovered')
        self.ready()
        for pid in pids:
            status=Path('/proc')/str(pid)/'stat'
            self.wait(lambda:not status.exists() or status.read_text().split()[2]=='Z')

    def test_read_only_output_reports_failure_instead_of_generating_forever(self):
        source=self.source('readonly',kind='pcd')
        source.parent.chmod(0o555)
        try:
            self.wait(lambda:self.state('readonly')['status']=='failed')
            self.assertTrue(self.state('readonly')['message'])
        finally:
            source.parent.chmod(0o755)

    def test_backend_selection_prevents_old_pbstream_shadowing_lightning(self):
        root=self.root/'mixed'
        (root/'same').mkdir(parents=True)
        (root/'same.pbstream').write_bytes(b'old cartographer map')
        (root/'same/global.pcd').write_bytes(b'preferred pcd')
        process=subprocess.Popen([SERVER,'--maps-dir',str(root),'--address','127.0.0.1',
            '--port','0','--source-format','pcd','--exporter-pcd',str(self.exporter),
            '--settle-seconds','0','--scan-seconds','.05'],
            stdout=subprocess.PIPE,stderr=self.log,text=True)
        try:
            match=re.search(r'listening on 127.0.0.1:(\d+)',process.stdout.readline())
            self.assertIsNotNone(match)
            url='http://127.0.0.1:'+match.group(1)+'/maps/same/status'
            def ready():
                with OPENER.open(url,timeout=2) as response:
                    return json.load(response)['status']=='ready'
            self.wait(ready)
            self.assertTrue((root/'same/global.ply').is_file())
            self.assertFalse((root/'same.ply').exists())
            self.assertEqual(self.calls(),['global.pcd'])
        finally:
            process.terminate()
            process.wait(timeout=3)
            process.stdout.close()

    def test_duplicate_root_and_invalid_arguments_fail(self):
        command=[SERVER,'--maps-dir',str(self.maps),'--address','127.0.0.1','--port','0']
        result=subprocess.run(command,capture_output=True,timeout=3)
        self.assertNotEqual(result.returncode,0)
        for value in ('nan','0','0.000000001','10','0.1garbage'):
            result=subprocess.run(command+['--voxel',value],capture_output=True,timeout=3)
            self.assertNotEqual(result.returncode,0)

if __name__ == '__main__':
    unittest.main()
