#!/usr/bin/env python3
"""Read-only 3D asset API; exports run serially outside the navigation process."""
import argparse
import concurrent.futures
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit


class Assets:
    def __init__(self, root, exporter, voxel=0.1):
        self.root = Path(root).resolve()
        self.exporter = exporter
        self.voxel = voxel
        self.lock = threading.Lock()
        self.jobs = {}
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)

    def source(self, name):
        if not name or len(name) > 128 or '..' in name or any(c in name for c in '/\\\x00'):
            raise ValueError('Invalid map name')
        nested = self.root / name / (name + '.pbstream')
        p = nested if nested.is_file() else self.root / (name + '.pbstream')
        if not p.resolve().is_relative_to(self.root):
            raise ValueError('Map path escapes maps directory')
        if not p.is_file():
            raise FileNotFoundError(name)
        return p

    def paths(self, source):
        paths = source.with_suffix('.ply'), source.with_suffix('.3d.json')
        if any(not p.resolve().is_relative_to(self.root) for p in paths):
            raise ValueError('Asset path escapes maps directory')
        return paths

    def signature(self, source):
        stat = source.stat()
        return f'{stat.st_mtime_ns}:{stat.st_size}:{self.voxel}:0.55:v1'

    def status(self, name):
        source = self.source(name)
        signature = self.signature(source)
        ply, meta = self.paths(source)
        with self.lock:
            if name in self.jobs:
                return self.jobs[name]
            if meta.is_file():
                try:
                    state = json.loads(meta.read_text())
                    if state.get('version') == signature and (state['status'] != 'ready' or ply.is_file()):
                        return state
                except (ValueError, KeyError):
                    pass
            state = {'name': name, 'status': 'generating', 'frame_id': 'map', 'version': signature}
            self.jobs[name] = state
            self.pool.submit(self.export, name, source, signature)
            return state

    def export(self, name, source, signature):
        ply, meta = self.paths(source)
        state = {'name': name, 'frame_id': 'map', 'version': signature, 'voxel_size': self.voxel}
        try:
            result = subprocess.run([self.exporter, str(source), str(ply), str(self.voxel)],
                                    capture_output=True, text=True, timeout=300)
            if result.returncode:
                raise RuntimeError((result.stderr or result.stdout)[-2000:])
            if self.signature(source) != signature:
                raise RuntimeError('Map changed while exporting; request again')
            with ply.open('rb') as stream:
                header = stream.read(1024).split(b'end_header\n')[0].decode('ascii')
            count = int(re.search(r'element vertex (\d+)', header).group(1))
            state.update(status='ready', point_count=count, size_bytes=ply.stat().st_size)
        except Exception as exc:
            state.update(status='failed', message=str(exc))
        temporary = meta.with_suffix('.json.tmp')
        try:
            temporary.write_text(json.dumps(state, ensure_ascii=False), encoding='utf-8')
            os.replace(temporary, meta)
        finally:
            with self.lock:
                self.jobs.pop(name, None)

    def watch(self, stopped):
        seen = {}
        while not stopped.wait(10):
            for source in list(self.root.glob('*.pbstream')) + list(self.root.glob('*/*.pbstream')):
                try:
                    name = source.stem
                    signature = self.signature(source)
                    # Require two identical observations to avoid reading an active write.
                    if seen.get(str(source)) == signature:
                        self.status(name)
                    seen[str(source)] = signature
                except (OSError, ValueError):
                    continue


def handler_for(assets):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            try:
                parts = unquote(urlsplit(self.path).path).strip('/').split('/')
                if len(parts) != 3 or parts[0] != 'maps' or parts[2] not in ('status', 'cloud.ply'):
                    self.send_error(404)
                    return
                name = parts[1]
                state = assets.status(name)
                if parts[2] == 'status':
                    body = json.dumps(state, ensure_ascii=False).encode()
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json; charset=utf-8')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif state['status'] != 'ready':
                    self.send_error(409, '3D map is not ready')
                else:
                    path, _ = assets.paths(assets.source(name))
                    # fstat the opened file so atomic replacement cannot mismatch length.
                    with path.open('rb') as stream:
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/octet-stream')
                        self.send_header('Content-Length', str(os.fstat(stream.fileno()).st_size))
                        self.send_header('Cache-Control', 'no-cache')
                        self.end_headers()
                        shutil.copyfileobj(stream, self.wfile)
            except FileNotFoundError:
                self.send_error(404, 'PBStream map not found')
            except ValueError:
                self.send_error(400, 'Invalid map name')
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                print(f'3D asset request failed: {exc}', flush=True)
                self.send_error(500)
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--maps-dir', required=True)
    parser.add_argument('--address', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8089)
    parser.add_argument('--voxel', type=float, default=0.1)
    parser.add_argument('--exporter', default=str(Path(__file__).absolute().with_name('pbstream_to_ply')))
    args = parser.parse_args()
    if not 0 < args.voxel < 10:
        parser.error('voxel must be between 0 and 10 meters')
    assets = Assets(args.maps_dir, args.exporter, args.voxel)
    stopped = threading.Event()
    server = ThreadingHTTPServer((args.address, args.port), handler_for(assets))
    watcher = threading.Thread(target=assets.watch, args=(stopped,), daemon=True)
    watcher.start()
    def terminate(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    print(f'3D maps: {assets.root}; HTTP {args.address}:{args.port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        server.server_close()
        assets.pool.shutdown(wait=True)


if __name__ == '__main__':
    main()
