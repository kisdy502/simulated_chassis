#!/usr/bin/env python3
"""URDF mesh 静态 HTTP 服务(F1.4 / F3)。

上位机前端加载 URDF 时把 package://<pkg>/<相对路径> 重写为
http://<host>:<port>/<pkg>/<相对路径>(VITE_URDF_MESH_BASE 指向本服务)。
本进程按 ament 索引把 <pkg> 解析到 install share 目录并返回文件。

用法:
  python3 urdf_mesh_server.py --port 8090 [--host 0.0.0.0]
须在 source 过工作空间的终端/launch 中运行(依赖 AMENT_PREFIX_PATH)。
"""

import argparse
import mimetypes
import os
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

from ament_index_python.packages import get_package_share_directory


def make_handler():
    class MeshHandler(SimpleHTTPRequestHandler):
        def translate_path(self, path):
            # 只服务 /<package_name>/<相对路径>;拒绝 .. 防穿越
            parts = [p for p in path.split('/') if p]
            if len(parts) < 2:
                return None
            pkg, rel = parts[0], os.path.join(*parts[1:])
            if '..' in parts:
                return None
            try:
                base = get_package_share_directory(pkg)
            except Exception:
                return None
            # 注意不能用 realpath:--symlink-install 下 share 内文件是符号链接,
            # realpath 会解析到 src 目录导致前缀校验误判;normpath 不解析符号链接,
            # '..' 已在 parts 层拒绝,安全性等价
            base_n = os.path.normpath(base)
            candidate = os.path.normpath(os.path.join(base_n, rel))
            if not candidate.startswith(base_n + os.sep):
                return None
            return candidate

        def do_GET(self):
            path = self.translate_path(self.path.split('?')[0])
            if path is None or not os.path.isfile(path):
                self.send_error(404, 'mesh not found', self.path)
                return
            ctype = mimetypes.guess_type(path)[0] or 'application/octet-stream'
            try:
                with open(path, 'rb') as f:
                    data = f.read()
            except OSError:
                self.send_error(500, 'read error', self.path)
                return
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')  # 前端跨域加载 mesh
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, fmt, *args):
            pass  # 静音访问日志,异常仍会打印

    return MeshHandler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8090)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), make_handler())
    print(f'[urdf_mesh_server] listening on {args.host}:{args.port} '
          f'(package://<pkg>/<rel> -> share dir)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
