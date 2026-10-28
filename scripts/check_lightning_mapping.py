#!/usr/bin/env python3
"""Compile regressions/syntax in /tmp; never rebuild or overwrite a live SLAM library."""
import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--syntax', action='store_true', help='Also syntax-check changed SLAM/render translation units')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    vendor = workspace / 'src/lightning-lm'
    flags_file = workspace / 'build/lightning/src/CMakeFiles/lightning.libs.dir/flags.make'
    if not flags_file.is_file():
        parser.error('Requires an existing native Lightning build for its platform-specific include paths')
    flags = {}
    for line in flags_file.read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1)
            flags[key] = shlex.split(value)
    includes = flags['CXX_INCLUDES']
    with tempfile.TemporaryDirectory(prefix='lightning-frame-check-') as temporary:
        executable = str(Path(temporary) / 'test_mapping_frames')
        subprocess.run(['g++', '-std=c++17', '-O0', '-pthread', *includes,
                        str(vendor / 'test/test_mapping_frames.cc'), '-lglog', '-o', executable], check=True)
        subprocess.run([executable], check=True)
        graph_test = str(Path(temporary) / 'test_loop_validation')
        miao = workspace / 'install/lightning/lib/libmiao.core.so'
        if not miao.is_file():
            parser.error('Missing installed MIAO library for the synthetic pose-graph regression')
        subprocess.run(['g++', '-std=c++17', '-O0', '-pthread', *includes,
                        str(vendor / 'test/test_loop_validation.cc'), str(miao),
                        '-Wl,-rpath,' + str(miao.parent), '-lglog', '-ltbb', '-o', graph_test], check=True)
        subprocess.run([graph_test], check=True)
        if args.syntax:
            for name in ('core/system/slam.cc', 'core/g2p5/g2p5.cc', 'core/loop_closing/loop_closing.cc'):
                print(f'Syntax check: {name}', flush=True)
                subprocess.run(['g++', '-std=c++17', '-O0', '-fPIC', '-fopenmp', '-fsyntax-only',
                                *flags['CXX_DEFINES'], *includes, str(vendor / 'src' / name)], check=True)
    print('PASS: live build/install outputs were not modified')


if __name__ == '__main__':
    main()
