"""Integration check against Cartographer serialization and the native exporter."""
from pathlib import Path
import struct
import subprocess
import sys
import tempfile


def main():
    fixture, exporter = sys.argv[1:]
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / 'fixture.pbstream'
        target = Path(directory) / 'fixture.ply'
        subprocess.run([fixture, str(source)], check=True)
        subprocess.run([exporter, str(source), str(target)], check=True)
        header, payload = target.read_bytes().split(b'end_header\n', 1)
        assert b'element vertex 1\n' in header, header
        points = list(struct.iter_unpack('<fff', payload))
        assert len(points) == 1, points
        assert all(abs(a - b) < 1e-5 for a, b in zip(points[0], (10, 21, 5))), points
        invalid = subprocess.run([exporter, str(source), str(target), '0'], capture_output=True)
        assert invalid.returncode != 0
        print('Optimized pose, overlapping submap deduplication, free-space filtering and invalid options: PASS')


if __name__ == '__main__':
    main()
