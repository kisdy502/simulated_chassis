"""Run the native PCL exporter on an ASCII XYZ PCD, including non-finite points."""
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

with tempfile.TemporaryDirectory() as directory:
    source=Path(directory)/'map.pcd'
    target=Path(directory)/'map.ply'
    source.write_text('''VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 4
HEIGHT 1
POINTS 4
DATA ascii
1 2 3
1.01 2.01 3.01
4 5 6
nan 0 0
''')
    subprocess.run([sys.argv[1],str(source),str(target),'0.1'],check=True)
    header,body=target.read_bytes().split(b'end_header\n',1)
    assert b'element vertex 2\n' in header,header
    points=list(struct.iter_unpack('<fff',body))
    assert len(points)==2,points
    assert any(abs(p[0]-1.005)<1e-4 and abs(p[2]-3.005)<1e-4 for p in points),points
    assert any(p==(4.,5.,6.) for p in points),points
    for value in ('0','nan','1e-300','10','0.1garbage'):
        result=subprocess.run([sys.argv[1],str(source),str(target),value],capture_output=True)
        assert result.returncode!=0,value
    source.write_text(source.read_text().replace('1 2 3\n', '1e20 2 3\n'))
    result=subprocess.run([sys.argv[1],str(source),str(target),'0.1'],capture_output=True)
    assert result.returncode!=0 and b'index range' in result.stderr,result.stderr
    print('PCD XYZ, NaN filtering, voxel centroid, Z retention and invalid options: PASS')
