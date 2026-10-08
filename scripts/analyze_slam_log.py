#!/usr/bin/env python3
"""分析 /tmp/agv_slam.log 里 cartographer 约束日志：闭环对象分布、平移差分布。"""
import re
from collections import Counter

pat = re.compile(
    r'Node \(0, (\d+)\) with (\d+) points on submap \(0, (\d+)\) '
    r'differs by translation ([0-9.]+) rotation ([0-9.]+) with score ([0-9.]+)%')

rows = []
for line in open('/tmp/agv_slam.log', errors='ignore'):
    m = pat.search(line)
    if m:
        rows.append(tuple(float(g) for g in m.groups()))

print('约束总数:', len(rows))
if rows:
    submaps = Counter(int(r[2]) for r in rows)
    print('闭环对象 submap 分布 (top10):', submaps.most_common(10))
    nodes = sorted(int(r[0]) for r in rows)
    print('节点号范围: %d ~ %d' % (nodes[0], nodes[-1]))
    ts = sorted(r[3] for r in rows)
    print('平移差: min=%.2f 中位=%.2f max=%.2f' % (ts[0], ts[len(ts)//2], ts[-1]))
    ss = sorted(r[5] for r in rows)
    print('匹配得分: min=%.1f%% 中位=%.1f%% max=%.1f%%' % (ss[0], ss[len(ss)//2], ss[-1]))
    # 平移差 > 1m 的约束 = 大修正（跳变来源）
    big = [r for r in rows if r[3] > 1.0]
    print('平移差>1m 的约束数:', len(big))
    for r in big[:10]:
        print('  node=%d submap=%d 平移差=%.2f score=%.1f%%' % (r[0], r[2], r[3], r[5]))

comp = [l.strip() for l in open('/tmp/agv_slam.log', errors='ignore') if 'computations resulted' in l]
nonzero = [l for l in comp if 'resulted in 0 ' not in l]
print()
print('约束计算批次: 总 %d, 非零 %d' % (len(comp), len(nonzero)))
for l in nonzero[:5]:
    print(' ', l.split(']')[-1].strip())
