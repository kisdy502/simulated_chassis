#!/usr/bin/env python3
import re
import matplotlib.pyplot as plt
import numpy as np

# 读取数据
with open('odom_pose.txt', 'r') as f:
    content = f.read()

# 按 '---' 分割消息
messages = content.split('---')

xs, ys, yaws = [], [], []

for msg in messages:
    # 提取 x, y, z, w
    x_match = re.search(r'x:\s*([-+]?\d*\.\d+|\d+)', msg)
    y_match = re.search(r'y:\s*([-+]?\d*\.\d+|\d+)', msg)
    z_match = re.search(r'z:\s*([-+]?\d*\.\d+|\d+)', msg)
    w_match = re.search(r'w:\s*([-+]?\d*\.\d+|\d+)', msg)
    
    if x_match and y_match and z_match and w_match:
        x = float(x_match.group(1))
        y = float(y_match.group(1))
        z = float(z_match.group(1))
        w = float(w_match.group(1))
        yaw = 2 * np.arctan2(z, w)
        
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)

print(f'Recorded {len(xs)} poses')

# 绘制轨迹
plt.figure(figsize=(10, 10))
plt.plot(xs, ys, 'b-', linewidth=2)
plt.plot(xs[0], ys[0], 'go', markersize=10, label='Start')
plt.plot(xs[-1], ys[-1], 'ro', markersize=10, label='End')
plt.axis('equal')
plt.grid(True)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Odom Trajectory')
plt.legend()
plt.savefig('trajectory.png', dpi=150)
plt.show()

# 打印统计信息
print(f'Total distance: {np.sum(np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)):.3f} m')
print(f'Net displacement: ({xs[-1]-xs[0]:.3f}, {ys[-1]-ys[0]:.3f}) m')
print(f'Total rotation: {np.sum(np.abs(np.diff(yaws))):.3f} rad ({np.sum(np.abs(np.diff(yaws)))*180/np.pi:.1f}°)')