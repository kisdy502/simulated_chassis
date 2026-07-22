#!/bin/bash
# 地图导出工具 - 将OccupancyGrid转为PNG图片

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

MAP_NAME=${1:-"slam_map"}
OUTPUT_DIR="/ros2_ws/maps"

echo "=========================================="
echo "地图导出工具"
echo "=========================================="

# 安装依赖（如果没装）
pip install numpy Pillow opencv-python-headless -q 2>/dev/null || true

python3 << 'EOF'
import os
import sys
import rclpy
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import numpy as np

def map_callback(msg):
    """将ROS地图转换为PNG"""
    
    # 获取地图数据
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    
    # 转换数据 (-1=未知, 0=空闲, 100=障碍)
    data = np.array(msg.data, dtype=np.int8).reshape((height, width))
    
    # 转换为RGB图像
    img = np.ones((height, width, 3), dtype=np.uint8) * 255  # 白色背景(未知)
    
    # 空闲区域 (0) -> 白色 (255,255,255)
    img[data == 0] = [255, 255, 255]
    
    # 障碍物 (100) -> 黑色 (0,0,0)
    img[data == 100] = [0, 0, 0]
    
    # 已探索但未知 (-1) -> 浅灰色 (220,220,220)
    img[data == -1] = [220, 220, 220]
    
    # 保存PNG
    output_path = f"/ros2_ws/maps/{sys.argv[1]}_map.png"
    Image.fromarray(img).save(output_path)
    
    print(f"\n✅ 地图已保存到: {output_path}")
    print(f"\n【地图信息】")
    print(f"  尺寸: {width} x {height} 像素")
    print(f"  分辨率: {resolution} m/pixel ({resolution*100} cm/pixel)")
    print(f"  实际大小: ~{width*resolution:.1f}m x {height*resolution:.1f}m")
    
    # 统计信息
    total = width * height
    unknown = np.sum(data == -1)
    free = np.sum(data == 0)
    occupied = np.sum(data == 100)
    
    print(f"\n【像素统计】")
    print(f"  总像素: {total}")
    print(f"  空闲区域: {free} ({free*100/total:.1f}%)")
    print(f"  障碍物: {occupied} ({occupied*100/total:.1f}%)")
    print(f"  未探索: {unknown} ({unknown*100/total:.1f}%)")
    
    print(f"\n【建图质量评估】")
    coverage = (total - unknown) / total * 100
    if coverage > 80:
        quality = "优秀 ⭐⭐⭐"
    elif coverage > 60:
        quality = "良好 ⭐⭐"
    elif coverage > 40:
        quality = "一般 ⭐"
    else:
        quality = "需要继续探索"
    
    print(f"  探索覆盖率: {coverage:.1f}%")
    print(f"  质量评级: {quality}")
    
    rclpy.shutdown()

def main():
    rclpy.init()
    node = rclpy.create_node('map_exporter')
    
    sub = node.create_subscription(
        OccupancyGrid,
        '/map',
        map_callback,
        10
    )
    
    print("等待地图数据...")
    try:
        rclpy.spin_once(node, timeout_sec=5.0)
        if not rclpy.ok():
            print("❌ 未收到地图数据")
            return
    except Exception as e:
        print(f"错误: {e}")

if __name__ == "__main__":
    main()
EOF "$MAP_NAME"
