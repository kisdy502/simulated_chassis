#!/bin/bash
# SLAM 建图状态检查与地图管理工具

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

case "$1" in
    status)
        echo "=========================================="
        echo "SLAM 建图状态"
        echo "=========================================="
        
        echo ""
        echo "【Cartographer节点】"
        ros2 node list | grep -E "carto" || echo "未运行"
        
        echo ""
        echo "【数据流】"
        for topic in /scan_1 /scan_2 /odom /imu; do
            hz=$(timeout 2 ros2 topic hz $topic 2>/dev/null | grep -oP "average rate: \K[0-9.]+" || echo "0 Hz")
            printf "  %-15s %s\n" "$topic:" "$hz"
        done
        
        echo ""
        echo "【地图状态】"
        map_info=$(timeout 2 ros2 topic echo /map --once 2>/dev/null)
        if [ -n "$map_info" ]; then
            width=$(echo "$map_info" | grep -A1 "width:" | tail -1 | tr -d ' ')
            height=$(echo "$map_info" | grep -A1 "height:" | tail -1 | tr -d ' ')
            res=$(echo "$map_info" | grep -A1 "resolution:" | tail -1 | tr -d ' ')
            echo "  尺寸: ${width} x ${height} pixels"
            echo "  分辨率: ${res} m/pixel"
            
            # 计算实际大小
            w_m=$(echo "$width * $res" | bc 2>/dev/null || echo "?")
            h_m=$(echo "$height * $res" | bc 2>/dev/null || echo "?")
            echo "  实际: ~${w_m}m x ${h_m}m"
        else
            echo "  ⚠️  地图为空或未发布"
        fi
        
        echo ""
        echo "【机器人位置(odom)】"
        timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -A6 "position:" | head -4
        
        ;;
        
    save)
        map_name=${2:-"slam_map_$(date +%Y%m%d_%H%M%S)"}
        pbstream_file="/ros2_ws/maps/${map_name}.pbstream"
        
        echo "保存Cartographer状态到: ${map_name}.pbstream"
        
        # 调用write_state服务
        result=$(ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
            "{filename: '${pbstream_file}'}" 2>&1)
        
        if echo "$result" | grep -q "success"; then
            echo "✅ 状态已保存: ${pbstream_file}"
            
            # 同时保存PGM/YAML格式（如果map_saver可用）
            if ros2 node list 2>/dev/null | grep -q "map_saver"; then
                yaml_file="/ros2_ws/maps/${map_name}"
                ros2 service call /map_saver_server/save_map nav2_msgs/srv/SaveMap \
                    "{map_url: '${yaml_file}'}" 2>/dev/null && \
                echo "✅ 地图文件: ${yaml_file}.yaml"
            fi
        else
            echo "❌ 保存失败"
            echo "$result"
        fi
        ;;
        
    explore)
        echo "让机器人自动探索环境..."
        echo "按 Ctrl+C 停止"
        
        # 圆形运动探索
        while true; do
            # 直走3秒
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                "{linear: {x: 0.3}, angular: {z: 0.0}}" --once 2>/dev/null
            sleep 3
            
            # 转弯1秒
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                "{linear: {x: 0.0}, angular: {z: 0.5}}" --once 2>/dev/null
            sleep 1
            
            # 直走3秒
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                "{linear: {x: 0.3}, angular: {z: 0.0}}" --once 2>/dev/null
            sleep 3
            
            # 反向转弯
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                "{linear: {x: 0.0}, angular: {z: -0.5}}" --once 2>/dev/null
            sleep 1
        done
        ;;
        
    *)
        echo "用法: $0 {status|save|explore} [参数]"
        echo ""
        echo "命令:"
        echo "  status          查看SLAM状态和数据流"
        echo "  save [name]     保存地图 (默认: slam_map_时间戳)"
        echo "  explore         让机器人自动移动探索"
        ;;
esac
