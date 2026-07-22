#!/bin/bash

set -e

COMPOSE_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$COMPOSE_DIR"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

usage() {
    echo "Usage: $0 {build|up|up-nav|up-slam|down|restart|logs|status|save-map|explore|clean}"
    echo ""
    echo "Commands:"
    echo "  build       - Build all Docker images"
    echo "  up          - Start core services (robot + bridge)"
    echo "  up-nav      - Start with Nav2 navigation"
    echo "  up-slam     - Start with Cartographer SLAM"
    echo "  down        - Stop and remove containers"
    echo "  restart     - Restart services"
    echo "  logs        - View logs [service_name]"
    echo "  status      - Check container status"
    echo "  save-map    - Save SLAM map (arg: map_name)"
    echo "  explore     - Auto-explore for mapping (args: speed turn_speed)"
    echo "  clean       - Remove images, volumes and cache"
    echo ""
    echo "Examples:"
    echo "  $0 build              # Build images"
    echo "  $0 up                 # Robot + Bridge"
    echo "  $0 up-slam            # Robot + Bridge + SLAM"
    echo "  $0 explore 0.3 0.5    # Start exploration"
    echo "  $0 save-map my_map    # Save map as my_map"
    echo "  $0 logs -f agv-bridge # Follow bridge logs"
}

build_images() {
    echo -e "${GREEN}Building ROS2 base image (for nav2/slam)...${NC}"
    docker compose build nav2-server cartographer || true
    
    echo -e "${GREEN}Building simulated robot image...${NC}"
    docker compose build robot-simulation
    
    echo -e "${GREEN}Building AGV bridge image...${NC}"
    docker compose build agv-bridge
    
    echo -e "${GREEN}All images built successfully!${NC}"
}

start_services() {
    local mode=$1

    case "$mode" in
        nav)
            echo -e "${GREEN}Starting: Robot + Bridge + Navigation${NC}"
            docker compose --profile navigation up -d --force-recreate
            ;;
        slam)
            echo -e "${GREEN}Starting: Robot + Bridge + SLAM${NC}"
            docker compose --profile slam up -d --force-recreate
            ;;
        *)
            echo -e "${GREEN}Starting: Robot + Bridge${NC}"
            docker compose up -d robot-simulation agv-bridge
            ;;
    esac

    echo -e "${GREEN}Services started. Checking status...${NC}"
    sleep 3
    docker compose ps
}

stop_services() {
    echo -e "${YELLOW}Stopping all services...${NC}"
    docker compose down --remove-orphans
    echo -e "${GREEN}Services stopped.${NC}"
}

restart_services() {
    stop_services
    sleep 2
    start_services ""
}

view_logs() {
    local service=$1
    if [ -n "$service" ]; then
        docker compose logs -f "$service"
    else
        docker compose logs -f
    fi
}

check_status() {
    echo -e "${GREEN}=== Container Status ===${NC}"
    docker compose ps
    
    echo ""
    echo -e "${GREEN}=== Resource Usage ===${NC}"
    docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}"
}

cleanup() {
    echo -e "${RED}This will remove all containers, images and volumes!${NC}"
    read -p "Are you sure? (y/N) " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker compose down -v --rmi all --remove-orphans
        docker system prune -f
        echo -e "${GREEN}Cleanup completed.${NC}"
    fi
}

save_map() {
    local map_name=${1:-"slam_map"}
    echo -e "${GREEN}Saving Cartographer state as: ${map_name}${NC}"

    docker exec cartographer_slam bash -c "
        source /opt/ros/jazzy/setup.bash
        source /ros2_ws/install/setup.bash
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \"{filename: '/ros2_ws/maps/${map_name}.pbstream'}\"
    "

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Map saved to ./maps/${map_name}.pbstream${NC}"
        ls -la ./maps/
        echo ""
        echo -e "${YELLOW}提示: 使用离线建图将pbstream转换为地图:${NC}"
        echo "  ./deploy.sh up-slam-offline bag:=<bag_file> pbstream:=./maps/${map_name}.pbstream"
    else
        echo -e "${RED}Failed to save map${NC}"
    fi
}

explore() {
    echo -e "${GREEN}Starting auto-exploration for SLAM...${NC}"
    echo "Press Ctrl+C to stop"
    
    docker exec -it robot_simulation bash -c '
        source /opt/ros/jazzy/setup.bash
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        
        LINEAR_SPEED=${1:-0.3}
        TURN_SPEED=${2:-0.5}
        
        echo "Exploration started (speed: $LINEAR_SPEED m/s, turn: $TURN_SPEED rad/s)"
        echo "Press Ctrl+C to stop"
        echo ""
        
        mode=0
        trap "echo Stopping; ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.0}}\" --once; exit" INT TERM
        
        while true; do
            case $mode in
                0)  # 直走
                    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                        "{linear: {x: $LINEAR_SPEED}, angular: {z: 0.0}}" --once
                    sleep 4
                    mode=1
                    ;;
                1)  # 左转
                    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                        "{linear: {x: 0.0}, angular: {z: $TURN_SPEED}}" --once
                    sleep 1.5
                    mode=2
                    ;;
                2)  # 直走
                    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                        "{linear: {x: $LINEAR_SPEED}, angular: {z: 0.0}}" --once
                    sleep 4
                    mode=3
                    ;;
                3)  # 右转
                    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                        "{linear: {x: 0.0}, angular: {z: -$TURN_SPEED}}" --once
                    sleep 1.5
                    mode=0
                    ;;
            esac
        done
    ' "$@"
}

case "$1" in
    build)
        build_images
        ;;
    up)
        start_services ""
        ;;
    up-nav)
        start_services "nav"
        ;;
    up-slam)
        start_services "slam"
        ;;
    down)
        stop_services
        ;;
    restart)
        restart_services
        ;;
    logs)
        view_logs "$2"
        ;;
    status)
        check_status
        ;;
    save-map)
        save_map "$2"
        ;;
    explore)
        explore "$2" "$3"
        ;;
    check-map)
        check_map_quality
        ;;
    clean)
        cleanup
        ;;
    *)
        usage
        exit 1
        ;;
esac
