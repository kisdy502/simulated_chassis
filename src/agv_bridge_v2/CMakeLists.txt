cmake_minimum_required(VERSION 3.8)
project(agv_bridge_v2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==================== 依赖 ====================
# 注意：本包已不再需要 libwebsockets / libcurl / nlohmann_json。
#   对外通信由 rosbridge_server 承担，对外契约由 agv_bridge_v2_interfaces 的 IDL 定义。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

# 标准接口包：/agv/follow_edge、/agv/set_control、/agv/status
find_package(agv_bridge_v2_interfaces REQUIRED)

set(AGV_ROS_DEPS
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  sensor_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  agv_bridge_v2_interfaces
)

# ==================== 可执行文件 ====================
# 全部业务逻辑只有 5 个源文件：
#   AgvNavServerNode  —— 瘦节点：action/srv/status/TF 接口
#   NavigationManager —— 直线/贝塞尔/倒车路径生成 + nav2 FollowPath/Spin 调度
#   LocalizationMonitor —— TF 定位收敛监控
#   TransformUtils    —— 四元数/偏航角/贝塞尔数学
add_executable(agv_nav_server
  src/agv_nav_server_main.cpp
  src/AgvNavServerNode.cpp
  src/NavigationManager.cpp
  src/LocalizationMonitor.cpp
  src/TransformUtils.cpp
)

target_include_directories(agv_nav_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(agv_nav_server ${AGV_ROS_DEPS})

# ==================== 安装 ====================
install(TARGETS agv_nav_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch docs scripts
  DESTINATION share/${PROJECT_NAME}
)

# ==================== 导出 ====================
ament_export_dependencies(${AGV_ROS_DEPS})

ament_package()
