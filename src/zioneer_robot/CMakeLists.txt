cmake_minimum_required(VERSION 3.8)
project(zioneer_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Odom Relay Node
add_executable(odom_relay_node
  src/odom_relay_node.cpp
)
ament_target_dependencies(odom_relay_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

# Cmd Vel Relay Node
add_executable(cmd_vel_relay_node
  src/cmd_vel_relay_node.cpp
)
ament_target_dependencies(cmd_vel_relay_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Twist → TwistStamped 转发节点（兼容较新版本 diff_drive_controller）
add_executable(twist_to_stamped_relay
  src/twist_to_stamped_relay.cpp
)
ament_target_dependencies(twist_to_stamped_relay
  rclcpp
  geometry_msgs
)

# Gamepad Teleop Node
add_executable(gamepad_teleop_node
  src/gamepad_teleop_node.cpp
)
ament_target_dependencies(gamepad_teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# PointCloud2 本体过滤节点（剔除打在机器人本体上的回波）
add_executable(pointcloud_footprint_filter_node
  src/pointcloud_footprint_filter_node.cpp
)
ament_target_dependencies(pointcloud_footprint_filter_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS odom_relay_node cmd_vel_relay_node twist_to_stamped_relay gamepad_teleop_node pointcloud_footprint_filter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY
  param/
  DESTINATION share/${PROJECT_NAME}/param
)
install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
)
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(DIRECTORY
  world/
  DESTINATION share/${PROJECT_NAME}/world
)
install(DIRECTORY
  meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
