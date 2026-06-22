cmake_minimum_required(VERSION 3.8)
project(simulated_chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_util
  pluginlib
)

add_library(simulated_chassis_core SHARED
  src/three_wheel_steering_controller.cpp
)
target_include_directories(simulated_chassis_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(simulated_chassis_core ${dependencies})

# ros2_control 使用现代 CMake target，不能通过 ament_target_dependencies 链接
target_link_libraries(simulated_chassis_core
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  Eigen3::Eigen
)


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

# Gamepad Teleop Node
add_executable(gamepad_teleop_node
  src/gamepad_teleop_node.cpp
)

ament_target_dependencies(gamepad_teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
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


install(TARGETS odom_relay_node gamepad_teleop_node cmd_vel_relay_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS simulated_chassis_core 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install directories
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


pluginlib_export_plugin_description_file(controller_interface plugin.xml)


# Add your nodes, libraries and plugins here.
# Example:
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
