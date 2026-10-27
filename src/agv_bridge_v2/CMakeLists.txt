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
find_package(ament_index_cpp REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

# 标准接口包：/agv/follow_edge、/agv/set_control、/agv/status
find_package(agv_bridge_v2_interfaces REQUIRED)

set(AGV_ROS_DEPS
  rclcpp
  ament_index_cpp
  rclcpp_action
  geometry_msgs
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  nav2_msgs
  lifecycle_msgs
  cartographer_ros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  agv_bridge_v2_interfaces
)

# ==================== 可执行文件 ====================
# 全部业务逻辑只有 6 个源文件：
#   AgvNavServerNode  —— 瘦节点：action/srv/status/TF 接口
#   NavigationManager —— 直线/贝塞尔/倒车路径生成 + nav2 FollowPath/Spin 调度
#   LocalizationMonitor —— TF 定位收敛监控
#   MapFileManager    —— 地图目录扫描 + pgm/yaml -> OccupancyGrid（/agv/get_map 等）
#   TransformUtils    —— 四元数/偏航角/贝塞尔数学
add_executable(agv_nav_server
  src/agv_nav_server_main.cpp
  src/AgvNavServerNode.cpp
  src/NavigationManager.cpp
  src/LocalizationMonitor.cpp
  src/MapFileManager.cpp
  src/TransformUtils.cpp
)

target_include_directories(agv_nav_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(agv_nav_server ${AGV_ROS_DEPS})

# ==================== 安装 ====================
add_executable(urdf_mesh_server src/urdf_mesh_server.cpp)
ament_target_dependencies(urdf_mesh_server ament_index_cpp)
target_link_libraries(urdf_mesh_server Boost::system Threads::Threads)

install(TARGETS agv_nav_server urdf_mesh_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
)

# ==================== 导出 ====================
ament_export_dependencies(${AGV_ROS_DEPS})

if(BUILD_TESTING)
  find_program(BASH_EXECUTABLE bash REQUIRED)
  find_program(CURL_EXECUTABLE curl REQUIRED)
  add_test(NAME test_urdf_mesh_server
    COMMAND ${BASH_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_urdf_mesh_server.sh
      $<TARGET_FILE:urdf_mesh_server>)
  set_tests_properties(test_urdf_mesh_server PROPERTIES TIMEOUT 45)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation_readiness
    test/test_navigation_readiness.cpp
    src/NavigationManager.cpp src/LocalizationMonitor.cpp src/TransformUtils.cpp
    ENV ROS_DOMAIN_ID=87 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    TIMEOUT 90)
  target_include_directories(test_navigation_readiness PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_navigation_readiness ${AGV_ROS_DEPS})
  ament_add_gtest(test_battery_gateway test/test_battery_gateway.cpp
    ENV ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp TIMEOUT 45)
  target_include_directories(test_battery_gateway PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_battery_gateway ${AGV_ROS_DEPS})
endif()

ament_package()
