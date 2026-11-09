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
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
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
  rcl_interfaces
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
# 业务入口、导航调度与独立路径生成模块：
#   AgvNavServerNode  —— 瘦节点：action/srv/status/TF 接口
#   NavigationManager —— 任务阶段及 Nav2 动作调度；navigation/*PathBuilder 只生成几何路径
#   LocalizationMonitor —— TF 定位收敛监控
#   MapFileManager    —— 地图目录扫描 + pgm/yaml -> OccupancyGrid（/agv/get_map 等）
#   TransformUtils    —— 四元数/偏航角/贝塞尔数学
set(AGV_PATH_BUILDER_SOURCES
  src/navigation/StraightPathBuilder.cpp
  src/navigation/BezierPathBuilder.cpp
  src/navigation/BackUpPathBuilder.cpp
)
set(AGV_NAV2_SESSION_SOURCES src/navigation/Nav2ActionSession.cpp)
set(AGV_NAVIGATION_EXECUTOR_SOURCES
  src/navigation/FreeNavigationExecutor.cpp
  src/navigation/PathNavigationExecutor.cpp
)
add_executable(agv_nav_server
  ${AGV_NAV2_SESSION_SOURCES}
  ${AGV_NAVIGATION_EXECUTOR_SOURCES}
  ${AGV_PATH_BUILDER_SOURCES}
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
install(TARGETS agv_nav_server DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
)

# ==================== 导出 ====================
ament_export_dependencies(${AGV_ROS_DEPS})

if(BUILD_TESTING)
  find_program(BASH_EXECUTABLE bash REQUIRED)
  find_program(CURL_EXECUTABLE curl REQUIRED)
  find_program(PYTHON3_EXECUTABLE python3 REQUIRED)
  add_test(NAME test_navigation_admission
    COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_admission.py
      $<TARGET_FILE:agv_nav_server>)
  set_tests_properties(test_navigation_admission PROPERTIES TIMEOUT 60
    ENVIRONMENT "ROS_DOMAIN_ID=93;RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation_executors test/test_navigation_executors.cpp
    src/TransformUtils.cpp ${AGV_PATH_BUILDER_SOURCES} ${AGV_NAVIGATION_EXECUTOR_SOURCES})
  target_include_directories(test_navigation_executors PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_navigation_executors ${AGV_ROS_DEPS})
  ament_add_gtest(test_path_builders test/test_path_builders.cpp
    src/TransformUtils.cpp ${AGV_PATH_BUILDER_SOURCES})
  target_include_directories(test_path_builders PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_path_builders ${AGV_ROS_DEPS})
  ament_add_gtest(test_localization_operation test/test_localization_operation.cpp
    ENV ROS_DOMAIN_ID=94 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp TIMEOUT 45)
  target_include_directories(test_localization_operation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_localization_operation ${AGV_ROS_DEPS})
  ament_add_gtest(test_navigation_readiness
    test/test_navigation_readiness.cpp
    src/NavigationManager.cpp src/LocalizationMonitor.cpp src/TransformUtils.cpp
    ${AGV_NAV2_SESSION_SOURCES}
    ${AGV_NAVIGATION_EXECUTOR_SOURCES}
    ${AGV_PATH_BUILDER_SOURCES}
    ENV ROS_DOMAIN_ID=87 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    TIMEOUT 90)
  target_include_directories(test_navigation_readiness PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_navigation_readiness ${AGV_ROS_DEPS})
  add_test(NAME test_action_session_lock_order
    COMMAND $<TARGET_FILE:test_navigation_readiness>
      --gtest_filter=NavigationReadinessTest.TransportFeedbackDoesNotWaitForManagerLock:NavigationReadinessTest.FrozenSpinFeedbackCancelsButWaitsForTerminalResult
      --gtest_repeat=10)
  set_tests_properties(test_action_session_lock_order PROPERTIES TIMEOUT 90
    ENVIRONMENT "ROS_DOMAIN_ID=95;RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
  ament_add_gtest(test_battery_gateway test/test_battery_gateway.cpp
    ENV ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp TIMEOUT 45)
  target_include_directories(test_battery_gateway PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_battery_gateway ${AGV_ROS_DEPS})
endif()

ament_package()
