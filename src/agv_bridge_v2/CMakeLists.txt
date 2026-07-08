cmake_minimum_required(VERSION 3.8)
project(agv_bridge_v2)

# 默认使用 C++17
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
endif()

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED) # 添加这一行
find_package(rclcpp_action REQUIRED) # 添加rclcpp_action
find_package(tf2 REQUIRED) # 添加tf2
find_package(diagnostic_msgs REQUIRED) # 添加这一行
find_package(angles REQUIRED)

# 查找第三方库
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(CURL REQUIRED)
# 查找 JSON 库
find_package(nlohmann_json REQUIRED)

# 使用 pkg-config 查找 libwebsockets
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBWEBSOCKETS REQUIRED libwebsockets)

# 包含目录
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${nlohmann_json_INCLUDE_DIRS}
)
# # 源文件列表
# set(SOURCE_FILES
#     src/bean/*.cpp
#     # 主文件
#     src/AgvBridgeNode.cpp
#     # 原有WebSocket文件
#     src/websocket_server.cpp
#     # 新增模块化文件
#     src/HttpApiClient.cpp

#     #src/DataCollector.cpp
#     #src/StateManager.cpp
#     src/LocalizationMonitor.cpp
#     src/NavigationManager.cpp

#     # 主程序入口
#     src/main.cpp
# )

file(GLOB SOURCE_FILES
        "src/*.cpp"
        "src/bean/*.cpp"
)

# 创建可执行文件
add_executable(agv_bridge_node ${SOURCE_FILES})

# 使用ament_target_dependencies处理所有ROS2依赖
ament_target_dependencies(agv_bridge_node
        rclcpp
        geometry_msgs
        nav_msgs
        sensor_msgs
        tf2_ros
        std_msgs
        tf2_geometry_msgs
        nav2_msgs
        diagnostic_msgs
        tf2
        angles
)

# 链接第三方库
target_link_libraries(agv_bridge_node
        ${LIBWEBSOCKETS_LIBRARIES}
        OpenSSL::SSL
        OpenSSL::Crypto
        Threads::Threads
        CURL::libcurl # 添加这一行
        nlohmann_json::nlohmann_json
)

# 包含头文件
target_include_directories(agv_bridge_node
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${LIBWEBSOCKETS_INCLUDE_DIRS}
        ${nlohmann_json_INCLUDE_DIRS}
)

# 安装
install(TARGETS agv_bridge_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

# 导出
ament_export_dependencies(rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros std_msgs tf2_geometry_msgs nav2_msgs rclcpp_action tf2 diagnostic_msgs angles)
ament_export_libraries(agv_bridge_node)

ament_package()
