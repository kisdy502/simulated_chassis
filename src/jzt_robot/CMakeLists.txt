cmake_minimum_required(VERSION 3.8)
project(jzt_robot)

find_package(ament_cmake REQUIRED)

# 纯配置文件包，无 C++ 源码，仅安装目录

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
