cmake_minimum_required(VERSION 3.8)
project(simulated_arm)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
