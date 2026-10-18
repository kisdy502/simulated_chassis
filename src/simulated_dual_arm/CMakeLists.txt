cmake_minimum_required(VERSION 3.8)
project(simulated_dual_arm)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dual_arm_sim_node.py
  scripts/urdf_mesh_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
