cmake_minimum_required(VERSION 3.8)
project(arm_manager_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmCommand.action" "msg/ArmStatus.msg" "msg/TeachPoint.msg"
  "srv/SaveTeachPoint.srv" "srv/ListTeachPoints.srv" "srv/DeleteTeachPoint.srv"
  "srv/InjectFault.srv" DEPENDENCIES geometry_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
