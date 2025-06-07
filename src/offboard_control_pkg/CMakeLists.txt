cmake_minimum_required(VERSION 3.8)
project(offboard_control_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Drone1Info.msg"
  "msg/Drone2Info.msg"
  "msg/Drone3Info.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_python_install_package(offboard_control_py)

ament_package()
