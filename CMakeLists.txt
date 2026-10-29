cmake_minimum_required(VERSION 3.8)
project(lite6_labauto)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MovePose.msg"
  "msg/LabwareOBB.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

install(PROGRAMS
  scripts/robot_init.py
  scripts/robot_disable.py
  scripts/robot_move.py
  scripts/gripper_serial.py
  scripts/Droplet_CamToBase.py
  scripts/Marker_CamToBase.py
  scripts/DropletAruco_detection.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config sketchbook DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
