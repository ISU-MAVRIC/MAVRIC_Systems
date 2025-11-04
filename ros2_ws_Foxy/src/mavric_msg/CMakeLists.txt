cmake_minimum_required(VERSION 3.5)
project(mavric_msg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find all .msg files in this package
file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")

# Find all .srv files in this package
file(GLOB srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")

# Generate all message and service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()


