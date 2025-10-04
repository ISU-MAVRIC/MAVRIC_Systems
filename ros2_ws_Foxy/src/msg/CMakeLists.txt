cmake_minimum_required(VERSION 3.5)
project(msg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find all .msg files in this package
file(GLOB msg_files "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

# Convert to paths relative to the package root (CMake knows CMAKE_CURRENT_SOURCE_DIR)
set(rel_msg_files "")
foreach(file_path ${msg_files})
  file(RELATIVE_PATH rel_msg_path "${CMAKE_CURRENT_SOURCE_DIR}" "${file_path}")
  list(APPEND rel_msg_files "${rel_msg_path}")
endforeach()

# Generate all message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${rel_msg_files}
)

ament_package()






if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


