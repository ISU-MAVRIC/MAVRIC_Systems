cmake_minimum_required(VERSION 3.8)
project(spark_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmData.msg"
  "msg/Drivetrain.msg"
  "msg/ScaleFeedback.msg" 
  "msg/Steer.msg"
  "msg/Steertrain.msg"
  DEPENDENCIES std_msgs
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables with correct path
install(PROGRAMS
  spark_can/SparkCAN_Drive_Train.py
  spark_can/Scale_Startups.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package
install(DIRECTORY
  spark_can/
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
