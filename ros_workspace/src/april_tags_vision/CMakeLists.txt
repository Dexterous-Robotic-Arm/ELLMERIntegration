cmake_minimum_required(VERSION 3.8)
project(april_tags_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/april_tags_detector_node.py
  scripts/test_april_tags.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python package
install(DIRECTORY
  ${PROJECT_NAME}
  DESTINATION lib/python3.10/site-packages/
)

# Install message files
install(FILES
  msg/AprilTagDetection.msg
  msg/AprilTagDetectionArray.msg
  DESTINATION share/${PROJECT_NAME}/msg
)

# Generate message files
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Test dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
