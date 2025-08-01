set(_AMENT_PACKAGE_NAME "gazebo_ros")
set(gazebo_ros_VERSION "3.9.0")
set(gazebo_ros_MAINTAINER "Jose Luis Rivero <jrivero@osrfoundation.org>, Louise Poubel <louise@openrobotics.org>")
set(gazebo_ros_BUILD_DEPENDS "builtin_interfaces" "gazebo_dev" "gazebo_msgs" "rcl" "rclcpp" "rclpy" "rmw" "std_srvs" "tinyxml_vendor")
set(gazebo_ros_BUILDTOOL_DEPENDS "ament_cmake")
set(gazebo_ros_BUILD_EXPORT_DEPENDS "geometry_msgs" "sensor_msgs" "builtin_interfaces" "gazebo_dev" "gazebo_msgs" "rcl" "rclcpp" "rclpy" "rmw" "std_srvs" "tinyxml_vendor")
set(gazebo_ros_BUILDTOOL_EXPORT_DEPENDS )
set(gazebo_ros_EXEC_DEPENDS "launch_ros" "python3-lxml" "builtin_interfaces" "gazebo_dev" "gazebo_msgs" "rcl" "rclcpp" "rclpy" "rmw" "std_srvs" "tinyxml_vendor")
set(gazebo_ros_TEST_DEPENDS "ament_cmake_gtest" "ament_lint_auto" "ament_lint_common" "geometry_msgs" "launch_testing_ament_cmake" "ros2run" "sensor_msgs" "std_msgs")
set(gazebo_ros_GROUP_DEPENDS )
set(gazebo_ros_MEMBER_OF_GROUPS )
set(gazebo_ros_DEPRECATED "This package has been deprecated as of January 2025 with Gazebo classic 11 reaching
      end-of-life. Users are highly encouraged to migrate to the new Gazebo
      using our migration guides (https://gazebosim.org/docs/latest/gazebo_classic_migration)")
set(gazebo_ros_EXPORT_TAGS)
list(APPEND gazebo_ros_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND gazebo_ros_EXPORT_TAGS "<deprecated>This package has been deprecated as of January 2025 with Gazebo classic 11 reaching
      end-of-life. Users are highly encouraged to migrate to the new Gazebo
      using our migration guides (https://gazebosim.org/docs/latest/gazebo_classic_migration)</deprecated>")
