cmake_minimum_required(VERSION 3.8)
project(ufactory_ellmer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)   

# ── 2. tell rosidl to create code for the service ──────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetCoordinates.msg"
  "msg/TargetCoordinatesArray.msg"
  "srv/SetInt16.srv"
  DEPENDENCIES geometry_msgs
)

# ── 3. make sure the generated libraries are exported to downstream packages ──
ament_export_dependencies(rosidl_default_runtime)

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

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME})

ament_package()
