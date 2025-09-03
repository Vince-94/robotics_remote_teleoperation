cmake_minimum_required(VERSION 3.8)
project(robot_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)


######################
# Python executables #
######################

# install python targets
install(PROGRAMS
  ${PROJECT_NAME}/status_publisher.py
  ${PROJECT_NAME}/cmd_vel_subscriber.py
  ${PROJECT_NAME}/camera_publisher.py
  ${PROJECT_NAME}/watchdog.py
  ${PROJECT_NAME}/fake_cmd_vel_publisher.py
  ${PROJECT_NAME}/fake_emergency_stop_publisher.py
  ${PROJECT_NAME}/fake_camera_publisher.py
  ${PROJECT_NAME}/monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)


###########
# Install #
###########

# Installing directories
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)


###########
# Testing #
###########

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
