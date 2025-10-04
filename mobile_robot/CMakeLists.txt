cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(simple_controller src/simple_controller.cpp)

add_executable(twist_relay src/twist_relay.cpp)

ament_target_dependencies(twist_relay rclcpp geometry_msgs)

ament_target_dependencies(simple_controller rclcpp geometry_msgs sensor_msgs ${Eigen_LIBRARIES})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

install (
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install (
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install (
  DIRECTORY xacro
  DESTINATION share/${PROJECT_NAME}
)

install (
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install (
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

install (
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)
install (
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
install (
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS simple_controller twist_relay
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  src/scan_repub.py
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
