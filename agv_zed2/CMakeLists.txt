cmake_minimum_required(VERSION 3.8)
project(agv_zed2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(mapping src/mapping.cpp)
ament_target_dependencies(mapping
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  tf2
  tf2_ros
  yaml-cpp
  ament_index_cpp
)

target_link_libraries(mapping
  ${OpenCV_LIBS}
  yaml-cpp
)

install(TARGETS
  mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/aruco_detector.py
  scripts/aruco_mapper.py
  scripts/aruco_localizer.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
config 
map
  DESTINATION share/${PROJECT_NAME}
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

# Export dependencies for other packages that might depend on agv_zed2
# This ensures that if another package links against agv_zed2, it will also find these dependencies.
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  tf2
  tf2_ros
  yaml-cpp
  OpenCV
  ament_index_cpp
)

ament_package()
