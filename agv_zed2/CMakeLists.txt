cmake_minimum_required(VERSION 3.8)
project(agv_zed2)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ======================================================
# Node 1: aruco_detector
# ======================================================
add_executable(aruco_detector src/aruco_detector.cpp)

ament_target_dependencies(aruco_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
)

target_link_libraries(aruco_detector
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

# ======================================================
# Node 2: aruco_mapper
# ======================================================
add_executable(aruco_mapper src/aruco_mapper.cpp)

ament_target_dependencies(aruco_mapper
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml-cpp
  ament_index_cpp
)

target_link_libraries(aruco_mapper
  yaml-cpp
)

# ======================================================
# Node 3: aruco_localizer
# ======================================================
add_executable(aruco_localizer src/aruco_localizer.cpp)

ament_target_dependencies(aruco_localizer
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml-cpp
  ament_index_cpp
)

target_link_libraries(aruco_localizer
  yaml-cpp
)

# ======================================================
# Node 4: camera_zed2
# ======================================================
add_executable(camera_zed2 src/camera_zed2.cpp)

ament_target_dependencies(camera_zed2
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(camera_zed2
  ${OpenCV_LIBRARIES}
)
# =====================
# Install executables
# =====================
install(TARGETS
  aruco_detector
  aruco_mapper
  aruco_localizer
  camera_zed2
  DESTINATION lib/${PROJECT_NAME}
)

# =====================
# Install config files
# =====================
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

# =====================
# Tests (optional)
# =====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =====================
# Export dependencies
# =====================
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml-cpp
  OpenCV
  ament_index_cpp
)

ament_package()
