cmake_minimum_required(VERSION 3.8)
project(agv_mapping_with_knowns_poses)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Thêm các gói phụ thuộc cho ArUco
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d highgui aruco)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)


# C++ Executables
add_executable(mapping_with_knowns_poses src/mapping_with_knowns_poses.cpp)
ament_target_dependencies(mapping_with_knowns_poses
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(mapping_aruco src/mapping_aruco.cpp)
ament_target_dependencies(mapping_aruco
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  ament_index_cpp
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(mapping_aruco ${OpenCV_LIBRARIES} yaml-cpp)

# Thêm thư mục include vào đường dẫn của trình biên dịch
include_directories(include)

<<<<<<< HEAD
add_executable(agv_mapping_with_knowns_poses src/mapping_with_knowns_poses.cpp)
ament_target_dependencies(agv_mapping_with_knowns_poses rclcpp sensor_msgs tf2_ros nav_msgs geometry_msgs tf2)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  agv_mapping_with_knowns_poses
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch maps config
=======
# Install resources
install(DIRECTORY launch maps
>>>>>>> 9d3407644596706568c23a5bf8361ad8ee59e2de
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS mapping_with_knowns_poses mapping_aruco
  DESTINATION lib/${PROJECT_NAME})

ament_package()