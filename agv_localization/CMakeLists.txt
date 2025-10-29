cmake_minimum_required(VERSION 3.8)
project(agv_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d highgui aruco)
find_package(yaml-cpp REQUIRED)

# Python Executables
ament_python_install_package(agv_localization)

# C++ Executables
add_executable(imu_republisher src/imu_republisher.cpp)
ament_target_dependencies(imu_republisher rclcpp sensor_msgs)

add_executable(odom_republisher src/odom_republisher.cpp)
ament_target_dependencies(odom_republisher rclcpp nav_msgs)

add_executable(aruco_detector_sim src/aruco_detector_sim.cpp)
ament_target_dependencies(aruco_detector_sim
  rclcpp sensor_msgs geometry_msgs std_msgs visualization_msgs
  cv_bridge tf2 tf2_ros tf2_geometry_msgs ament_index_cpp
)
target_link_libraries(aruco_detector_sim ${OpenCV_LIBRARIES} yaml-cpp)

# Includes
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Install resources
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS imu_republisher odom_republisher aruco_detector_sim
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()