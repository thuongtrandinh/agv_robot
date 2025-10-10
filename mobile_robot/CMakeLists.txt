cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d highgui aruco)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Executables
add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs sensor_msgs)

add_executable(twist_relay src/twist_relay.cpp)
ament_target_dependencies(twist_relay rclcpp geometry_msgs)

add_executable(aruco_detector_sim src/aruco_detector_sim.cpp)
ament_target_dependencies(aruco_detector_sim
  rclcpp sensor_msgs geometry_msgs std_msgs visualization_msgs
  cv_bridge tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(aruco_detector_sim ${OpenCV_LIBRARIES} yaml-cpp)

# Includes
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Install resources
install(DIRECTORY urdf launch xacro config include rviz worlds photos models maps
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS simple_controller twist_relay aruco_detector_sim
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  src/scan_repub.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
