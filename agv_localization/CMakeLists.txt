cmake_minimum_required(VERSION 3.8)
project(agv_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================
# DEPENDENCIES
# ============================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# ============================================
# PYTHON PACKAGE
# ============================================
ament_python_install_package(${PROJECT_NAME})

# ============================================
# INSTALL - Config và Launch files
# ============================================
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

