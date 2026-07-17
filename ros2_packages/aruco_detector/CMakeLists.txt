cmake_minimum_required(VERSION 3.16)
project(aruco_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4.6 REQUIRED)

# Include directories
include_directories(include)

# Main executable
add_executable(aruco_detector_node 
  src/aruco_detector_node.cpp
)

ament_target_dependencies(aruco_detector_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)

target_include_directories(aruco_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the executable
install(TARGETS aruco_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  set(ament_cmake_copyright_FOUND TRUE)
#  ament_lint_auto_find_test_files()
#endif()

ament_package()
