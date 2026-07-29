cmake_minimum_required(VERSION 3.8)
project(landing_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(landing_controller src/landing_controller.cpp)

ament_target_dependencies(landing_controller
  rclcpp
  geometry_msgs
  px4_msgs
)

target_include_directories(landing_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS landing_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_landing_math test/test_landing_math.cpp)
  target_include_directories(test_landing_math PRIVATE include)
endif()

ament_package()
