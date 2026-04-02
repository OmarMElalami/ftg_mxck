cmake_minimum_required(VERSION 3.5)
project(follow_the_gap_v0)

# Keep the language level unchanged for Foxy compatibility.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacle_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(follow_the_gap
  src/main.cpp
  src/follow_the_gap.cpp
  src/obstacle.cpp
  src/gap.cpp
  src/corner.cpp
  src/lidar_data.cpp
)

target_include_directories(follow_the_gap
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(follow_the_gap
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  obstacle_msgs
  tf2_geometry_msgs
)

install(
  TARGETS follow_the_gap
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
