cmake_minimum_required(VERSION 3.8)
project(px4_local_pose_publisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros_com REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(pose_subscriber_tf2
  src/pose_subscriber_tf2.cpp
)

target_include_directories(pose_subscriber_tf2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(pose_subscriber_tf2 PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(pose_subscriber_tf2
  rclcpp
  geometry_msgs
  std_msgs
  px4_msgs
  px4_ros2_cpp
  px4_ros_com
)

# LPE по лидарной одометрии (3D-лидар + point_lio) — аналог pose_subscriber_tf2
add_executable(lidar_odom_publisher
  src/lidar_odom_publisher.cpp
)

target_include_directories(lidar_odom_publisher PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(lidar_odom_publisher
  rclcpp
  nav_msgs
  px4_msgs
  px4_ros2_cpp
  px4_ros_com
)

install(TARGETS pose_subscriber_tf2 lidar_odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()