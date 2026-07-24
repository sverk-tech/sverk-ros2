cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(offboard_interfaces REQUIRED)
find_package(PkgConfig REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)


include_directories(include)

add_executable(offboard_control src/offboard_control.cpp)
target_include_directories(offboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

# Установка C++17 стандарта
target_compile_features(offboard_control PUBLIC cxx_std_17)

ament_target_dependencies(offboard_control
 rclcpp
 rclcpp_action
 px4_msgs
 px4_ros_com
 std_srvs
 offboard_interfaces
 std_msgs
 geometry_msgs

 tf2_ros
 tf2_geometry_msgs
)

# В target_link_libraries добавить:
# target_link_libraries(${PROJECT_NAME}
#   ${Eigen3_LIBRARIES}
#   tf2::tf2
# )

install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Header-only excitation signal library (no ROS/PX4 deps) — gtest.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_excitation_signals test/test_excitation_signals.cpp)
  if(TARGET test_excitation_signals)
    target_include_directories(test_excitation_signals PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_compile_features(test_excitation_signals PRIVATE cxx_std_17)
  endif()

  # Node-independent logic: transforms, SpeedController, setpoint validation,
  # diagnostics string mappings (prompt 102). No ROS graph needed at runtime.
  ament_add_gtest(test_offboard_logic test/test_offboard_logic.cpp)
  if(TARGET test_offboard_logic)
    target_include_directories(test_offboard_logic PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${Eigen3_INCLUDE_DIRS})
    target_compile_features(test_offboard_logic PRIVATE cxx_std_17)
    ament_target_dependencies(test_offboard_logic
      rclcpp rclcpp_action px4_msgs px4_ros_com std_srvs offboard_interfaces
      std_msgs geometry_msgs tf2_ros tf2_geometry_msgs)
  endif()
endif()

ament_package()