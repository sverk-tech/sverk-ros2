cmake_minimum_required(VERSION 3.8)
project(aruco_det_loc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)  # component composition
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)

# ----- Messages -----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ----- Include dirs (for headers like aruco_det_loc/aruco_loc_utils.hpp) -----
include_directories(include)

# Common ament deps lists (avoids repetition)
set(DETECT_DEPS
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)

set(LOC_DEPS
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)

set(TF_DEPS
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---------------------------------------------------------------------------
# aruco_detect — build as shared library for component composition.
# rclcpp_components_register_node also creates the standalone executable.
# ---------------------------------------------------------------------------
add_library(aruco_detect_component SHARED src/aruco_detect_node.cpp)
ament_target_dependencies(aruco_detect_component ${DETECT_DEPS})
rosidl_target_interfaces(aruco_detect_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
# Generates the 'aruco_detect_node' executable that loads the component
rclcpp_components_register_node(aruco_detect_component
  PLUGIN "aruco_det_loc::ArucoDetectNode"
  EXECUTABLE aruco_detect_node
)
install(TARGETS aruco_detect_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------------------------------------------------------------------
# aruco_loc — build as shared library for component composition.
# ---------------------------------------------------------------------------
add_library(aruco_loc_component SHARED src/aruco_loc_node.cpp)
ament_target_dependencies(aruco_loc_component ${LOC_DEPS})
rosidl_target_interfaces(aruco_loc_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rclcpp_components_register_node(aruco_loc_component
  PLUGIN "aruco_det_loc::ArucoLocNode"
  EXECUTABLE aruco_loc_node
)
install(TARGETS aruco_loc_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------------------------------------------------------------------
# aruco_tf_node — TF publisher component (offset + world-frame pose for VPE)
# ---------------------------------------------------------------------------
add_library(aruco_tf_component SHARED src/aruco_tf_node.cpp)
ament_target_dependencies(aruco_tf_component ${TF_DEPS})
rclcpp_components_register_node(aruco_tf_component
  PLUGIN "aruco_det_loc::ArucoTfNode"
  EXECUTABLE aruco_tf_node
)
install(TARGETS aruco_tf_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Установка launch файлов
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Per-platform CPU / threading / coarse-to-fine parameter files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts (diagnostic helpers)
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# ----- Install headers -----
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
