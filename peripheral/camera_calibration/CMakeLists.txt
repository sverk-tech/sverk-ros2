cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetCalibrationConfig.srv"
  "srv/SetAutoCapture.srv"
  "srv/GetState.srv"
  "srv/ListCaptures.srv"
  "srv/RemoveCapture.srv"
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  src/camera_calibration_node.py
  src/verify_calibration_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()