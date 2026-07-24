cmake_minimum_required(VERSION 3.8)
project(ros_uwb)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UwbResult.msg"
  "msg/UwbPeerInfo.msg"
  "srv/SetUserData.srv"
  DEPENDENCIES std_msgs
)

ament_python_install_package(pyuwb
  PACKAGE_DIR pyuwb/src/pyuwb
)

install(PROGRAMS
  scripts/uwb_node.py
  RENAME uwb_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_uwb_node test/test_uwb_node.py)
  ament_add_pytest_test(test_pyuwb pyuwb/tests)
endif()

ament_package()
