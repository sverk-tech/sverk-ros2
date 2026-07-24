cmake_minimum_required(VERSION 3.8)
project(offboard_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GetParam.srv"
    "srv/ResetParam.srv"
    "srv/BackupParams.srv"
    "srv/MotorTest.srv"
    "srv/GetTelemetry.srv"
    "srv/Navigate.srv"
    "srv/NavigateGlobal.srv"
    "srv/SetAltitude.srv"
    "srv/SetAttitude.srv"
    "srv/SetParam.srv"
    "srv/SetPosition.srv"
    "srv/SetRates.srv"
    "srv/SetTrajectory.srv"
    "srv/SetVelocity.srv"
    "srv/SetYaw.srv"
    "srv/SetYawRate.srv"
    "srv/Flip.srv"
    "srv/StopExcitation.srv"
    "srv/EmergencyStop.srv"
    "srv/ReturnToLaunch.srv"

    "action/Calibrate.action"
    "action/DownloadLog.action"
    "action/RunExcitation.action"
    "action/ReceiveTelemetry.action"
    "action/NavigateTo.action"

    "msg/State.msg"
    "msg/Param.msg"
    "msg/ExcitationSample.msg"
    "msg/Telemetry.msg"
    "msg/OffboardStatus.msg"
    "msg/TrajectoryCorrection.msg"

    DEPENDENCIES action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
