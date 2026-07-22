cmake_minimum_required(VERSION 3.8)
project(haply_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandleButtons.msg"
  "msg/Inverse3State.msg"
  "msg/HandleState.msg"
  "msg/HaplyState.msg"
  "msg/HaplyControl.msg"
  "msg/StudyTask.msg"
  "msg/StudySession.msg"
  "msg/StudyTrialState.msg"
  "msg/StudyDwellProgress.msg"
  "msg/StudyStartRequest.msg"
  "msg/StudyAbortRequest.msg"
  "msg/StudyCursor.msg"
  "msg/StudyButtonPress.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
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
