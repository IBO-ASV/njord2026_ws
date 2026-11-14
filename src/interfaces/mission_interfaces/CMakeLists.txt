cmake_minimum_required(VERSION 3.8)
project(mission_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunTask.action"
  "msg/MissionStatus.msg"
  "msg/TaskInfo.msg"
  "srv/GetMissionStatus.srv"
  "srv/ListTasks.srv"
  "srv/StartTask.srv"
  "srv/StopTask.srv"
  DEPENDENCIES action_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
