cmake_minimum_required(VERSION 3.8)
project(njord_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# インターフェース生成ツール
find_package(rosidl_default_generators REQUIRED)

# 依存する型パッケージ
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ==========================================
# インターフェース定義ファイルのリスト化
# ==========================================
# ここに作成した .action, .msg, .srv を全て列挙します
set(action_files
  "action/ConfigureSystem.action"
  "action/RunTask.action"
)

set(msg_files
  "msg/BuoyRoi.msg"
  "msg/BuoyDetection.msg"
  "msg/BuoyDetectionArray.msg"
  "msg/Thruster.msg"
  "msg/Bms.msg"
  "msg/ControlState.msg"
  "msg/HealthSignal.msg"
  "msg/HealthState.msg"
  "msg/MissionStatus.msg"
  "msg/Nav2RuntimeStatus.msg"
  "msg/OperatorCommand.msg"
  "msg/OperatorResponse.msg"
  "msg/TaskInfo.msg"
)

set(srv_files
  "srv/GetMissionStatus.srv"
  "srv/ListTasks.srv"
  "srv/SetControlMode.srv"
  "srv/StartTask.srv"
  "srv/StopTask.srv"
)

# ==========================================
# インターフェースの生成
# ==========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs std_msgs
)

# ==========================================
# エクスポート処理
# ==========================================
ament_export_dependencies(rosidl_default_runtime)

ament_package()
