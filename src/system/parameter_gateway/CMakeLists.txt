cmake_minimum_required(VERSION 3.8)
project(parameter_gateway)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(runtime_interfaces REQUIRED)

# ライブラリの作成 (Component)
add_library(parameter_gateway_component SHARED src/node.cpp)
target_include_directories(parameter_gateway_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(parameter_gateway_component
  rclcpp
  rclcpp_components
  rclcpp_action
  rcl_interfaces
  runtime_interfaces
)

# 実行ファイルの作成
add_executable(parameter_gateway src/main.cpp) # main関数だけ別ファイルにする場合
# または下記のようにNode登録マクロを使っているのであれば、専用のmainを書かずにRCLCPP_COMPONENTS_REGISTER_NODEのみで運用し、
# 汎用コンテナ(rclcpp_components)でロードすることも可能です。
# ここではわかりやすく、シンプルなmain.cppを用意してリンクする方式を書きます。

target_link_libraries(parameter_gateway parameter_gateway_component)
ament_target_dependencies(parameter_gateway rclcpp rclcpp_components rclcpp_action rcl_interfaces runtime_interfaces)

# インストール設定
install(TARGETS parameter_gateway_component parameter_gateway
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
