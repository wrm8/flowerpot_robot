cmake_minimum_required(VERSION 3.8)
project(decision_making_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs_pkg REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成 service 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartTransport.srv"
  DEPENDENCIES std_msgs
)

# PID控制器库
add_library(pid_controller STATIC
  src/pid_controller.cpp
)

target_include_directories(pid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(pid_controller PUBLIC cxx_std_17)

# 决策节点
add_executable(decision_making_node
  src/decision_making.cpp
)

# 设置 include 目录
target_include_directories(decision_making_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(decision_making_node PUBLIC cxx_std_17)

# 链接库
target_link_libraries(decision_making_node
  pid_controller
)

# ✅ 添加依赖（不要链接 ${PROJECT_NAME}）
ament_target_dependencies(decision_making_node
  rclcpp
  std_msgs
  common_msgs_pkg
  rosidl_default_runtime
)

# ✅ 添加这个：链接服务生成的库
target_link_libraries(decision_making_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 安装
install(TARGETS
  pid_controller
  decision_making_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装服务文件
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()