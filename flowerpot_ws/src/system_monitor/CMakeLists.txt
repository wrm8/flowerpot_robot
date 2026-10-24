cmake_minimum_required(VERSION 3.8)
project(node_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 添加可执行文件
add_executable(node_monitor
  src/node_monitor.cpp
)

target_include_directories(node_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(node_monitor PUBLIC cxx_std_17)

# 链接依赖
ament_target_dependencies(node_monitor
  rclcpp
  std_msgs
  ament_index_cpp
)

# 安装
install(TARGETS node_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()cmake_minimum_required(VERSION 3.8)
project(node_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 添加可执行文件
add_executable(node_monitor
  src/node_monitor.cpp
)

target_include_directories(node_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(node_monitor PUBLIC cxx_std_17)

# 链接依赖
ament_target_dependencies(node_monitor
  rclcpp
  std_msgs
  ament_index_cpp
)

# 安装
install(TARGETS node_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()