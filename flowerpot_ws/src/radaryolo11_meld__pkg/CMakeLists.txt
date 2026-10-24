cmake_minimum_required(VERSION 3.8)
project(radaryolo11_meld__pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(yolo11_msgs REQUIRED)


# 匈牙利算法库（头文件即可，因为都是inline）
add_library(hungarian_algorithm INTERFACE)
target_include_directories(hungarian_algorithm INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(hungarian_algorithm INTERFACE cxx_std_17)

# 主节点
add_executable(radaryolo11_meld_node
  src/radaryolo11_meld_node.cpp
)

ament_target_dependencies(
  radaryolo11_meld_node
  rclcpp
  std_msgs
  radar_msgs
  yolo11_msgs
)

target_include_directories(radaryolo11_meld_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(radaryolo11_meld_node PUBLIC cxx_std_17)

target_link_libraries(radaryolo11_meld_node
  hungarian_algorithm
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# 安装
install(TARGETS
  radaryolo11_meld_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()