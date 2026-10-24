cmake_minimum_required(VERSION 3.8)
project(potbot_hmi)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(decision_making_pkg REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

include_directories(
  include
)

# 源文件 + 手动生成的 moc 文件
set(SOURCES
  src/main.cpp
  src/mainwindow.cpp
  src/ros_interface.cpp
  moc_mainwindow.cpp
)

add_executable(potbot_hmi_node ${SOURCES})

ament_target_dependencies(potbot_hmi_node
  rclcpp
  std_msgs
  decision_making_pkg
)

target_link_libraries(potbot_hmi_node
  Qt5::Widgets
)

install(TARGETS potbot_hmi_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
