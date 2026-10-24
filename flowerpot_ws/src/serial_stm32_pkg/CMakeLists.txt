cmake_minimum_required(VERSION 3.8)
project(serial_stm32_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_msgs_pkg REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

# 查找 libserial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

add_executable(serial_stm32_node
  src/serial_comm.cpp
)

target_include_directories(serial_stm32_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_compile_features(serial_stm32_node PUBLIC cxx_std_17)

target_link_libraries(serial_stm32_node
  ${LIBSERIAL_LIBRARIES}
)

ament_target_dependencies(serial_stm32_node
  rclcpp
  common_msgs_pkg
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  std_msgs
)

install(TARGETS serial_stm32_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()