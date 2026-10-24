cmake_minimum_required(VERSION 3.8)
project(camera_pkg)

# 手动设置 OpenCV（在 cv_bridge 之前）
set(OpenCV_FOUND TRUE)
set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")
set(OpenCV_LIBRARIES 
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_videoio
    opencv_highgui
    opencv_calib3d
    opencv_features2d
)
set(OpenCV_VERSION "4.8.0")

# 查找 ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 现在找 cv_bridge，它应该能通过上面设置的变量找到 OpenCV
find_package(cv_bridge REQUIRED)

# 创建可执行文件
add_executable(camera_node src/camera_node.cpp)

# 链接依赖
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# 添加 OpenCV 头文件
target_include_directories(camera_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_node ${OpenCV_LIBRARIES})

# 安装
install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
ament_package()