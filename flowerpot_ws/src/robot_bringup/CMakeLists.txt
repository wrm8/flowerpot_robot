cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

# 查找依赖
find_package(ament_cmake REQUIRED)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 如果有配置文件目录
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

# 安装 Python 脚本（如果需要）
# install(PROGRAMS
#   scripts/your_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()