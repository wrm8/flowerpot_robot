#camera
#ssh远程加载图形界面
export DISPLAY=:0
#把压缩图像转化为原始图像
ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=/image_raw/compressed -r out:=/image_raw
#开启摄像头,不需要解压缩，直接出来个窗口，显示画面
ros2 run rqt_image_view rqt_image_view
#修改文件内容
cat > 文件精确地址 << 'EOF'
修改的内容
EOF

cat > ~/wrm/flower_robot-master_WRM/flowerpot_ws/src/radar_pkg/scripts/lidar_visualize.py << 'EOF'
修改的内容
EOF

cat > ~/wrm/flower_robot-master_WRM/flowerpot_ws/src/radar_pkg/src/radar_node.cpp << 'EOF'
修改的内容
EOF

colcon build --packages-select yolo11_msgs yolo11_pkg camera_pkg radaryolo11_meld__pkg lslidar_driver lslidar_msgs radar_msgs radar_pkg common_msgs_pkg imu_pkg gps_pkg decision_making_pkg serial_stm32_pkg potbot_hmi robot_bringup

colcon build --packages-select potbot_hmi
colcon build --packages-select gps_pkg
colcon build --packages-select serial_stm32_pkg
colcon build --packages-select common_msgs_pkg decision_making_pkg
source install/setup.bash
ros2 launch robot_bringup robot_all.launch.py    //所有节点
ros2 run potbot_hmi potbot_hmi_node
ros2 run imu_pkg imu_node
ros2 run gps_pkg gps_node
ros2 run serial_stm32_pkg serial_stm32_node
ros2 run decision_making_pkg decision_making_node
ros2 run radaryolo11_meld__pkg radaryolo11_meld_node

ros2 param set /decision_making_node speed_kp 0.5
ros2 param set /decision_making_node speed_ki 0.00
ros2 param set /decision_making_node speed_kd 0.00