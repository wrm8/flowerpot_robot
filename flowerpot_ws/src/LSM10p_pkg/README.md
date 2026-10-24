# lslidar

## Description
The `lslidar package is a linux ROS2 driver for lslidar M10 ,M10_GPS,M10_P,M10_PLUS and N10.
The package is tested on Ubuntu 20.04 with ROS2 indigo.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
colcon build
source install/setup.bash
ros2 launch lslidar_driver lslidar_launch.py
```
open new terminal
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 1\ 		(open radar)
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 0\ 		(close radar)


```

Note that this launch file launches both the driver, which is the only launch file needed to be used.


## FAQ


## Bug Report

Prefer to open an issue. You can also send an E-mail to honghangli@lslidar.com




RERTION 

#######################################################

#编译
colcon build --packages-select lslidar_driver lslidar_msgs radar_msgs radar_pkg
source install/setup.bash
#运行串口版
ros2 launch lslidar_driver lsm10p_uart_launch.py  //激光雷达驱动，发布点云
//ros2 launch lslidar_driver lslidar_launch.py

ros2 run radar_pkg radar_node  //订阅 /scan 雷达数据,点云处理，发布目标位置 /radar_targets

#运行可视化
python3 src/radar_pkg/scripts/lidar_visualize.py  //实时显示处理过的聚类

##SSH
/export DISPLAY=:0/ros2 run rviz2 rviz2   //实时显示未处理的雷达点云
###########
#列出串口，要注意用正确的串口，要不没数据
ls /dev/ttyUSB*
ls /dev/ttyACM*
ls /dev/tty*
ls /dev/tty* | grep -E "ttyUSB|ttyACM|ttyAMA|ttyTHS"  //查看所有串口
lsusb//查看usb
# 创建 ttyACM0 指向实际设备
sudo ln -s /dev/ttyCH343USB0 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyCH343USB0
ls -la /dev/ttyACM0 //验证
cat /dev/ttyCH341USB0 //输出串口数据
# 如果有串口但识别不成功，临时授予权限
sudo chmod 666 /dev/ttyACM1
# 或者将用户添加到 dialout 组（永久），需要重新登录生效
sudo usermod -a -G dialout $USER
# 一直获取激光雷达数据
ros2 topic echo /scan
# 获取一帧完整的激光雷达数据
ros2 topic echo /scan --once
# 保存完整的一帧数据（包括所有角度）
ros2 topic echo /scan --once --field ranges > scan_360.txt
#保存一帧所有的数据，不删减
ros2 topic echo /scan --once --full > scan_full.txt

# 查看有多少个点（应该是 360度 / 角度增量）
wc -l scan_360.txt

#动态调参
# 调整固定半径范围（根据实际花盆大小）
ros2 param set /radar_node fixed_min_radius 0.06
ros2 param set /radar_node fixed_max_radius 0.22

# 调整墙角过滤灵敏度（比值越小越容易过滤直线，0.1表示非常敏感）
ros2 param set /radar_node wall_eigenvalue_ratio 0.15

# 关闭墙角过滤
ros2 param set /radar_node enable_corner_filter false

# 调整检测距离范围
ros2 param set /radar_node max_range 5.0

# 调整自适应分段阈值
ros2 param set /radar_node near_thresh 0.6
ros2 param set /radar_node mid_thresh 1.5
