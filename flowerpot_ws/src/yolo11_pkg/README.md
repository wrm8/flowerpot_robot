#编译
colcon build --packages-select yolo11_msgs yolo11_pkg camera_pkg radaryolo11_meld__pkg
source install/setup.bash

ros2 run camera_pkg camera_node
ros2 run radaryolo11_meld__pkg radaryolo11_meld_node

#ssh远程加载图形界面
export DISPLAY=:0

#开启摄像头,不需要解压缩，直接出来个窗口，显示画面
ros2 run rqt_image_view rqt_image_view

#运行onnx.py,不显示画面
python3 src/yolo11_pkg/scripts/onnx.py \
  --model src/yolo11_pkg/scripts/best.onnx \
  --yaml src/yolo11_pkg/scripts/flower.yaml
#+show显示画面
python3 src/yolo11_pkg/scripts/onnx.py \
  --model src/yolo11_pkg/scripts/best.onnx \
  --yaml src/yolo11_pkg/scripts/flower.yaml \
  --show

#查看所有话题
ros2 topic list
ros2 node list //查看正在运行的节点
ros2 topic list -v //查看订阅发布情况
ros2 topic echo /yolo11_data --field array
# 查看环境变量中是否包含了你的消息包路径
echo $AMENT_PREFIX_PATH