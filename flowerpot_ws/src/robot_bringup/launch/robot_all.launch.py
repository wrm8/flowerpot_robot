from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    workspace = '/home/wrm/wrm/flower_robot-master_WRM/flowerpot_ws'
    
    # 获取各个包的路径
    lslidar_dir = get_package_share_directory('lslidar_driver')
    
    return LaunchDescription([

        ExecuteProcess(
            cmd=["bash", "-c", "chmod 666 /dev/ttyCH341USB0 || true; sleep 2; ros2 run imu_pkg imu_node"],
            name="imu_node",
            output="screen"
        ),
        
        # 2. radar_pkg 节点
        Node(
            package='radar_pkg',
            executable='radar_node',
            name='radar_node',
            output='screen'
        ),
        
        # 3. camera_pkg 节点
        Node(
            package='camera_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        
        # 4. radaryolo11_meld__pkg 节点
        Node(
            package='radaryolo11_meld__pkg',
            executable='radaryolo11_meld_node',
            name='radaryolo11_meld_node',
            output='screen'
        ),
        
        # 5. gps_pkg 节点
        Node(
            package='gps_pkg',
            executable='gps_node',
            name='gps_node',
            output='screen'
        ),
        
        # 6. potbot_hmi 节点
        Node(
            package='potbot_hmi',
            executable='potbot_hmi_node',
            name='potbot_hmi_node',
            output='screen'
        ),
        
        # 7. serial_stm32_pkg 节点
        Node(
            package='serial_stm32_pkg',
            executable='serial_stm32_node',
            name='serial_stm32_node',
            output='screen'
        ),
        
        # 8. decision_making_pkg节点
        Node(
            package='decision_making_pkg',
            executable='decision_making_node',
            name='decision_making_node',
            output='screen',
            emulate_tty=True
        ),
        # # 1. IMU 节点
        # Node(
        #     package='imu_pkg',
        #     executable='imu_node',
        #     name='imu_node',
        #     output='screen',
        #      # 加这一行！延迟 2~3 秒启动
        #     parameters=[{"start_delay": 2.0}],
        # ),
        
        # 9. YOLO 检测节点
        ExecuteProcess(
            cmd=[
                'python3',
                f'{workspace}/src/yolo11_pkg/scripts/onnx.py',
                '--model', f'{workspace}/src/yolo11_pkg/scripts/best.onnx',
                '--yaml', f'{workspace}/src/yolo11_pkg/scripts/flower.yaml'
            ],
            name='yolo_node',
            output='screen'
        ),
        
        # 10. 激光雷达驱动 - 包含其 launch 文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lslidar_dir, 'launch', 'lsm10p_uart_launch.py')
            )
        ),
    ])