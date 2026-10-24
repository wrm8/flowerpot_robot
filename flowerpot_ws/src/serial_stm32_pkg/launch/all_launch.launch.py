from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 1. 决策节点
        Node(
            package="decision_making_pkg",
            executable="decision_making_node",
            name="decision_making_node",
            output="screen",
            emulate_tty=True,
        ),

        # 2. 串口节点
        Node(
            package="serial_stm32_pkg",
            executable="serial_stm32_node",
            name="serial_stm32_node",
            output="screen",
            emulate_tty=True,
        ),

        # 你想加什么节点，就继续往下加...

    ])