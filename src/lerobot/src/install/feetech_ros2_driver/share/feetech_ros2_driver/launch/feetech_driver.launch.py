from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='feetech_ros2_driver',
            executable='feetech_driver_node',
            output='screen',
            name='feetech_driver'
        ),
        Node(
            package='feetech_ros2_driver',
            executable='feetech_interface_node',
            output='screen',
            name='feetech_interface'
        )
    ])
