from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_interface',
            executable='stm32_bridge',
            name='stm32_bridge',
            output='screen'
        )
    ])

    