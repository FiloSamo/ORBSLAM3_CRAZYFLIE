from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    imu_streamer = Node(
            package='crazyflie_package',
            executable='imu_streamer',
            output='screen',  # Optional: Print output to screen
        )
    
    img_streamer = Node(
            package='crazyflie_package',
            executable='img_streamer',
            output='screen',  # Optional: Print output to screen
        )

    return LaunchDescription([
        imu_streamer , img_streamer,
    ])