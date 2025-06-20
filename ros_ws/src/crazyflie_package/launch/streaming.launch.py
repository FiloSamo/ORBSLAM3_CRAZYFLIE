#     __________  _____ ____     __  __      _ __        
#    / ____/ __ \/ ___// __ \   / / / /___  (_) /_  ____ 
#   / /   / /_/ /\__ \/ /_/ /  / / / / __ \/ / __ \/ __ \
#  / /___/ ____/___/ / ____/  / /_/ / / / / / /_/ / /_/ /
#  \____/_/    /____/_/       \____/_/ /_/_/_.___/\____/ 
                                                       
                                                               
#  Authors: Filippo Samorì, Filippo Ugolini and Daniele Crivellari
#  20/06/2025
#  University of Bologna, Italy
#  License: BSD-3-Clause

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    uri = LaunchConfiguration("URI", default='radio://0/86/2M/E7E7E7E7ED')

    ip = LaunchConfiguration("IP", default='192.168.4.1')
    port = LaunchConfiguration("PORT", default=5000)

    log_active = LaunchConfiguration("LOG_ACTIVE", default=False) # Default to False, can be set to True to enable file logging
    
    imu_streamer = Node(
            package='crazyflie_package',
            executable='imu_streamer',
            output='screen',  # Optional: Print output to screen
            parameters=[{'URI': uri, 'log_active': log_active}],
        )
    
    img_streamer = Node(
            package='crazyflie_package',
            executable='img_streamer',
            output='screen',  # Optional: Print output to screen
            parameters=[{'ip': ip, 'port': port}],
        )

    return LaunchDescription([
        imu_streamer , img_streamer,
    ])