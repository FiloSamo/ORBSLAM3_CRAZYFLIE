from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    slam_pkg_path= get_package_share_directory("orb_slam3")

    vocab_file = os.path.join(slam_pkg_path,"config","ORBvoc.txt")
    settings_file = os.path.join(slam_pkg_path,"config","camera_and_slam_settings.yaml")


    slam_node = Node(
            package='orb_slam3',
            executable='orb_slam3',
            output='screen',  # Optional: Print output to screen
            parameters=[
              {"vocab_path":vocab_file},
              {"config_path":settings_file}]
        )

    return LaunchDescription([
        slam_node
    ])