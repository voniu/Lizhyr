from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory("lizhyr_restorer")
    params_file = os.path.join(share_dir, "config", "restorer.yaml")

    return LaunchDescription([
        Node(
            package="lizhyr_restorer",
            executable="restorer_node",
            name="restorer_node",
            parameters=[params_file],
            output="screen",
        )
    ])
