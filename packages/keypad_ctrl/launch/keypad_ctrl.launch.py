import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("keypad_ctrl"),
        "config",
        "keypad_ctrl.yaml",
    )
    return LaunchDescription([
        Node(
            package="keypad_ctrl",
            executable="keypad_ctrl",
            name="keypad_ctrl",
            output="screen",
            parameters=[config],
        ),
    ])
