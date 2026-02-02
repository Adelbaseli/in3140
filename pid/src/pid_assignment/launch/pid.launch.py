#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    perspective_file = os.path.join(
        get_package_share_directory('pid_assignment'),
        'launch',
        'crustcrawler.perspective'
    )

    return LaunchDescription([
        # Launch PID controller node
        Node(
            package='pid_assignment',
            executable='node',  # <-- corrected here
            name='pid_controller',
            output='screen',
        ),

        # Launch RQT with PID perspective
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='pid_rqt',
            output='screen',
            arguments=['--perspective-file', perspective_file],
            respawn=False,
        ),
    ])
