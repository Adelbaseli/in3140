#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to Crustcrawler Gazebo controller launch file
    controller_launch_file = os.path.join(
        get_package_share_directory('crustcrawler_pen_gazebo'),
        'launch',
        'controller.launch.py'  # Make sure this exists in ROS 2 version
    )

    return LaunchDescription([
        # Include Gazebo controller launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch_file),
            launch_arguments={'control': 'effort'}.items()
        ),

        # Launch one-shot node to fix joints
        Node(
            package='pid_assignment',
            executable='fixer.py',
            name='gazebo_fix',
            output='screen'
        ),
    ])
