#!/usr/bin/env python3
from launch     import LaunchDescription
from launch_ros import actions

def generate_launch_description():
    return LaunchDescription([
        actions.Node(
            package='kortex_examples',
            executable='pose_recorder',
            output='screen',
        ),
    ])
