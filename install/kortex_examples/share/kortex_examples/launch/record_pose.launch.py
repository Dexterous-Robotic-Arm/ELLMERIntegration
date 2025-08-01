#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '-m', 'kortex_examples.ELLMER.pose_recorder'],
            output='screen',
        ),
    ])
