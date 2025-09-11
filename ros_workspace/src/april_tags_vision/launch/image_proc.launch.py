#!/usr/bin/env python3
"""
Image Processing Launch - Matches your friend's setup
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for image processing."""
    
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='cam_hand',
        description='Camera name'
    )
    
    # Image Processing Node
    image_proc_node = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc',
        output='screen',
        remappings=[
            ('image', '/camera/cam_hand/color/image_raw'),
            ('camera_info', '/camera/cam_hand/color/camera_info'),
            ('__ns', '/cam_hand'),
        ],
        parameters=[{
            'qos_overrides./camera/cam_hand/color/camera_info.reliability': 'best_effort'
        }]
    )
    
    # Log info
    log_info = LogInfo(
        msg="🖼️ Image processing launched"
    )
    
    return LaunchDescription([
        camera_name_arg,
        image_proc_node,
        log_info,
    ])
