#!/usr/bin/env python3
"""
RealSense Camera Launch - Matches your friend's setup
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for RealSense camera."""
    
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='cam_hand',
        description='Camera name'
    )
    
    serial_no_arg = DeclareLaunchArgument(
        'serial_no',
        default_value='153122078759',
        description='RealSense camera serial number'
    )
    
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='file://' + os.path.expanduser('~/ELLMERIntegration/ros_workspace/src/april_tags_vision/config/cam_hand.yaml'),
        description='Camera calibration file URL'
    )
    
    # RealSense Camera Node
    realsense_node = Node(
        package='realsense2_camera',
        executable='rs_launch.py',
        name='realsense_camera',
        output='screen',
        parameters=[{
            'camera_name': LaunchConfiguration('camera_name'),
            'serial_no': LaunchConfiguration('serial_no'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'enable_color': True,
            'enable_depth': True,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30.0,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30.0,
            # Fix QoS compatibility issues
            'qos_overrides./camera/cam_hand/color/camera_info.reliability': 'best_effort',
            'qos_overrides./camera/cam_hand/color/camera_info.durability': 'volatile',
            'qos_overrides./camera/cam_hand/color/image_raw.reliability': 'best_effort',
            'qos_overrides./camera/cam_hand/color/image_raw.durability': 'volatile',
        }]
    )
    
    # Log info
    log_info = LogInfo(
        msg="📷 RealSense camera launched with calibration"
    )
    
    return LaunchDescription([
        camera_name_arg,
        serial_no_arg,
        camera_info_url_arg,
        realsense_node,
        log_info,
    ])
