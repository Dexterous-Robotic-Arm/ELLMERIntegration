#!/usr/bin/env python3
"""
April Tags Detector Launch File
===============================

Basic launch file for April Tags detection.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for April Tags detector."""
    
    # Declare launch arguments
    tag_size_arg = DeclareLaunchArgument(
        'tag_size_mm',
        default_value='100.0',
        description='Tag size in millimeters'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.8',
        description='Confidence threshold for detection'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera_link',
        description='Camera frame ID'
    )
    
    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame_id',
        default_value='base_link',
        description='Robot base frame ID'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    enable_3d_pose_arg = DeclareLaunchArgument(
        'enable_3d_pose',
        default_value='true',
        description='Enable 3D pose estimation'
    )
    
    enable_robot_coords_arg = DeclareLaunchArgument(
        'enable_robot_coords',
        default_value='true',
        description='Enable robot coordinate transformation'
    )
    
    show_debug_image_arg = DeclareLaunchArgument(
        'show_debug_image',
        default_value='true',
        description='Show debug image with detections'
    )
    
    # April Tags detector node
    apriltag_node = Node(
        package='april_tags_vision',
        executable='april_tags_detector_node.py',
        name='april_tags_detector',
        output='screen',
        parameters=[{
            'tag_size_mm': LaunchConfiguration('tag_size_mm'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'robot_frame_id': LaunchConfiguration('robot_frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'enable_3d_pose': LaunchConfiguration('enable_3d_pose'),
            'enable_robot_coords': LaunchConfiguration('enable_robot_coords'),
            'show_debug_image': LaunchConfiguration('show_debug_image'),
        }]
    )
    
    # Log info
    log_info = LogInfo(
        msg="🏷️ April Tags detector launched (TagStandard41h12, 100mm)"
    )
    
    return LaunchDescription([
        tag_size_arg,
        confidence_arg,
        camera_frame_arg,
        robot_frame_arg,
        publish_rate_arg,
        enable_3d_pose_arg,
        enable_robot_coords_arg,
        show_debug_image_arg,
        apriltag_node,
        log_info,
    ])
