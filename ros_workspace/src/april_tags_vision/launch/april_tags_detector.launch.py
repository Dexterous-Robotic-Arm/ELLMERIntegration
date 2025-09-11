#!/usr/bin/env python3
"""
Launch file for April Tags Detector Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():
    """Generate launch description for April Tags detector."""
    
    # Declare launch arguments
    tag_size_arg = DeclareLaunchArgument(
        'tag_size_mm',
        default_value='50.0',
        description='Physical size of April Tags in millimeters'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.8',
        description='Minimum confidence for tag detection (0.0-1.0)'
    )
    
    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera_link',
        description='Camera frame ID'
    )
    
    robot_frame_id_arg = DeclareLaunchArgument(
        'robot_frame_id',
        default_value='base_link',
        description='Robot base frame ID'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Detection publish rate in Hz'
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
        default_value='false',
        description='Show debug image with detections'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.241',
        description='Robot IP address'
    )
    
    use_realsense_arg = DeclareLaunchArgument(
        'use_realsense',
        default_value='true',
        description='Use RealSense camera directly'
    )
    
    # April Tags Detector Node
    april_tags_node = Node(
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
            'robot_ip': LaunchConfiguration('robot_ip'),
        }]
    )
    
    # RealSense Camera Node (optional)
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30.0,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_realsense'))
    )
    
    # RViz for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            os.path.dirname(__file__), '..', 'config', 'april_tags.rviz'
        )],
        condition=IfCondition(LaunchConfiguration('show_debug_image'))
    )
    
    # Log info
    log_info = LogInfo(
        msg="🏷️ April Tags Detector launched with RealSense camera"
    )
    
    return LaunchDescription([
        tag_size_arg,
        confidence_threshold_arg,
        camera_frame_id_arg,
        robot_frame_id_arg,
        publish_rate_arg,
        enable_3d_pose_arg,
        enable_robot_coords_arg,
        show_debug_image_arg,
        robot_ip_arg,
        use_realsense_arg,
        realsense_node,
        april_tags_node,
        rviz_node,
        log_info,
    ])
