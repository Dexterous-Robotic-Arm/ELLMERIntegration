#!/usr/bin/env python3
"""
Launch file for April Tags Detector with Robot Integration
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():
    """Generate launch description for April Tags detector with robot."""
    
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
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.241',
        description='Robot IP address'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='xarm',
        description='Robot type (xarm, ur, etc.)'
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
    
    # April Tags Detector Node
    april_tags_node = Node(
        package='april_tags_vision',
        executable='april_tags_detector_node.py',
        name='april_tags_detector',
        output='screen',
        parameters=[{
            'tag_size_mm': LaunchConfiguration('tag_size_mm'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'camera_frame_id': 'camera_link',
            'robot_frame_id': 'base_link',
            'publish_rate': 10.0,
            'enable_3d_pose': True,
            'enable_robot_coords': LaunchConfiguration('enable_robot_coords'),
            'show_debug_image': LaunchConfiguration('show_debug_image'),
            'robot_ip': LaunchConfiguration('robot_ip'),
        }]
    )
    
    # RealSense Camera Node
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
        }]
    )
    
    # Robot Description (if available)
    robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_robot_coords'))
    )
    
    # Static Transform Publisher for camera
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_transform_publisher',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            os.path.dirname(__file__), '..', 'config', 'april_tags_robot.rviz'
        )],
        condition=IfCondition(LaunchConfiguration('show_debug_image'))
    )
    
    # Log info
    log_info = LogInfo(
        msg="🤖 April Tags Detector launched with robot integration"
    )
    
    return LaunchDescription([
        tag_size_arg,
        confidence_threshold_arg,
        robot_ip_arg,
        robot_type_arg,
        enable_robot_coords_arg,
        show_debug_image_arg,
        realsense_node,
        static_transform_publisher,
        robot_description_node,
        april_tags_node,
        rviz_node,
        log_info,
    ])
