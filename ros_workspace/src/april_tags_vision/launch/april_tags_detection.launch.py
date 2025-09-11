#!/usr/bin/env python3
"""
April Tags Detection Launch - Matches your friend's setup
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for April Tags detection."""
    
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='cam_hand',
        description='Camera name'
    )
    
    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.1',  # 100mm = 0.1m
        description='Tag size in meters'
    )
    
    tag_ids_arg = DeclareLaunchArgument(
        'tag_ids',
        default_value='[0,1,2,3,4,5,6,7,8,9,10]',
        description='Tag IDs to detect'
    )
    
    # April Tags Detection Node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_detector',
        output='screen',
        remappings=[
            ('image_rect', '/cam_hand/image_rect'),
            ('/cam_hand/camera_info', '/camera/cam_hand/color/camera_info'),
            ('detections', '/cam_hand/tag_detections'),
        ],
        parameters=[{
            'family': 'tagStandard41h12',
            'size': LaunchConfiguration('tag_size'),
            'pose_estimation_method': 'pnp',
            'z_up': True,
            'approximate_sync': True,
            'publish_tf': True,
            'qos_overrides./cam_hand/camera_info.reliability': 'best_effort',
            'qos_overrides./cam_hand/image_rect.reliability': 'best_effort',
            'tag.ids': LaunchConfiguration('tag_ids'),
            'tag.frames': ['tag41h12:0_hand', 'tag41h12:1_hand', 'tag41h12:2_hand', 
                          'tag41h12:3_hand', 'tag41h12:4_hand', 'tag41h12:5_hand',
                          'tag41h12:6_hand', 'tag41h12:7_hand', 'tag41h12:8_hand',
                          'tag41h12:9_hand', 'tag41h12:10_hand']
        }]
    )
    
    # Log info
    log_info = LogInfo(
        msg="🏷️ April Tags detection launched (TagStandard41h12, 100mm)"
    )
    
    return LaunchDescription([
        camera_name_arg,
        tag_size_arg,
        tag_ids_arg,
        apriltag_node,
        log_info,
    ])
