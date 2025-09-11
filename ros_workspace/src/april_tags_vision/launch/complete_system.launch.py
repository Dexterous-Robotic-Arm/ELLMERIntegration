#!/usr/bin/env python3
"""
Complete System Launch - All components together
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for complete system."""
    
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
        default_value='file:///home/apriltag_ws/calibration/cam_hand.yaml',
        description='Camera calibration file URL'
    )
    
    # Include RealSense camera
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/Users/eugenevorobiov/ELLMERIntegration/ros_workspace/src/april_tags_vision/launch/realsense_camera.launch.py'
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'serial_no': LaunchConfiguration('serial_no'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
        }.items()
    )
    
    # Include image processing
    image_proc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/Users/eugenevorobiov/ELLMERIntegration/ros_workspace/src/april_tags_vision/launch/image_proc.launch.py'
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
        }.items()
    )
    
    # Include April Tags detection
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/Users/eugenevorobiov/ELLMERIntegration/ros_workspace/src/april_tags_vision/launch/april_tags_detection.launch.py'
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
        }.items()
    )
    
    # Robot Control Node
    robot_control_node = Node(
        package='april_tags_vision',
        executable='april_tags_detector_node.py',
        name='robot_control',
        output='screen',
        parameters=[{
            'tag_size_mm': 100.0,
            'confidence_threshold': 0.8,
            'camera_frame_id': 'cam_hand_color_optical_frame',
            'robot_frame_id': 'base_link',
            'publish_rate': 10.0,
            'enable_3d_pose': True,
            'enable_robot_coords': True,
            'show_debug_image': True,
            'robot_ip': '192.168.1.241',
        }]
    )
    
    # Log info
    log_info = LogInfo(
        msg="🤖 Complete April Tags robot system launched"
    )
    
    return LaunchDescription([
        camera_name_arg,
        serial_no_arg,
        camera_info_url_arg,
        realsense_launch,
        image_proc_launch,
        apriltag_launch,
        robot_control_node,
        log_info,
    ])
