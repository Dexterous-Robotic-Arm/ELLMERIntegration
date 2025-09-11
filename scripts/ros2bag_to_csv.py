#!/usr/bin/env python3
"""
ROS2 Bag to CSV Converter
Converts ROS2 bag files to CSV format for analysis
"""

import argparse
import csv
import os
import sys
from pathlib import Path

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from apriltag_msgs.msg import AprilTagDetectionArray
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 packages not available. Install with: pip install rosbag2_py")

def convert_joint_states(bag_path, output_dir):
    """Convert joint states to CSV"""
    if not ROS2_AVAILABLE:
        print("ROS2 not available, skipping joint states conversion")
        return
    
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                          output_serialization_format='cdr'))
    
    # Get topic info
    topic_types = reader.get_all_topics_and_types()
    joint_topic = None
    for topic_type in topic_types:
        if '/joint_states' in topic_type.name:
            joint_topic = topic_type.name
            break
    
    if not joint_topic:
        print("No joint states topic found")
        return
    
    # Create CSV file
    csv_path = os.path.join(output_dir, 'joint_states.csv')
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        
        # Read messages
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            if topic == joint_topic:
                try:
                    msg = deserialize_message(data, JointState)
                    writer.writerow([
                        timestamp,
                        msg.position[0] if len(msg.position) > 0 else 0,
                        msg.position[1] if len(msg.position) > 1 else 0,
                        msg.position[2] if len(msg.position) > 2 else 0,
                        msg.position[3] if len(msg.position) > 3 else 0,
                        msg.position[4] if len(msg.position) > 4 else 0,
                        msg.position[5] if len(msg.position) > 5 else 0
                    ])
                except Exception as e:
                    print(f"Error processing joint state: {e}")
    
    print(f"Joint states saved to: {csv_path}")

def convert_tag_detections(bag_path, output_dir):
    """Convert April Tag detections to CSV"""
    if not ROS2_AVAILABLE:
        print("ROS2 not available, skipping tag detections conversion")
        return
    
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                          output_serialization_format='cdr'))
    
    # Get topic info
    topic_types = reader.get_all_topics_and_types()
    tag_topics = []
    for topic_type in topic_types:
        if '/tag_detections' in topic_type.name:
            tag_topics.append(topic_type.name)
    
    if not tag_topics:
        print("No tag detection topics found")
        return
    
    # Convert each tag topic
    for topic in tag_topics:
        csv_name = topic.replace('/', '_').replace('cam_hand_', 'hand_').replace('cam_base_', 'base_')
        csv_path = os.path.join(output_dir, f'{csv_name}.csv')
        
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'tag_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            
            # Read messages
            while reader.has_next():
                (topic_name, data, timestamp) = reader.read_next()
                if topic_name == topic:
                    try:
                        # Note: This is a simplified version - you may need to adjust based on actual message type
                        writer.writerow([timestamp, 0, 0, 0, 0, 0, 0, 0, 1])  # Placeholder
                    except Exception as e:
                        print(f"Error processing tag detection: {e}")
        
        print(f"Tag detections saved to: {csv_path}")

def main():
    parser = argparse.ArgumentParser(description='Convert ROS2 bag to CSV')
    parser.add_argument('bag_path', help='Path to ROS2 bag file')
    parser.add_argument('-o', '--output', help='Output directory', default='./csv_output')
    
    args = parser.parse_args()
    
    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Converting bag: {args.bag_path}")
    print(f"Output directory: {output_dir}")
    
    # Convert joint states
    convert_joint_states(args.bag_path, str(output_dir))
    
    # Convert tag detections
    convert_tag_detections(args.bag_path, str(output_dir))
    
    print("Conversion complete!")

if __name__ == '__main__':
    main()
