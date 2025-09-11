# April Tags Vision ROS Package

A ROS 2 package for April Tags detection with RealSense camera integration and robot coordinate transformation.

## Features

- **April Tags Detection**: Detects TagStandard41h12 family April Tags
- **3D Pose Estimation**: Calculates 3D pose of detected tags
- **RealSense Integration**: Works with Intel RealSense D435 camera
- **Robot Integration**: Transforms coordinates to robot base frame
- **ROS 2 Compatible**: Full ROS 2 Humble/Iron support
- **Multiple Output Formats**: Both custom and legacy message formats

## Dependencies

### System Dependencies
```bash
# Install ROS 2 (Ubuntu 22.04)
sudo apt install ros-humble-desktop

# Install RealSense SDK
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# Install Python dependencies
pip install apriltag opencv-python numpy scipy
```

### ROS Dependencies
- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `vision_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`
- `cv_bridge`

## Installation

1. **Clone the package**:
```bash
cd ~/ros2_ws/src
git clone <repository-url> april_tags_vision
```

2. **Build the package**:
```bash
cd ~/ros2_ws
colcon build --packages-select april_tags_vision
source install/setup.bash
```

3. **Install Python dependencies**:
```bash
pip install apriltag opencv-python numpy scipy
```

## Usage

### Basic Detection

Launch the April Tags detector with RealSense camera:

```bash
ros2 launch april_tags_vision april_tags_detector.launch.py
```

### With Robot Integration

Launch with robot coordinate transformation:

```bash
ros2 launch april_tags_vision april_tags_with_robot.launch.py
```

### Custom Parameters

Launch with custom parameters:

```bash
ros2 launch april_tags_vision april_tags_detector.launch.py \
    tag_size_mm:=100.0 \
    confidence_threshold:=0.9 \
    show_debug_image:=true
```

### Test the Package

Run the test node to monitor detections:

```bash
ros2 run april_tags_vision test_april_tags.py
```

## Topics

### Published Topics

- `/april_tags/detections` (`april_tags_vision/msg/AprilTagDetectionArray`)
  - Main detection results with full 3D pose information

- `/april_tags/debug_image` (`sensor_msgs/msg/Image`)
  - Debug image with drawn detections (if enabled)

- `/detected_objects` (`std_msgs/msg/String`)
  - Legacy format for compatibility with existing systems

### Subscribed Topics

- `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
  - Input color images from camera

- `/camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`)
  - Camera calibration information

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tag_size_mm` | float | 50.0 | Physical size of April Tags in millimeters |
| `confidence_threshold` | float | 0.8 | Minimum confidence for detection (0.0-1.0) |
| `camera_frame_id` | string | "camera_link" | Camera frame ID |
| `robot_frame_id` | string | "base_link" | Robot base frame ID |
| `publish_rate` | float | 10.0 | Detection publish rate in Hz |
| `enable_3d_pose` | bool | true | Enable 3D pose estimation |
| `enable_robot_coords` | bool | true | Enable robot coordinate transformation |
| `show_debug_image` | bool | false | Show debug image with detections |
| `robot_ip` | string | "192.168.1.241" | Robot IP address |

## Messages

### AprilTagDetection

```yaml
std_msgs/Header header
int32 tag_id
string tag_family
float32 tag_size_mm
float32 confidence
float32 decision_margin
int32 hamming
float32 goodness
geometry_msgs/Point2D center
geometry_msgs/Point2D[] corners
geometry_msgs/Pose pose_3d
float64 distance
bool pose_valid
geometry_msgs/Point position_robot
geometry_msgs/Quaternion orientation_robot
bool robot_coords_valid
```

### AprilTagDetectionArray

```yaml
std_msgs/Header header
AprilTagDetection[] detections
int32 num_detections
float32 processing_time_ms
string camera_frame_id
string robot_frame_id
```

## Configuration

Create a configuration file `config/april_tags.yaml`:

```yaml
april_tags_detector:
  ros__parameters:
    tag_size_mm: 50.0
    confidence_threshold: 0.8
    camera_frame_id: "camera_link"
    robot_frame_id: "base_link"
    publish_rate: 10.0
    enable_3d_pose: true
    enable_robot_coords: true
    show_debug_image: false
    robot_ip: "192.168.1.241"
```

## Troubleshooting

### Common Issues

1. **No detections**: Check camera connection and April Tag visibility
2. **Poor detection**: Adjust `confidence_threshold` parameter
3. **Wrong coordinates**: Verify camera calibration and robot transformation
4. **High CPU usage**: Reduce `publish_rate` parameter

### Debug Mode

Enable debug image to visualize detections:

```bash
ros2 launch april_tags_vision april_tags_detector.launch.py show_debug_image:=true
```

### Logging

Enable verbose logging:

```bash
ros2 run april_tags_vision april_tags_detector_node.py --ros-args --log-level debug
```

## Examples

### Python Example

```python
import rclpy
from rclpy.node import Node
from april_tags_vision.msg import AprilTagDetectionArray

class AprilTagsSubscriber(Node):
    def __init__(self):
        super().__init__('april_tags_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/april_tags/detections',
            self.detection_callback,
            10
        )
    
    def detection_callback(self, msg):
        for detection in msg.detections:
            print(f"Tag ID: {detection.tag_id}")
            print(f"Confidence: {detection.confidence}")
            if detection.pose_valid:
                print(f"3D Position: {detection.pose_3d.position}")

def main():
    rclpy.init()
    node = AprilTagsSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## License

MIT License - see LICENSE file for details.
