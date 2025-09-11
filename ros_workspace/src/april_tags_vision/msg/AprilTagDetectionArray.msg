# April Tag Detection Array Message
# Contains multiple April Tag detections

# Header with timestamp and frame information
std_msgs/Header header

# Array of April Tag detections
AprilTagDetection[] detections

# Detection statistics
int32 num_detections            # Number of tags detected
float32 processing_time_ms      # Time taken to process the image (ms)
string camera_frame_id          # Camera frame ID
string robot_frame_id           # Robot base frame ID
