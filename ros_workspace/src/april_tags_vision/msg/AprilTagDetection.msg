# April Tag Detection Message
# Represents a single April Tag detection

# Header with timestamp and frame information
std_msgs/Header header

# Tag identification
int32 tag_id                    # April Tag ID number
string tag_family               # Tag family (e.g., "tagStandard41h12")
float32 tag_size_mm             # Physical size of the tag in millimeters

# Detection properties
float32 confidence              # Detection confidence (0.0-1.0)
float32 decision_margin         # Decision margin from April Tags library
int32 hamming                   # Hamming distance
float32 goodness                # Goodness metric

# 2D image coordinates
geometry_msgs/Point2D center    # Center of the tag in image coordinates
geometry_msgs/Point2D[] corners # Four corners of the tag in image coordinates

# 3D pose information (if available)
geometry_msgs/Pose pose_3d      # 3D pose of the tag in camera frame
float64 distance                # Distance from camera in meters
bool pose_valid                 # Whether 3D pose estimation was successful

# Robot coordinates (if available)
geometry_msgs/Point position_robot  # Position in robot base frame (mm)
geometry_msgs/Quaternion orientation_robot  # Orientation in robot base frame
bool robot_coords_valid         # Whether robot coordinates are available
