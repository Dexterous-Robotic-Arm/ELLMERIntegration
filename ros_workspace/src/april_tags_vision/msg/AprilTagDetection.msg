# April Tag Detection Message
# Single April Tag detection result

int32 tag_id                    # April Tag ID
string object_name              # Mapped object name (e.g., "bottle", "book")
geometry_msgs/Point center      # Center point in pixels
float64 confidence              # Detection confidence (0.0-1.0)
string tag_family               # Tag family (e.g., "tagStandard41h12")

# 3D pose information
geometry_msgs/Point position_3d # 3D position in robot base frame (mm)
geometry_msgs/Quaternion orientation_3d  # 3D orientation
bool has_3d_pose                # Whether 3D pose is available

# Additional information
float64 tag_size_mm             # Physical tag size in millimeters
string frame_id                 # Reference frame ID