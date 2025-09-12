#!/usr/bin/env python3
"""
Direct Robot Approach - No Safety, No Complexity
================================================

Simple, direct approach function that just moves the robot to detected coordinates.
No safety checks, no transformations, no verification - just pure movement.

Usage:
    from robot_control.robot_controller.direct_approach import direct_approach
    direct_approach(robot_arm, [x, y, z])
"""

def direct_approach(robot_arm, coordinates, speed=300):
    """
    Move robot directly to coordinates - no safety, no checks, just move.
    
    Args:
        robot_arm: XArmAPI instance (already connected)
        coordinates: [x, y, z] in mm
        speed: Movement speed in mm/s (default 300)
    
    Returns:
        bool: True if command sent successfully, False otherwise
    """
    try:
        if robot_arm is None:
            print("[DIRECT] No robot connection")
            return False
            
        if len(coordinates) < 3:
            print("[DIRECT] Invalid coordinates")
            return False
            
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        
        print(f"[DIRECT] Moving to: X={x:.1f}, Y={y:.1f}, Z={z:.1f}mm at {speed}mm/s")
        
        # Direct robot movement - no safety, no checks
        result = robot_arm.set_position(
            x=x, y=y, z=z,
            roll=0, pitch=90, yaw=0,  # Fixed orientation
            speed=speed, 
            wait=True
        )
        
        if result == 0:
            print(f"[DIRECT] SUCCESS: Robot moved to [{x:.1f}, {y:.1f}, {z:.1f}]")
            return True
        else:
            print(f"[DIRECT] FAILED: Movement failed with code {result}")
            return False
            
    except Exception as e:
        print(f"[DIRECT] ERROR: {e}")
        return False


def approach_detected_object(robot_arm, object_index, object_name, speed=300):
    """
    Approach a detected object directly - no safety, just move.
    
    Args:
        robot_arm: XArmAPI instance (already connected)
        object_index: ObjectIndex instance with latest coordinates
        object_name: Name of object to approach (e.g., "bottle")
        speed: Movement speed in mm/s (default 300)
        
    Returns:
        bool: True if approach successful, False otherwise
    """
    try:
        # Get latest coordinates from object index
        with object_index._global_lock:
            coordinates = object_index.latest_mm.get(object_name)
            
        if coordinates is None:
            print(f"[DIRECT] Object '{object_name}' not detected")
            return False
            
        print(f"[DIRECT] Found {object_name} at: {coordinates}")
        
        # Move directly to object coordinates
        return direct_approach(robot_arm, coordinates, speed)
        
    except Exception as e:
        print(f"[DIRECT] ERROR approaching {object_name}: {e}")
        return False
