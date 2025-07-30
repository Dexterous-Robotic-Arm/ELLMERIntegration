# run_motion_bridge.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3',
                 '-m', 'kortex_examples.ELLMER.motion_bridge'],
            output='screen'
        ),
    ])
