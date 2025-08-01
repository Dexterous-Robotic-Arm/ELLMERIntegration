import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess

def generate_launch_description():
    # Directly embed it here:
    return LaunchDescription([
        SetEnvironmentVariable(name='GOOGLE_API_KEY', value='AIzaSyBVTtIECu5DbGyONas6MlcPyWEIzRPStMg'),
        ExecuteProcess(
            cmd=['python3','-m','kortex_examples.ELLMER.robot_brain'],
            output='screen'
        ),
    ])