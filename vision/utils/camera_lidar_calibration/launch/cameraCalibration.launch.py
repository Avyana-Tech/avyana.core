from launch import LaunchDescription
from launch_ros.actions import Node

#=====================================
# Calibrate camera with a chess board.
#=====================================
def generate_launch_description():
    return LaunchDescription([
        # ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.015 --ros-args --remap image:=/image_raw
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            parameters=[{'size': '7x9',
                         'square':0.015}],
            remappings=[('image','/image_raw')],
            output='screen',
        )
    ])  