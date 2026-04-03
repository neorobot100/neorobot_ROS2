from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_node',
            output='screen',
            parameters=[{
                'port': '/dev/ydlidar',
                'frame_id': 'laser_frame',
                'baudrate': 230400,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'range_min': 0.12,
                'range_max': 16.0,
                'invalid_range_is_inf': False
            }]
        )
    ])
