from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        # RealSense 카메라
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join('/root/ros2_ws/install/realsense2_camera/share/realsense2_camera/launch', 'rs_launch.py')
            ])
        ),
        
        # S3 LiDAR
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 1000000,
                'frame_id': 'laser_frame'
            }]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/root/ros2_ws/install/sllidar_ros2/share/sllidar_ros2/rviz/sllidar_ros2.rviz']
        )
    ])
