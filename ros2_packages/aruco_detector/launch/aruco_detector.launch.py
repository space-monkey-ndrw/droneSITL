from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_detector',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': False},
            ],
        ),
    ])
