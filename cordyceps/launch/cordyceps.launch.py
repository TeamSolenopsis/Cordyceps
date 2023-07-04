from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cordyceps',
            executable='manager',
            name='manager',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'number_of_robots': 3},
                {'diameter': 1.0},
                {'path_filename': 'squareDiffdrive'},
            ]
        )
    ])
