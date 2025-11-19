from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='serial_stream',
            executable='serial_stream_node',
            name='serial_stream_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 115200,
                'data_count': 3,
                'data0_format': 'time: %lf',
                'data1_format': 'volt: %lf',
                'data2_format': 'temp: %lf',
            }],
            remappings=[
                ('~/data0', 'time'),
                ('~/data1', 'voltage'),
                ('~/data2', 'tempurature'),
            ],
        ),
    ])
