#!/usr/bin/env python3
"""

ros2 launch nanospec nanospec.launch.py
ros2 launch nanospec nanospec.launch.py serial_port:=/dev/ttyUSB0

ros2 action send_goal /acquire_spectrum kortex_interfaces/action/AcquireSpectrum "{}"

ros2 action send_goal /acquire_spectrum kortex_interfaces/action/AcquireSpectrum \
    "{integration_time: 1, frame_avg_num: 20, enable_auto_exposure: true}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/nanospec',
        description='Serial port for the NSP32 spectrometer'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    publish_on_acquire_arg = DeclareLaunchArgument(
        'publish_on_acquire',
        default_value='true',
        description='Publish spectrum data to /spectral_data topic after each acquisition'
    )

    nanospec_node = Node(
        package='nanospec',
        executable='nanospec_action_server',
        name='nanospec_action_server',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'publish_on_acquire': LaunchConfiguration('publish_on_acquire'),
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        publish_on_acquire_arg,
        nanospec_node,
    ])

