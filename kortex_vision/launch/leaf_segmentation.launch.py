#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            # YOLO Node
            Node(
                package="kortex_vision",
                executable="pistachio_leaf_segmentation",
                name="pistachio_leaf_segmentation",
                output="screen",
            ),
        ]
    )
