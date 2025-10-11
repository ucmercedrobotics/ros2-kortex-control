#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="sam_b.pt",
                description="Path to SAM model file",
            ),
            DeclareLaunchArgument(
                "confidence_threshold",
                default_value="0.5",
                description="Confidence threshold for detections",
            ),
            DeclareLaunchArgument(
                "image_path",
                default_value="/tmp/bus.jpg",
                description="Path to test image",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="1.0",
                description="Rate to publish images (Hz)",
            ),
            # SAM Node
            Node(
                package="kortex_vision",
                executable="pistachio_leaf_segmentation",
                name="pistachio_leaf_segmentation",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "confidence_threshold": LaunchConfiguration(
                            "confidence_threshold"
                        ),
                    }
                ],
                output="screen",
            ),
            # Image Publisher Node
            Node(
                package="kortex_vision",
                executable="image_publisher",
                name="image_publisher",
                parameters=[
                    {
                        "image_path": LaunchConfiguration("image_path"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                    }
                ],
                output="screen",
            ),
        ]
    )
