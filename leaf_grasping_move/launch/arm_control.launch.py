from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="leaf_grasping_move",
                executable="arm_commander",
                name="arm_commander",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    "arm_commander:=FATAL",
                    "--log-level",
                    "move_group:=FATAL",
                ],
            ),
            Node(
                package="leaf_grasping_move",
                executable="target_manager",
                name="target_manager",
                output="screen",
            ),
        ]
    )
