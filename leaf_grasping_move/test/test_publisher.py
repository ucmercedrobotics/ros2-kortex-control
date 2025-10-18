#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory

import os
import json
import pandas as pd
import sys

from kortex_interfaces.msg import LeafPoseArrays
from geometry_msgs.msg import Pose
from std_msgs.msg import String


class TargetPublisher(Node):
    def __init__(self):
        super().__init__("target_publisher_node")
        package_name = "leaf_grasping_move"
        package_share_directory = get_package_share_directory(package_name)
        default_json_path = os.path.join(
            package_share_directory, "resources", "poses_combined.json"
        )

        self.declare_parameter("json_file_path", default_json_path)
        json_path = (
            self.get_parameter("json_file_path").get_parameter_value().string_value
        )
        self.json_path = os.path.expanduser(json_path)

        self.all_targets = self._load_targets_from_file()
        if not self.all_targets:
            self.get_logger().error("Shutting down due to no targets being loaded.")
            self.destroy_node()
            rclpy.shutdown()
            return

        self.current_target_index = 0

        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.publisher_ = self.create_publisher(
            LeafPoseArrays, "/multi_target_poses", qos_profile
        )

        self.status_subscriber_ = self.create_subscription(
            String, "/arm_task_status", self._status_callback, 10
        )

        self.get_logger().info(
            f" Target Publisher is ready. Loaded {len(self.all_targets)} targets."
        )

        # This timer will repeatedly check if the TargetManager is listening.
        self.connection_check_timer_ = self.create_timer(
            0.1, self._wait_for_subscriber_and_publish  # Check every 100ms
        )
        self.get_logger().info("Waiting for TargetManager to connect...")

    def _wait_for_subscriber_and_publish(self):

        # get_subscription_count() checks how many nodes are listening.
        if self.publisher_.get_subscription_count() > 0:
            self.get_logger().info(
                "TargetManager connected! Publishing the first target..."
            )

            # Publish the first message
            self._publish_next_target()

            self.connection_check_timer_.cancel()

    def _load_targets_from_file(self):

        if not os.path.exists(self.json_path):
            self.get_logger().error(f"JSON file not found at: {self.json_path}")
            return []

        try:
            with open(self.json_path, "r") as f:
                data = json.load(f)
            self.get_logger().info(f"Successfully loaded data from {self.json_path}")
            return data
        except Exception as e:
            self.get_logger().error(f"Failed to read or parse JSON file: {e}")
            return []

    def _status_callback(self, msg):

        if msg.data == "COMPLETE":
            self.get_logger().info("-----------------------------------------")
            self.get_logger().info(
                "Received 'COMPLETE' status. Publishing next target..."
            )
            self._publish_next_target()

    def _create_pose_array(self, raw_poses_list):

        pose_msgs = []
        if not isinstance(raw_poses_list, list):
            return pose_msgs

        for pose_data in raw_poses_list:
            if len(pose_data) == 7:
                msg = Pose()
                msg.position.x = float(pose_data[0])
                msg.position.y = float(pose_data[1])
                msg.position.z = float(pose_data[2])
                msg.orientation.x = float(pose_data[3])
                msg.orientation.y = float(pose_data[4])
                msg.orientation.z = float(pose_data[5])
                msg.orientation.w = float(pose_data[6])
                pose_msgs.append(msg)
        return pose_msgs

    def _publish_next_target(self):

        if self.current_target_index >= len(self.all_targets):
            self.get_logger().info("All targets have been published. Mission complete!")
            return

        target_data = self.all_targets[self.current_target_index]
        source_file = target_data.get("source_file", "unknown")

        self.get_logger().info(
            f"Publishing all targets in scan {self.current_target_index + 1}/{len(self.all_targets)} "
            f"(from source: {source_file})"
        )

        msg = LeafPoseArrays()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.poses1 = self._create_pose_array(target_data.get("Poses1"))
        msg.poses2 = self._create_pose_array(target_data.get("Poses2"))
        msg.poses3 = self._create_pose_array(target_data.get("Poses3"))
        msg.poses4 = self._create_pose_array(target_data.get("Poses4"))
        msg.poses5 = self._create_pose_array(target_data.get("Poses5"))

        self.publisher_.publish(msg)
        self.current_target_index += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        target_publisher = TargetPublisher()
        if rclpy.ok():
            rclpy.spin(target_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if "target_publisher" in locals() and rclpy.ok():
            target_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
