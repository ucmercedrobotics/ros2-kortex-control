#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "input_image", 10)

        # Get image path from parameter
        self.image_path = self.declare_parameter("image_path", "").value
        self.publish_rate = self.declare_parameter("publish_rate", 1.0).value

        if not self.image_path or not os.path.exists(self.image_path):
            self.get_logger().error(f"Image path not found: {self.image_path}")
            return

        # Load image
        self.image = cv2.imread(self.image_path)
        if self.image is None:
            self.get_logger().error(f"Failed to load image: {self.image_path}")
            return

        self.get_logger().info(f"Loaded image: {self.image_path}")

        # Create timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_image)

    def publish_image(self):
        try:
            msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.image_pub.publish(msg)
            self.get_logger().info("Published image")
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
