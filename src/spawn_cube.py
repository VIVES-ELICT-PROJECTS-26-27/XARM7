#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class SpawnCube(Node):
    def __init__(self):
        super().__init__("spawn_cube")

        self.publisher = self.create_publisher(
            Marker,
            "/visualization_marker",
            10
        )

        self.timer = self.create_timer(1.0, self.publish_cube)

    def publish_cube(self):
        marker = Marker()

        marker.header.frame_id = "link_base"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "objects"
        marker.id = 0

        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position relative to robot base
        marker.pose.position.x = 0.4
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0

        # Dimensions (meters)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Red, opaque
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.publisher.publish(marker)


def main():
    rclpy.init()
    node = SpawnCube()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()