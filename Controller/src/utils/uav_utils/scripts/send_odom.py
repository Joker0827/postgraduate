#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdometrySender(Node):
    def __init__(self):
        super().__init__("odom_sender")
        self.publisher = self.create_publisher(Odometry, "odom", 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.message = Odometry()
        self.message.header.frame_id = "world"
        self.message.pose.pose.orientation.w = 1.0

    def publish(self):
        self.message.header.stamp = (self.get_clock().now() - Duration(seconds=0.2)).to_msg()
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = OdometrySender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
