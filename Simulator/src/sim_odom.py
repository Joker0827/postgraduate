#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher = self.create_publisher(Odometry, '/sim/odom', qos_profile_sensor_data)
        self.velocity = self.declare_parameter('velocity', 2.0).value
        self.message = Odometry()
        self.message.header.frame_id = 'world'
        self.message.child_frame_id = 'quadrotor'
        self.message.pose.pose.position.y = 2.0
        self.message.pose.pose.position.z = 1.6
        self.message.pose.pose.orientation.w = 1.0
        self.message.twist.twist.linear.x = self.velocity
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / 30.0, self.publish_odometry)

    def publish_odometry(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds * 1e-9
        self.message.header.stamp = current_time.to_msg()
        self.message.pose.pose.position.x = self.velocity * elapsed_time
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
