#!/usr/bin/env python3
import copy
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class OdometryConverter:
    def __init__(self, node, index):
        self.node = node
        prefix = f"converter{index}."
        self.frame_in = node.declare_parameter(prefix + "frame_id_in", "world").value
        self.frame_out = node.declare_parameter(prefix + "frame_id_out", "world").value
        self.broadcast_tf = node.declare_parameter(prefix + "broadcast_tf", False).value
        self.body_frame = node.declare_parameter(prefix + "body_frame_id", "body").value
        self.intermediate_frame = node.declare_parameter(prefix + "intermediate_frame_id", "intermediate").value
        self.world_frame = node.declare_parameter(prefix + "world_frame_id", "world").value
        topic_prefix = f"~/converter{index}/"
        self.odom_pub = node.create_publisher(Odometry, topic_prefix + "out_odom", 10)
        self.path_pub = node.create_publisher(Path, topic_prefix + "out_path", 10)
        self.subscription = node.create_subscription(
            Odometry, topic_prefix + "in_odom", self.callback, qos_profile_sensor_data
        )
        self.path = deque(maxlen=30000)
        self.timer = node.create_timer(0.5, self.publish_path)
        if self.broadcast_tf:
            static_frames = set((self.frame_in, self.world_frame)) - {self.frame_out}
            transforms = [self.transform(self.frame_out, child) for child in sorted(static_frames)]
            if transforms:
                node.static_broadcaster.sendTransform(transforms)

    def transform(self, parent, child):
        msg = TransformStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.rotation.w = 1.0
        return msg

    def callback(self, input_msg):
        if input_msg.header.frame_id != self.frame_in:
            self.node.get_logger().warning(
                f"Expected odometry frame {self.frame_in}, received {input_msg.header.frame_id}"
            )
            return
        msg = copy.deepcopy(input_msg)
        msg.header.frame_id = self.frame_out
        msg.child_frame_id = ""
        self.odom_pub.publish(msg)
        if self.broadcast_tf:
            body = self.transform(self.world_frame, self.body_frame)
            body.header.stamp = msg.header.stamp
            body.transform.translation.x = msg.pose.pose.position.x
            body.transform.translation.y = msg.pose.pose.position.y
            body.transform.translation.z = msg.pose.pose.position.z
            body.transform.rotation = msg.pose.pose.orientation
            intermediate = copy.deepcopy(body)
            intermediate.child_frame_id = self.intermediate_frame
            q = msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            intermediate.transform.rotation.x = 0.0
            intermediate.transform.rotation.y = 0.0
            intermediate.transform.rotation.z = math.sin(yaw/2)
            intermediate.transform.rotation.w = math.cos(yaw/2)
            self.node.broadcaster.sendTransform([body, intermediate])
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.append(pose)

    def publish_path(self):
        if self.path:
            msg = Path()
            msg.header = self.path[-1].header
            msg.poses = list(self.path)
            self.path_pub.publish(msg)


class TransformAssist(Node):
    def __init__(self):
        super().__init__("tf_assist")
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        count = self.declare_parameter("converter_count", 1).value
        self.converters = [OdometryConverter(self, index) for index in range(count)]


def main():
    rclpy.init()
    node = TransformAssist()
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
