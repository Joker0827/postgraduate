#!/usr/bin/env python3
"""Observe a running ROS 2 simulation and exercise a new navigation goal."""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, SO3Command
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def vector(v):
    return [v.x, v.y, v.z]


class ClosedLoopCheck(Node):
    def __init__(self):
        super().__init__('yopo_closed_loop_check')
        self.counts = {name: 0 for name in ('odom', 'depth', 'command', 'so3')}
        self.timestamps = {name: [] for name in self.counts}
        self.errors = []
        self.position = None
        self.min_height = math.inf
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 1)
        self.subscriptions_live = [
            self.create_subscription(Odometry, '/sim/odom', self.odom, qos_profile_sensor_data),
            self.create_subscription(Image, '/depth_image', self.depth, qos_profile_sensor_data),
            self.create_subscription(PositionCommand, '/so3_control/pos_cmd', self.command, 10),
            self.create_subscription(SO3Command, '/so3_cmd', self.so3, 10),
        ]

    def record(self, name, values=()):
        self.counts[name] += 1
        self.timestamps[name].append(time.monotonic())
        if not all(math.isfinite(value) for value in values):
            self.errors.append(f'Nonfinite {name} values: {values}')

    def odom(self, msg):
        self.position = vector(msg.pose.pose.position)
        self.min_height = min(self.min_height, self.position[2])
        self.record('odom', self.position + vector(msg.twist.twist.linear))

    def depth(self, msg):
        self.record('depth')
        if msg.encoding != '32FC1' or len(msg.data) != msg.height * msg.step:
            self.errors.append('Invalid simulator depth payload')

    def command(self, msg):
        self.record('command', vector(msg.position) + vector(msg.velocity) +
                    vector(msg.acceleration) + [msg.yaw, msg.yaw_dot])
        if msg.header.frame_id != 'world':
            self.errors.append('Planner command is missing world frame')

    def so3(self, msg):
        self.record('so3', vector(msg.force) +
                    [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--startup-timeout', type=float, default=20.0)
    args = parser.parse_args()
    rclpy.init()
    node = ClosedLoopCheck()
    try:
        deadline = time.monotonic() + args.startup_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.position is not None and all(node.counts.values()):
                break
        else:
            raise RuntimeError(f'Missing ROS 2 streams at startup: {node.counts}')

        start = node.position.copy()
        node.min_height = start[2]
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = start[0] + 20.0
        goal.pose.position.y = start[1] + 3.0
        goal.pose.position.z = 2.0
        goal.pose.orientation.w = 1.0
        match_deadline = time.monotonic() + 5.0
        while node.goal_pub.get_subscription_count() == 0 and time.monotonic() < match_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.goal_pub.get_subscription_count() == 0:
            node.errors.append('Planner goal subscription was not discovered')
        node.goal_pub.publish(goal)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

        rates = {}
        for name, samples in node.timestamps.items():
            rates[name] = round((len(samples) - 1) / (samples[-1] - samples[0]), 2)
        travel = math.dist(start, node.position)
        for name, minimum in [('odom', 100), ('depth', 10), ('command', 20), ('so3', 20)]:
            if node.counts[name] < minimum:
                node.errors.append(f'Too few {name} messages: {node.counts[name]}')
        if travel < 3.0:
            node.errors.append(f'Insufficient motion after new goal: {travel:.2f} m')
        if node.min_height < 0.5:
            node.errors.append(f'Quadrotor fell below 0.5 m: {node.min_height:.2f}')
        result = {'counts': node.counts, 'rates_hz': rates, 'start': start,
                  'end': node.position, 'travel_m': round(travel, 2), 'errors': node.errors[:10]}
        print(json.dumps(result, indent=2))
        if node.errors:
            raise RuntimeError('Closed-loop verification failed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
