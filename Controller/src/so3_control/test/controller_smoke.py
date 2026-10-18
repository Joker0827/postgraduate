#!/usr/bin/env python3
"""Run actual ROS 2 controller nodes against command and service contracts."""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
from collections import deque

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from quadrotor_msgs.msg import PositionCommand, SO3Command
from quadrotor_msgs.srv import SetTakeoffLand


class ControllerHarness:
    def __init__(self, executable, parameters):
        self.namespace = f'/controller_smoke_{os.getpid()}_{executable}'
        parameters = dict(parameters)
        if executable == 'network_control_node':
            parameters.setdefault('mavros_namespace', self.namespace + '/mavros')
        self.log = tempfile.TemporaryFile(mode='w+')
        command = [str(Path(get_package_prefix('so3_control')) / 'lib' / 'so3_control' / executable),
                   '--ros-args', '-r', f'__ns:={self.namespace}']
        for name, value in parameters.items():
            command.extend(['-p', f'{name}:={value}'])
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        self.node = rclpy.create_node(f'controller_harness_{executable}')
        self.odom = Odometry()
        self.odom.pose.pose.orientation.w = 1.0
        self.odom_enabled = False
        self.messages = deque(maxlen=500)
        self.received = 0
        self.odom_pub = self.node.create_publisher(
            Odometry, self.namespace + '/odom', qos_profile_sensor_data)
        self.imu_pub = self.node.create_publisher(
            Imu, self.namespace + '/imu', qos_profile_sensor_data)
        self.command_pub = self.node.create_publisher(
            PositionCommand, self.namespace + '/position_cmd', 1)
        self.subscription = self.node.create_subscription(
            SO3Command, self.namespace + '/so3_cmd', self.receive, 10)
        self.timer = self.node.create_timer(0.02, self.publish_sensors)
        self.service = self.node.create_client(
            SetTakeoffLand, self.namespace + '/network_controller_node/takeoff_land')

    def receive(self, message):
        self.messages.append(message)
        self.received += 1

    def publish_sensors(self):
        if self.odom_enabled:
            self.odom.header.stamp = self.node.get_clock().now().to_msg()
            self.odom_pub.publish(self.odom)
            self.imu_pub.publish(Imu())

    def pump(self, seconds, predicate=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.log.seek(0)
                raise AssertionError(f'Controller exited: {self.log.read()}')
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if predicate is not None and predicate():
                return True
        return predicate is None or predicate()

    def flight_request(self, takeoff, altitude=0.2):
        if not self.service.wait_for_service(timeout_sec=5.0):
            self.log.seek(0)
            raise AssertionError(f'Takeoff service unavailable: {self.log.read()}')
        request = SetTakeoffLand.Request()
        request.takeoff = takeoff
        request.takeoff_altitude = altitude
        future = self.service.call_async(request)
        if not self.pump(3.0, future.done):
            raise AssertionError('Takeoff service did not return within 3 seconds')
        return future.result().res

    def close(self):
        self.node.destroy_node()
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5.0)
        self.log.close()


class ControllerContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault('ROS_DOMAIN_ID', '89')
        os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_network_takeoff_acceleration_and_landing(self):
        harness = ControllerHarness('network_control_node', {
            'is_simulation': 'true', 'auto_takeoff': 'false',
            'use_disturbance_observer': 'false',
        })
        try:
            self.assertFalse(harness.flight_request(True))
            harness.odom_enabled = True
            harness.pump(0.4)
            self.assertFalse(harness.flight_request(True, float('nan')))
            self.assertTrue(harness.flight_request(True))
            self.assertFalse(harness.flight_request(True))
            self.assertTrue(harness.pump(3.0, lambda: bool(harness.messages)))
            harness.pump(2.0)
            command = PositionCommand()
            command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            command.acceleration.x = 0.5
            command.acceleration.y = 0.2
            command.acceleration.z = 0.1
            command.yaw = 0.4
            harness.command_pub.publish(command)
            self.assertTrue(harness.pump(2.0, lambda: any(
                abs(msg.force.x - 0.98 * 0.5) < 1e-5 and
                abs(msg.force.y - 0.98 * 0.2) < 1e-5 and
                abs(msg.force.z - 0.98 * (9.81 + 0.1)) < 1e-5
                for msg in harness.messages)), 'Network acceleration must map to m * (a + g)')
            output = harness.messages[-1]
            norm = math.sqrt(sum(value ** 2 for value in (
                output.orientation.x, output.orientation.y,
                output.orientation.z, output.orientation.w)))
            self.assertAlmostEqual(norm, 1.0, places=6)
            self.assertAlmostEqual(output.k_r[0], 1.5, places=6)
            self.assertTrue(harness.flight_request(False))
            harness.pump(0.3)
            count = harness.received
            harness.pump(0.3)
            self.assertEqual(harness.received, count, 'Commands must stop after simulated disarm')
        finally:
            harness.close()

    def test_pid_hover_and_default_position_gain(self):
        harness = ControllerHarness('so3_control_node', {'so3_control.init_state_z': '1.0'})
        try:
            harness.odom.pose.pose.position.z = 1.0
            harness.odom_enabled = True
            self.assertTrue(harness.pump(5.0, lambda: bool(harness.messages)))
            self.assertAlmostEqual(harness.messages[-1].force.z, 0.98 * 9.81, places=5)
            command = PositionCommand()
            command.position.x = 0.1
            command.position.z = 1.0
            harness.command_pub.publish(command)
            self.assertTrue(harness.pump(2.0, lambda: any(
                abs(msg.force.x - 5.7 * 0.1) < 1e-5 for msg in harness.messages)),
                'PID position command must preserve the original gain')
            self.assertAlmostEqual(harness.messages[-1].force.z, 0.98 * 9.81, places=5)
        finally:
            harness.close()

    def test_mavros_mode_retry_arm_and_disarm(self):
        harness = ControllerHarness('network_control_node', {
            'is_simulation': 'false', 'auto_takeoff': 'false',
        })
        try:
            state = State()
            state.connected = True
            state.mode = 'MANUAL'
            mode_requests = []
            arm_requests = []
            publisher = harness.node.create_publisher(
                State, harness.namespace + '/mavros/state', qos_profile_sensor_data)

            def set_mode(request, response):
                mode_requests.append(request.custom_mode)
                response.mode_sent = len(mode_requests) > 1
                if response.mode_sent:
                    state.mode = request.custom_mode
                return response

            def set_arm(request, response):
                arm_requests.append(request.value)
                state.armed = request.value
                response.success = True
                return response

            mode_service = harness.node.create_service(
                SetMode, harness.namespace + '/mavros/set_mode', set_mode)
            arm_service = harness.node.create_service(
                CommandBool, harness.namespace + '/mavros/cmd/arming', set_arm)
            state_timer = harness.node.create_timer(0.02, lambda: publisher.publish(state))
            harness.odom.pose.pose.position.z = 0.5
            harness.odom_enabled = True
            harness.pump(0.4)
            self.assertTrue(harness.flight_request(True, 0.6))
            self.assertTrue(harness.pump(6.0, lambda: True in arm_requests))
            self.assertEqual(mode_requests, ['OFFBOARD', 'OFFBOARD'])
            self.assertGreater(harness.received, 20, 'Setpoints must continue during mode retry')
            harness.pump(1.6)
            self.assertTrue(harness.flight_request(False))
            harness.odom.pose.pose.position.z = 0.0
            self.assertTrue(harness.pump(3.0, lambda: False in arm_requests))
            harness.pump(0.3)
            count = harness.received
            harness.pump(0.3)
            self.assertEqual(harness.received, count)
        finally:
            harness.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
