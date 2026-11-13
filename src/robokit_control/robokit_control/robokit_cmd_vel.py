#!/usr/bin/env python3
"""ROS 2 Humble Twist -> Robokit API 2010 bridge."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from .robokit_tcp import RobokitClient
from .limits import DEFAULT_HOST, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED, limit_velocity, validate_limits


class RobokitBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("robokit_cmd_vel", **kwargs)
        defaults = {
            "hostname": DEFAULT_HOST, "robot_ip": "", "port": 19205, "protocol_version": 1,
            "rate_hz": 20.0, "command_timeout": 0.25,
            "socket_timeout": 0.2, "duration_ms": 300,
            "max_linear_speed": MAX_LINEAR_SPEED, "max_angular_speed": MAX_ANGULAR_SPEED,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        config = {name: self.get_parameter(name).value for name in defaults}
        for name in ("rate_hz", "command_timeout", "socket_timeout",
                     "max_linear_speed", "max_angular_speed"):
            if not math.isfinite(config[name]) or config[name] <= 0:
                raise ValueError(name + " must be finite and positive")
        self.duration_ms = config["duration_ms"]
        if not 1 <= self.duration_ms <= 5000:
            raise ValueError("duration_ms must be in 1..5000")
        if 1000.0 / config["rate_hz"] >= self.duration_ms:
            raise ValueError("Refresh period must be shorter than duration_ms")
        self.command_timeout = config["command_timeout"]
        self.max_linear_speed = config["max_linear_speed"]
        self.max_angular_speed = config["max_angular_speed"]
        validate_limits(self.max_linear_speed, self.max_angular_speed)
        hostname = config["robot_ip"] or config["hostname"]
        self.client = RobokitClient(
            hostname, config["port"], config["protocol_version"], config["socket_timeout"],
            self.max_linear_speed, self.max_angular_speed,
        )
        self.velocity = (0.0, 0.0, 0.0)
        self.last_received = None
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.on_velocity, qos)
        self.timer = self.create_timer(1.0 / config["rate_hz"], self.send_velocity)
        self.get_logger().info("Bridge ready. Waiting for /cmd_vel. Robot-frame vx, vy, w.")

    def on_velocity(self, message):
        vx, vy, w = message.linear.x, message.linear.y, message.angular.z
        try:
            self.velocity = limit_velocity(
                vx, vy, w, self.max_linear_speed, self.max_angular_speed
            )
        except ValueError:
            self.velocity = (0.0, 0.0, 0.0)
            self.last_received = time.monotonic()
            self.get_logger().error("Invalid velocity received; commanding zero")
            return
        self.last_received = time.monotonic()

    def send_velocity(self):
        # No API traffic until a command arrives, so merely launching the bridge
        # does not cancel an existing navigation task.
        if self.last_received is None:
            return
        velocity = self.velocity
        if time.monotonic() - self.last_received > self.command_timeout:
            velocity = (0.0, 0.0, 0.0)
        try:
            self.client.motion(*velocity, duration_ms=self.duration_ms)
        except (OSError, RuntimeError, ValueError) as error:
            self.velocity = (0.0, 0.0, 0.0)
            self.last_received = None
            raise RuntimeError("Robokit communication failed; bridge exiting: " + str(error)) from error

    def stop_and_close(self):
        # Only issue a stop if this bridge actually opened a connection.
        # A failed request closes it, but has_sent records whether sending began.
        if self.client.sock is not None or self.client.has_sent:
            try:
                self.client.stop(self.duration_ms)
            except (OSError, RuntimeError, ValueError) as error:
                print("Stop delivery unconfirmed:", error, file=sys.stderr)
        self.client.close()


def main():
    rclpy.init()
    node = None
    exit_code = 0
    try:
        node = RobokitBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (OSError, RuntimeError, ValueError) as error:
        print(error, file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.stop_and_close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
