"""Select a NOKOV rigid body and control a holonomic chassis to a goal pose."""

from dataclasses import fields
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mocap4r2_msgs.msg import RigidBodies
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .control import (
    ControlConfig, PoseController, normalized_quaternion, rotate_vector,
    transform_pose, yaw_from_quaternion,
)


def quaternion_values(quaternion):
    return (quaternion.x, quaternion.y, quaternion.z, quaternion.w)


class MocapPoseController(Node):
    def __init__(self, **kwargs):
        super().__init__("mocap_pose_controller", **kwargs)
        defaults = {
            "rigid_body_name": "", "rigid_bodies_topic": "/rigid_bodies",
            "source_frame": "map", "control_frame": "mocap_world",
            "rate_hz": 30.0, "pose_timeout": 0.2,
            "future_stamp_tolerance": 0.05, "max_tilt_rad": 0.35,
            "world_rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "world_translation_xyz": [0.0, 0.0, 0.0],
            "body_to_base_rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "body_to_base_translation_xyz": [0.0, 0.0, 0.0],
        }
        defaults.update(vars(ControlConfig()))
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        config = {name: self.get_parameter(name).value for name in defaults}
        if not config["rigid_body_name"]:
            raise ValueError("Set rigid_body_name to the chassis rigid body's exact NOKOV name")
        for name in ("source_frame", "control_frame", "rigid_bodies_topic"):
            if not config[name]:
                raise ValueError(name + " must be nonempty")
        for name in ("rate_hz", "pose_timeout", "future_stamp_tolerance", "max_tilt_rad"):
            if not math.isfinite(config[name]) or config[name] <= 0:
                raise ValueError(name + " must be finite and positive")
        for name in ("world_translation_xyz", "body_to_base_translation_xyz"):
            value = config[name]
            if len(value) != 3 or not all(math.isfinite(item) for item in value):
                raise ValueError(name + " must have three finite values")
        self.config = config
        self.world_rotation = normalized_quaternion(config["world_rotation_xyzw"])
        self.body_rotation = normalized_quaternion(config["body_to_base_rotation_xyzw"])
        self.controller = PoseController(ControlConfig(**{
            field.name: config[field.name] for field in fields(ControlConfig)
        }))
        self.pose = None
        self.pose_received_at = None
        self.pose_stamp = None
        self.last_frame_number = None
        self.last_source_stamp = None
        self.last_tick = time.monotonic()
        self.control_started = False
        self.status = None
        latest_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pose_publisher = self.create_publisher(PoseStamped, "chassis/pose", 1)
        self.status_publisher = self.create_publisher(String, "chassis/status", status_qos)
        self.mocap_subscription = self.create_subscription(
            RigidBodies, config["rigid_bodies_topic"], self.on_mocap, latest_sensor
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped, "chassis/goal_pose", self.on_goal, 1
        )
        self.cancel_service = self.create_service(Trigger, "chassis/cancel", self.on_cancel)
        self.timer = self.create_timer(1.0 / config["rate_hz"], self.on_timer)
        self.set_status("waiting_for_pose")

    def set_status(self, status):
        if status != self.status:
            self.status = status
            self.status_publisher.publish(String(data=status))
            self.get_logger().info(status)

    def publish_velocity(self, velocity=(0.0, 0.0, 0.0)):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = velocity
        self.command_publisher.publish(message)

    def fault(self, reason):
        self.controller.cancel()
        self.pose_received_at = None
        if self.control_started:
            self.publish_velocity()
        self.set_status(reason)

    def stamp_is_fresh(self, stamp):
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        return -self.config["future_stamp_tolerance"] <= age <= self.config["pose_timeout"]

    def pose_is_fresh(self):
        return (
            self.pose_received_at is not None
            and time.monotonic() - self.pose_received_at <= self.config["pose_timeout"]
            and self.stamp_is_fresh(self.pose_stamp)
        )

    def on_mocap(self, message):
        if message.header.frame_id != self.config["source_frame"]:
            self.fault("source_frame_mismatch")
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if stamp <= 0 or not self.stamp_is_fresh(stamp):
            self.fault("stale_source_stamp")
            return
        if self.last_frame_number == message.frame_number:
            # Duplicate frames must not refresh the tracking watchdog.
            return
        if self.last_frame_number is not None and (
            message.frame_number < self.last_frame_number or stamp <= self.last_source_stamp
        ):
            self.last_frame_number = message.frame_number
            self.last_source_stamp = stamp
            self.fault("source_reset_or_reordered_frame")
            return
        self.last_frame_number = message.frame_number
        self.last_source_stamp = stamp
        bodies = [body for body in message.rigidbodies
                  if body.rigid_body_name == self.config["rigid_body_name"]]
        if len(bodies) != 1:
            self.fault("rigid_body_missing_or_ambiguous")
            return
        body = bodies[0]
        try:
            position, orientation = transform_pose(
                (body.pose.position.x, body.pose.position.y, body.pose.position.z),
                quaternion_values(body.pose.orientation), self.world_rotation,
                self.config["world_translation_xyz"],
                self.config["body_to_base_translation_xyz"], self.body_rotation,
            )
            up_vector = rotate_vector(orientation, (0.0, 0.0, 1.0))
            if up_vector[2] < math.cos(self.config["max_tilt_rad"]):
                raise ValueError("Base orientation is not aligned with the control XY plane")
            self.pose = (position[0], position[1], yaw_from_quaternion(orientation))
        except ValueError as error:
            self.fault("invalid_pose: " + str(error))
            return
        self.pose_received_at = time.monotonic()
        self.pose_stamp = stamp
        current = PoseStamped()
        current.header.stamp = message.header.stamp
        current.header.frame_id = self.config["control_frame"]
        current.pose.position.x, current.pose.position.y, current.pose.position.z = position
        (current.pose.orientation.x, current.pose.orientation.y,
         current.pose.orientation.z, current.pose.orientation.w) = orientation
        self.pose_publisher.publish(current)
        if not self.control_started:
            self.set_status("ready")

    def on_goal(self, message):
        try:
            if message.header.frame_id != self.config["control_frame"]:
                raise ValueError("Goal frame_id must be " + self.config["control_frame"])
            if not self.pose_is_fresh():
                raise ValueError("Fresh chassis pose is required")
            yaw = yaw_from_quaternion(quaternion_values(message.pose.orientation))
            self.controller.set_goal((message.pose.position.x, message.pose.position.y, yaw))
        except ValueError as error:
            self.controller.cancel()
            if self.control_started:
                self.publish_velocity()
            self.set_status("goal_rejected: " + str(error))
            return
        self.control_started = True
        self.set_status("tracking")

    def on_cancel(self, request, response):
        self.controller.cancel()
        self.control_started = True
        self.publish_velocity()
        self.set_status("cancelled")
        response.success = True
        response.message = "Goal cancelled; zero velocity published"
        return response

    def on_timer(self):
        now = time.monotonic()
        dt = max(1e-6, min(now - self.last_tick, 0.1))
        self.last_tick = now
        if self.pose_received_at is not None and not self.pose_is_fresh():
            source_age = self.get_clock().now().nanoseconds / 1e9 - self.pose_stamp
            self.get_logger().warning(
                f"Pose timeout: receive gap={now - self.pose_received_at:.3f}s, "
                f"source stamp age={source_age:.3f}s"
            )
            self.fault("pose_timeout_goal_cancelled")
        if not self.control_started:
            return
        if self.controller.goal is None:
            self.publish_velocity()
            return
        velocity, status = self.controller.step(self.pose, now, dt)
        self.publish_velocity(velocity)
        self.set_status(status)


def main():
    rclpy.init()
    node = None
    code = 0
    try:
        node = MocapPoseController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (RuntimeError, ValueError) as error:
        print(error, file=sys.stderr)
        code = 1
    finally:
        if node is not None:
            if rclpy.ok() and node.control_started:
                node.publish_velocity()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
