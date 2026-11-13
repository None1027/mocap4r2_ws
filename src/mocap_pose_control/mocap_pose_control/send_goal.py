"""Publish a planar goal in the controller's configured world frame."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, required=True, help="World-frame x in metres")
    parser.add_argument("--y", type=float, required=True, help="World-frame y in metres")
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--frame", default="mocap_world")
    parser.add_argument("--topic", default="/chassis/goal_pose")
    args = parser.parse_args()
    if not args.frame or not all(math.isfinite(value) for value in (args.x, args.y, args.yaw_deg)):
        parser.error("Specify a frame and finite x, y and yaw values")
    rclpy.init(args=[])
    node = Node("send_chassis_goal")
    publisher = node.create_publisher(PoseStamped, args.topic, 1)
    result = 0
    try:
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("No goal subscriber found within 5 seconds")
        goal = PoseStamped()
        goal.header.frame_id = args.frame
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = args.x, args.y
        yaw = math.radians(args.yaw_deg)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        publisher.publish(goal)
        if not publisher.wait_for_all_acked(Duration(seconds=2.0)):
            raise RuntimeError("Goal transport acknowledgement timed out")
        print("Goal published. Check /chassis/status for acceptance and reached status.")
    except (KeyboardInterrupt, RuntimeError) as error:
        print(error, file=sys.stderr)
        result = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return result
