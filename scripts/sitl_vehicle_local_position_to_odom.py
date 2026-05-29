#!/usr/bin/env python3
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    half = yaw * 0.5
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class SitlLocalPositionToOdom(Node):
    def __init__(self, args):
        super().__init__("sitl_vehicle_local_position_to_odom")
        self.args = args
        self.count = 0
        self.invalid = 0
        self.start = time.time()
        self.summary_path = Path(args.summary) if args.summary else None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Odometry, args.output_topic, 10)
        self.create_subscription(VehicleLocalPosition, args.input_topic, self.cb, qos)
        self.create_timer(1.0, self.write_summary)

    def cb(self, msg: VehicleLocalPosition) -> None:
        if not (msg.xy_valid and msg.z_valid and not msg.dead_reckoning):
            self.invalid += 1
            return
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.args.frame_id
        out.child_frame_id = self.args.child_frame_id
        # PX4 local position is NED-like: x north, y east, z down.
        # EGO/ROS planning frame here is ENU-like: x east, y north, z up.
        out.pose.pose.position.x = float(msg.y)
        out.pose.pose.position.y = float(msg.x)
        out.pose.pose.position.z = float(-msg.z)
        out.pose.pose.orientation = yaw_to_quat(float(getattr(msg, "heading", 0.0)))
        out.twist.twist.linear.x = float(getattr(msg, "vy", 0.0))
        out.twist.twist.linear.y = float(getattr(msg, "vx", 0.0))
        out.twist.twist.linear.z = float(-getattr(msg, "vz", 0.0))
        self.pub.publish(out)
        self.count += 1

    def write_summary(self) -> None:
        if not self.summary_path:
            return
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.write_text(
            json.dumps(
                {
                    "input_topic": self.args.input_topic,
                    "output_topic": self.args.output_topic,
                    "frame_id": self.args.frame_id,
                    "child_frame_id": self.args.child_frame_id,
                    "duration_wall_s": time.time() - self.start,
                    "published": self.count,
                    "invalid_local_position": self.invalid,
                    "conversion": "PX4 NED local_position -> ROS ENU Odometry",
                    "passed": self.count > 0,
                },
                indent=2,
            ),
            encoding="utf-8",
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default="/fmu/out/vehicle_local_position")
    parser.add_argument("--output-topic", default="/sitl/odom_enu")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--child-frame-id", default="x500_base_link")
    parser.add_argument("--summary")
    args = parser.parse_args()
    rclpy.init()
    node = SitlLocalPositionToOdom(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
