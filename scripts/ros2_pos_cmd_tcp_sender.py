#!/usr/bin/env python3
import argparse
import json
import socket
import time

import rclpy
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node


class PosCmdTcpSender(Node):
    def __init__(self, args):
        super().__init__("ros2_pos_cmd_tcp_sender")
        self.args = args
        self.sock = None
        self.sent = 0
        self.dropped = 0
        self.last_connect_attempt = 0.0
        self.start_wall = time.time()
        self.create_subscription(PositionCommand, args.topic, self.on_cmd, 50)
        self.create_timer(1.0, self.report)

    def connect(self):
        if self.sock is not None:
            return True
        now = time.time()
        if now - self.last_connect_attempt < 1.0:
            return False
        self.last_connect_attempt = now
        try:
            self.sock = socket.create_connection((self.args.host, self.args.port), timeout=1.0)
            self.sock.settimeout(0.5)
            self.get_logger().info(f"connected to {self.args.host}:{self.args.port}")
            return True
        except OSError as exc:
            self.get_logger().warn(f"waiting for command bridge: {exc}")
            return False

    def on_cmd(self, msg):
        if self.args.duration > 0 and time.time() - self.start_wall > self.args.duration:
            return
        if not self.connect():
            self.dropped += 1
            return
        payload = {
            "stamp": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
            "frame_id": msg.header.frame_id,
            "position": [msg.position.x, msg.position.y, msg.position.z],
            "velocity": [msg.velocity.x, msg.velocity.y, msg.velocity.z],
            "acceleration": [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z],
            "yaw": msg.yaw,
            "yaw_dot": msg.yaw_dot,
        }
        try:
            self.sock.sendall((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
            self.sent += 1
        except OSError as exc:
            self.get_logger().warn(f"send failed: {exc}")
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
            self.dropped += 1

    def report(self):
        self.get_logger().info(f"sent={self.sent} dropped={self.dropped}")
        if self.args.duration > 0 and time.time() - self.start_wall > self.args.duration:
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/drone_0_planning/pos_cmd")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=19002)
    parser.add_argument("--duration", type=float, default=40.0)
    args = parser.parse_args()

    rclpy.init()
    node = PosCmdTcpSender(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sock is not None:
            node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
