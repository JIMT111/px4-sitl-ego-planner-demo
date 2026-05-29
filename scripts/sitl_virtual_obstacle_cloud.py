#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path
from typing import Iterable, List, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

Point = Tuple[float, float, float]


def box_surface_points(
    cx: float,
    cy: float,
    cz: float,
    sx: float,
    sy: float,
    sz: float,
    step: float,
    z_min: float,
    z_max: float,
) -> Iterable[Point]:
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    z0, z1 = max(cz - sz / 2.0, z_min), min(cz + sz / 2.0, z_max)
    xs = frange(x0, x1, step)
    ys = frange(y0, y1, step)
    zs = frange(z0, z1, step)
    for x in xs:
        for z in zs:
            yield (x, y0, z)
            yield (x, y1, z)
    for y in ys:
        for z in zs:
            yield (x0, y, z)
            yield (x1, y, z)


def frange(start: float, stop: float, step: float) -> List[float]:
    vals = []
    x = start
    while x <= stop + 1e-9:
        vals.append(round(x, 4))
        x += step
    return vals


def single_wall_points(args) -> List[Point]:
    # One simple vertical wall in the EGO/ROS planning frame.
    return list(
        box_surface_points(
            args.single_wall_x,
            args.single_wall_y,
            args.single_wall_z,
            args.single_wall_size_x,
            args.single_wall_size_y,
            args.single_wall_size_z,
            step=args.step,
            z_min=args.z_min,
            z_max=args.z_max,
        )
    )


class VirtualObstacleCloud(Node):
    def __init__(self, args):
        super().__init__("sitl_virtual_obstacle_cloud")
        self.args = args
        if args.scenario == "none":
            self.points = []
        else:
            self.points = single_wall_points(args)
        self.pub = self.create_publisher(PointCloud2, args.topic, 10)
        self.summary_path = Path(args.summary) if args.summary else None
        self.count = 0
        self.start = time.time()
        self.timer = self.create_timer(1.0 / args.rate_hz, self.on_timer)

    def on_timer(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.args.frame_id
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, self.points))
        self.count += 1
        if self.count == 1:
            self.get_logger().info(
                f"publishing {len(self.points)} virtual wall obstacle points on {self.args.topic}"
            )
        self.write_summary()

    def write_summary(self) -> None:
        if not self.summary_path:
            return
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.write_text(
            json.dumps(
                {
                    "topic": self.args.topic,
                    "frame_id": self.args.frame_id,
                    "scenario": self.args.scenario,
                    "point_count": len(self.points),
                    "intentional_empty": self.args.scenario == "none",
                    "publish_count": self.count,
                    "rate_hz": self.args.rate_hz,
                    "duration_wall_s": time.time() - self.start,
                    "safety": "Publishes only sensor_msgs/PointCloud2 virtual obstacles; no PX4 /fmu/in/* topics.",
                    "passed": self.count > 0 and len(self.points) > 0,
                },
                indent=2,
            ),
            encoding="utf-8",
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/sitl/virtual_obstacles")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--scenario", choices=["single_wall", "none"], default="single_wall")
    parser.add_argument("--step", type=float, default=0.6)
    parser.add_argument("--z-min", type=float, default=0.2)
    parser.add_argument("--z-max", type=float, default=2.4)
    parser.add_argument("--single-wall-x", type=float, default=1.0)
    parser.add_argument("--single-wall-y", type=float, default=0.0)
    parser.add_argument("--single-wall-z", type=float, default=1.45)
    parser.add_argument("--single-wall-size-x", type=float, default=0.3)
    parser.add_argument("--single-wall-size-y", type=float, default=4.0)
    parser.add_argument("--single-wall-size-z", type=float, default=2.7)
    parser.add_argument("--summary")
    args = parser.parse_args()
    rclpy.init()
    node = VirtualObstacleCloud(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
