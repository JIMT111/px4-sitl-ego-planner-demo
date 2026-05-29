#!/usr/bin/env python3
import argparse
import math
import re
import time
from pathlib import Path as FsPath

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def parse_bspline(path):
    text = FsPath(path).read_text(encoding="utf-8")
    knots = [
        float(x)
        for x in re.findall(r"^\s*-\s*(-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\s*$", text, re.M)
    ]
    pts = [
        tuple(float(v) for v in match.groups())
        for match in re.finditer(r"- x: ([^\n]+)\n\s+y: ([^\n]+)\n\s+z: ([^\n]+)", text)
    ]
    return knots, pts


def de_boor(knots, pts, degree, t):
    n = len(pts) - 1
    if t >= knots[n + 1]:
        k = n
    else:
        k = max(degree, min(n, next(i - 1 for i in range(degree + 1, len(knots)) if knots[i] > t)))
    d = [list(pts[j]) for j in range(k - degree, k + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            i = k - degree + j
            denom = knots[i + degree + 1 - r] - knots[i]
            alpha = 0.0 if denom == 0.0 else (t - knots[i]) / denom
            d[j] = [(1.0 - alpha) * d[j - 1][axis] + alpha * d[j][axis] for axis in range(3)]
    return tuple(d[degree])


def sample_bspline(knots, pts, degree, count):
    start = knots[degree]
    end = knots[len(pts)]
    return [de_boor(knots, pts, degree, start + (end - start) * i / (count - 1)) for i in range(count)]


def frange(start, stop, step):
    vals = []
    value = start
    while value <= stop + 1e-9:
        vals.append(round(value, 4))
        value += step
    return vals


def wall_points(cx, cy, cz, sx, sy, sz, step, z_min, z_max):
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    z0, z1 = max(cz - sz / 2.0, z_min), min(cz + sz / 2.0, z_max)
    xs = frange(x0, x1, step)
    ys = frange(y0, y1, step)
    zs = frange(z0, z1, step)
    pts = []
    for x in xs:
        for z in zs:
            pts.append((x, y0, z))
            pts.append((x, y1, z))
    for y in ys:
        for z in zs:
            pts.append((x0, y, z))
            pts.append((x1, y, z))
    return pts


class BsplinePathViz(Node):
    def __init__(self, args):
        super().__init__("bspline_path_for_rviz")
        self.args = args
        knots, pts = parse_bspline(args.bspline)
        self.samples = sample_bspline(knots, pts, args.degree, args.samples)
        self.wall = wall_points(
            args.wall_x,
            args.wall_y,
            args.wall_z,
            args.wall_size_x,
            args.wall_size_y,
            args.wall_size_z,
            args.wall_step,
            args.z_min,
            args.z_max,
        )
        self.path_pub = self.create_publisher(Path, args.path_topic, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, args.cloud_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, args.marker_topic, 10)
        self.start = time.time()
        self.timer = self.create_timer(1.0 / args.rate_hz, self.publish_all)
        self.get_logger().info(
            f"publishing {len(self.samples)} path samples and {len(self.wall)} wall points for RViz"
        )

    def header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.args.frame_id
        return header

    def publish_all(self):
        if self.args.duration > 0 and time.time() - self.start > self.args.duration:
            rclpy.shutdown()
            return
        header = self.header()
        path = Path()
        path.header = header
        for x, y, z in self.samples:
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, self.wall))
        self.marker_pub.publish(self.markers(header))

    def markers(self, header):
        arr = MarkerArray()
        arr.markers.extend(
            [
                self.sphere_marker(header, 1, self.args.start_x, self.args.start_y, self.args.start_z, 0.0, 0.8, 0.2),
                self.sphere_marker(header, 2, self.args.goal_x, self.args.goal_y, self.args.goal_z, 0.9, 0.1, 0.1),
                self.wall_marker(header),
            ]
        )
        return arr

    def sphere_marker(self, header, marker_id, x, y, z, r, g, b):
        marker = Marker()
        marker.header = header
        marker.ns = "ego_demo_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0
        return marker

    def wall_marker(self, header):
        marker = Marker()
        marker.header = header
        marker.ns = "ego_demo_wall"
        marker.id = 3
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.args.wall_x
        marker.pose.position.y = self.args.wall_y
        marker.pose.position.z = (self.args.z_min + self.args.z_max) / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.args.wall_size_x
        marker.scale.y = self.args.wall_size_y
        marker.scale.z = self.args.z_max - self.args.z_min
        marker.color.r = 0.9
        marker.color.g = 0.45
        marker.color.b = 0.05
        marker.color.a = 0.45
        return marker


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bspline", required=True)
    parser.add_argument("--degree", type=int, default=3)
    parser.add_argument("--samples", type=int, default=240)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--path-topic", default="/viz/sitl_ego_setpoint_path")
    parser.add_argument("--cloud-topic", default="/viz/sitl_obstacle_points")
    parser.add_argument("--marker-topic", default="/viz/sitl_nav_markers")
    parser.add_argument("--start-x", type=float, default=3.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--start-z", type=float, default=1.0)
    parser.add_argument("--goal-x", type=float, default=-3.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-z", type=float, default=1.0)
    parser.add_argument("--wall-x", type=float, default=2.0)
    parser.add_argument("--wall-y", type=float, default=0.0)
    parser.add_argument("--wall-z", type=float, default=1.45)
    parser.add_argument("--wall-size-x", type=float, default=0.3)
    parser.add_argument("--wall-size-y", type=float, default=1.2)
    parser.add_argument("--wall-size-z", type=float, default=2.7)
    parser.add_argument("--wall-step", type=float, default=0.1)
    parser.add_argument("--z-min", type=float, default=0.2)
    parser.add_argument("--z-max", type=float, default=2.8)
    args = parser.parse_args()

    rclpy.init()
    node = BsplinePathViz(args)
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
