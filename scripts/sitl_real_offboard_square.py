#!/usr/bin/env python3
import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def finite(values):
    return all(math.isfinite(float(v)) for v in values)


def set_if_present(msg, name, value):
    if hasattr(msg, name):
        setattr(msg, name, value)


class SitlRealOffboardSquare(Node):
    def __init__(self, args):
        super().__init__("sitl_real_offboard_square")
        if os.environ.get("ALLOW_SITL_REAL_OFFBOARD") != "YES":
            raise RuntimeError("Refusing real /fmu/in publication without ALLOW_SITL_REAL_OFFBOARD=YES")

        self.args = args
        self.start_wall = time.time()
        self.first_valid_local = None
        self.latest_local = None
        self.latest_status = None
        self.latest_control_mode = None
        self.ever_armed = False
        self.ever_offboard_nav = False
        self.ever_control_offboard = False
        self.local_count = 0
        self.status_count = 0
        self.control_mode_count = 0
        self.setpoint_count = 0
        self.offboard_count = 0
        self.command_count = 0
        self.invalid_setpoints = 0
        self.max_jump_m = 0.0
        self.max_position_error_m = 0.0
        self.max_displacement_m = 0.0
        self.last_setpoint = None
        self.mode_command_sent = False
        self.arm_command_sent = False
        self.land_command_sent = False
        self.trace = []

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(VehicleLocalPosition, args.local_position_topic, self.on_local, qos)
        self.create_subscription(VehicleStatus, args.vehicle_status_topic, self.on_status, qos)
        self.create_subscription(VehicleControlMode, args.control_mode_topic, self.on_control_mode, qos)

        self.traj_pub = self.create_publisher(TrajectorySetpoint, args.trajectory_topic, 10)
        self.offboard_pub = self.create_publisher(OffboardControlMode, args.offboard_topic, 10)
        self.command_pub = self.create_publisher(VehicleCommand, args.vehicle_command_topic, 10)
        self.timer = self.create_timer(1.0 / args.rate_hz, self.on_timer)

    def on_local(self, msg):
        self.latest_local = msg
        self.local_count += 1
        if self.args.trace_jsonl and (self.local_count % self.args.trace_stride == 0):
            self.trace.append({
                "kind": "local_position",
                "t": time.time() - self.start_wall,
                "x": float(msg.x),
                "y": float(msg.y),
                "z": float(msg.z),
                "xy_valid": bool(msg.xy_valid),
                "z_valid": bool(msg.z_valid),
                "dead_reckoning": bool(msg.dead_reckoning),
            })
        if self.first_valid_local is None and msg.xy_valid and msg.z_valid and not msg.dead_reckoning:
            self.first_valid_local = msg
        if self.first_valid_local is not None:
            dx = float(msg.x - self.first_valid_local.x)
            dy = float(msg.y - self.first_valid_local.y)
            dz = float(msg.z - self.first_valid_local.z)
            self.max_displacement_m = max(self.max_displacement_m, math.sqrt(dx * dx + dy * dy + dz * dz))
        if self.last_setpoint is not None:
            err = math.sqrt(
                (float(msg.x) - self.last_setpoint[0]) ** 2
                + (float(msg.y) - self.last_setpoint[1]) ** 2
                + (float(msg.z) - self.last_setpoint[2]) ** 2
            )
            self.max_position_error_m = max(self.max_position_error_m, err)

    def on_status(self, msg):
        self.latest_status = msg
        self.status_count += 1
        if int(msg.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED):
            self.ever_armed = True
        if int(msg.nav_state) == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.ever_offboard_nav = True

    def on_control_mode(self, msg):
        self.latest_control_mode = msg
        self.control_mode_count += 1
        if hasattr(msg, "flag_control_offboard_enabled") and bool(msg.flag_control_offboard_enabled):
            self.ever_control_offboard = True

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = int(command)
        for idx in range(1, 8):
            setattr(msg, f"param{idx}", float(params.get(f"param{idx}", 0.0)))
        msg.target_system = self.args.target_system
        msg.target_component = self.args.target_component
        msg.source_system = self.args.source_system
        msg.source_component = self.args.source_component
        msg.from_external = True
        self.command_pub.publish(msg)
        self.command_count += 1

    def setpoint_for_elapsed(self, elapsed):
        base = self.first_valid_local
        if base is None:
            return None
        # NED frame: z is positive down. A target_z of -1 means 1 m above origin.
        z = self.args.target_z_ned
        side = self.args.square_side_m
        period = self.args.square_period_s
        phase = (elapsed % period) / period
        if phase < 0.25:
            x = base.x + side * (phase / 0.25)
            y = base.y
        elif phase < 0.50:
            x = base.x + side
            y = base.y + side * ((phase - 0.25) / 0.25)
        elif phase < 0.75:
            x = base.x + side * (1.0 - (phase - 0.50) / 0.25)
            y = base.y + side
        else:
            x = base.x
            y = base.y + side * (1.0 - (phase - 0.75) / 0.25)
        return [float(x), float(y), float(z)]

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        set_if_present(msg, "actuator", False)
        set_if_present(msg, "thrust_and_torque", False)
        set_if_present(msg, "direct_actuator", False)
        self.offboard_pub.publish(msg)
        self.offboard_count += 1

    def publish_setpoint(self, position):
        if not finite(position):
            self.invalid_setpoints += 1
            return
        if self.last_setpoint is not None:
            jump = math.sqrt(sum((position[i] - self.last_setpoint[i]) ** 2 for i in range(3)))
            self.max_jump_m = max(self.max_jump_m, jump)
            if jump > self.args.max_jump_m:
                self.invalid_setpoints += 1
                return
        self.last_setpoint = list(position)
        if self.args.trace_jsonl:
            self.trace.append({
                "kind": "setpoint",
                "t": time.time() - self.start_wall,
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
            })

        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = position
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = math.nan
        self.traj_pub.publish(msg)
        self.setpoint_count += 1

    def on_timer(self):
        elapsed = time.time() - self.start_wall
        if elapsed > self.args.duration:
            if self.args.land_at_end and not self.land_command_sent:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.land_command_sent = True
                time.sleep(0.2)
            rclpy.shutdown()
            return

        if self.first_valid_local is None:
            return

        self.publish_offboard_mode()
        position = self.setpoint_for_elapsed(max(0.0, elapsed - self.args.warmup_s))
        if position is not None:
            self.publish_setpoint(position)

        if elapsed > self.args.warmup_s and not self.mode_command_sent:
            # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, PX4 custom main mode OFFBOARD=6.
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.mode_command_sent = True
        if elapsed > self.args.warmup_s + 0.5 and self.args.arm and not self.arm_command_sent:
            arm_param2 = 21196.0 if self.args.force_arm else 0.0
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=float(VehicleCommand.ARMING_ACTION_ARM),
                param2=arm_param2,
            )
            self.arm_command_sent = True

    def summary(self):
        status = self.latest_status
        local = self.latest_local
        control = self.latest_control_mode
        return {
            "duration_requested_s": self.args.duration,
            "duration_wall_s": time.time() - self.start_wall,
            "topics": {
                "trajectory": self.args.trajectory_topic,
                "offboard": self.args.offboard_topic,
                "vehicle_command": self.args.vehicle_command_topic,
                "local_position": self.args.local_position_topic,
                "vehicle_status": self.args.vehicle_status_topic,
            },
            "local_position_count": self.local_count,
            "vehicle_status_count": self.status_count,
            "control_mode_count": self.control_mode_count,
            "setpoint_count": self.setpoint_count,
            "offboard_mode_count": self.offboard_count,
            "vehicle_command_count": self.command_count,
            "mode_command_sent": self.mode_command_sent,
            "arm_command_sent": self.arm_command_sent,
            "force_arm_requested": self.args.force_arm,
            "land_command_sent": self.land_command_sent,
            "ever_armed": self.ever_armed,
            "ever_offboard_nav": self.ever_offboard_nav,
            "ever_control_offboard": self.ever_control_offboard,
            "invalid_setpoints": self.invalid_setpoints,
            "max_setpoint_jump_m": self.max_jump_m,
            "max_position_error_m": self.max_position_error_m,
            "max_displacement_m": self.max_displacement_m,
            "latest_local": {
                "xy_valid": bool(local.xy_valid) if local else None,
                "z_valid": bool(local.z_valid) if local else None,
                "dead_reckoning": bool(local.dead_reckoning) if local else None,
                "x": float(local.x) if local else None,
                "y": float(local.y) if local else None,
                "z": float(local.z) if local else None,
            },
            "latest_status": {
                "arming_state": int(status.arming_state) if status else None,
                "nav_state": int(status.nav_state) if status else None,
                "failsafe": bool(status.failsafe) if status else None,
                "pre_flight_checks_pass": bool(status.pre_flight_checks_pass) if status else None,
            },
            "latest_control_mode": {
                "flag_control_offboard_enabled": bool(control.flag_control_offboard_enabled)
                if control and hasattr(control, "flag_control_offboard_enabled")
                else None,
            },
            "published_to_real_fmu_in": True,
            "sitl_only": True,
            "passed": (
                self.local_count > 0
                and self.setpoint_count > 0
                and self.offboard_count > 0
                and self.mode_command_sent
                and (self.arm_command_sent or not self.args.arm)
                and (self.ever_armed or not self.args.arm)
                and self.ever_offboard_nav
                and self.ever_control_offboard
                and self.max_displacement_m >= self.args.min_displacement_m
                and self.invalid_setpoints == 0
                and (local is not None and local.xy_valid and local.z_valid and not local.dead_reckoning)
            ),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--warmup-s", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--local-position-topic", default="/fmu/out/vehicle_local_position")
    parser.add_argument("--vehicle-status-topic", default="/fmu/out/vehicle_status_v1")
    parser.add_argument("--control-mode-topic", default="/fmu/out/vehicle_control_mode")
    parser.add_argument("--trajectory-topic", default="/fmu/in/trajectory_setpoint")
    parser.add_argument("--offboard-topic", default="/fmu/in/offboard_control_mode")
    parser.add_argument("--vehicle-command-topic", default="/fmu/in/vehicle_command")
    parser.add_argument("--square-side-m", type=float, default=0.5)
    parser.add_argument("--square-period-s", type=float, default=12.0)
    parser.add_argument("--target-z-ned", type=float, default=-1.0)
    parser.add_argument("--max-jump-m", type=float, default=0.25)
    parser.add_argument("--target-system", type=int, default=1)
    parser.add_argument("--target-component", type=int, default=1)
    parser.add_argument("--source-system", type=int, default=1)
    parser.add_argument("--source-component", type=int, default=1)
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--force-arm", action="store_true")
    parser.add_argument("--land-at-end", action="store_true")
    parser.add_argument("--min-displacement-m", type=float, default=0.2)
    parser.add_argument("--summary", required=True)
    parser.add_argument("--trace-jsonl")
    parser.add_argument("--trace-stride", type=int, default=5)
    args = parser.parse_args()

    rclpy.init()
    node = SitlRealOffboardSquare(args)
    try:
        rclpy.spin(node)
    finally:
        payload = node.summary()
        Path(args.summary).parent.mkdir(parents=True, exist_ok=True)
        Path(args.summary).write_text(json.dumps(payload, indent=2), encoding="utf-8")
        if args.trace_jsonl:
            trace_path = Path(args.trace_jsonl)
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            with trace_path.open("w", encoding="utf-8") as handle:
                for item in node.trace:
                    handle.write(json.dumps(item, separators=(",", ":")) + "\n")
        print(json.dumps(payload, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
