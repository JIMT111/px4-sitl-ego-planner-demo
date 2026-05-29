#!/usr/bin/env python3
import argparse
import json
import math
import os
import socket
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def finite(seq):
    return all(math.isfinite(float(v)) for v in seq)


def norm(seq):
    return math.sqrt(sum(float(v) * float(v) for v in seq))


def enu_to_ned(pos):
    return [float(pos[1]), float(pos[0]), -float(pos[2])]


def set_if_present(msg, name, value):
    if hasattr(msg, name):
        setattr(msg, name, value)


def real_serial_agent_running() -> bool:
    try:
        out = subprocess.check_output(["pgrep", "-af", "MicroXRCEAgent serial|usb-Auterion_PX4_FMU"], text=True)
        return bool(out.strip())
    except subprocess.CalledProcessError:
        return False


class Px4SitlEgoCommandBridge(Node):
    def __init__(self, args):
        super().__init__("px4_sitl_ego_command_bridge")
        if os.environ.get("ALLOW_SITL_EGO_OFFBOARD") != "YES":
            raise RuntimeError("Refusing /fmu/in publication without ALLOW_SITL_EGO_OFFBOARD=YES")
        if real_serial_agent_running():
            raise RuntimeError("Refusing /fmu/in publication while real Pixhawk 6C serial Agent is running")
        self.args = args
        self.start_wall = time.time()
        self.first_cmd_wall = None
        self.latest_cmd = None
        self.latest_cmd_wall = 0.0
        self.latest_local = None
        self.latest_status = None
        self.latest_control_mode = None
        self.first_valid_local = None
        self.local_count = 0
        self.status_count = 0
        self.control_count = 0
        self.command_payload_count = 0
        self.invalid_payload_count = 0
        self.setpoint_count = 0
        self.offboard_count = 0
        self.vehicle_command_count = 0
        self.mode_command_sent = False
        self.arm_command_sent = False
        self.land_command_sent = False
        self.ever_armed = False
        self.ever_offboard_nav = False
        self.ever_control_offboard = False
        self.max_pos_norm = 0.0
        self.max_vel_norm = 0.0
        self.max_acc_norm = 0.0
        self.max_displacement_m = 0.0
        self.max_position_error_m = 0.0
        self.trace = []
        self.summary_path = Path(args.summary)
        self.jsonl_path = Path(args.jsonl)
        self.trace_path = Path(args.trace_jsonl) if args.trace_jsonl else None
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        self.jsonl_path.parent.mkdir(parents=True, exist_ok=True)
        if self.trace_path:
            self.trace_path.parent.mkdir(parents=True, exist_ok=True)
        self.jsonl = self.jsonl_path.open("w", encoding="utf-8")
        self.trace_out = self.trace_path.open("w", encoding="utf-8") if self.trace_path else None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(VehicleLocalPosition, args.local_position_topic, self.on_local, qos)
        self.create_subscription(VehicleStatus, args.vehicle_status_topic, self.on_status, qos)
        self.create_subscription(VehicleControlMode, args.control_mode_topic, self.on_control_mode, qos)
        self.ts_pub = self.create_publisher(TrajectorySetpoint, args.trajectory_topic, 10)
        self.off_pub = self.create_publisher(OffboardControlMode, args.offboard_topic, 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, args.vehicle_command_topic, 10)
        self.server_thread = threading.Thread(target=self.serve, daemon=True)
        self.server_thread.start()
        self.timer = self.create_timer(1.0 / args.rate_hz, self.on_timer)
        self.summary_timer = self.create_timer(1.0, self.write_summary)

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def active_time_s(self):
        return 0.0 if self.first_cmd_wall is None else time.time() - self.first_cmd_wall

    def write_trace(self, payload):
        if not self.trace_out:
            return
        self.trace_out.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self.trace_out.flush()

    def on_local(self, msg):
        self.latest_local = msg
        self.local_count += 1
        if self.first_valid_local is None and msg.xy_valid and msg.z_valid and not msg.dead_reckoning:
            self.first_valid_local = msg
        if self.first_valid_local is not None:
            dx = float(msg.x - self.first_valid_local.x)
            dy = float(msg.y - self.first_valid_local.y)
            dz = float(msg.z - self.first_valid_local.z)
            self.max_displacement_m = max(self.max_displacement_m, math.sqrt(dx * dx + dy * dy + dz * dz))
        if self.latest_cmd is not None:
            target = enu_to_ned(self.latest_cmd["position"])
            err = math.sqrt(
                (float(msg.x) - target[0]) ** 2
                + (float(msg.y) - target[1]) ** 2
                + (float(msg.z) - target[2]) ** 2
            )
            self.max_position_error_m = max(self.max_position_error_m, err)
        if self.first_cmd_wall is not None and self.local_count % self.args.trace_stride == 0:
            self.write_trace(
                {
                    "kind": "local_position",
                    "t": self.active_time_s(),
                    "x_enu": float(msg.y),
                    "y_enu": float(msg.x),
                    "z_enu": float(-msg.z),
                    "x_ned": float(msg.x),
                    "y_ned": float(msg.y),
                    "z_ned": float(msg.z),
                    "xy_valid": bool(msg.xy_valid),
                    "z_valid": bool(msg.z_valid),
                    "dead_reckoning": bool(msg.dead_reckoning),
                }
            )

    def on_status(self, msg):
        self.latest_status = msg
        self.status_count += 1
        if int(msg.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED):
            self.ever_armed = True
        if int(msg.nav_state) == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.ever_offboard_nav = True

    def on_control_mode(self, msg):
        self.latest_control_mode = msg
        self.control_count += 1
        if hasattr(msg, "flag_control_offboard_enabled") and bool(msg.flag_control_offboard_enabled):
            self.ever_control_offboard = True

    def serve(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.args.bind_host, self.args.port))
            srv.listen(1)
            while rclpy.ok():
                try:
                    conn, _addr = srv.accept()
                except OSError:
                    break
                threading.Thread(target=self.handle_conn, args=(conn,), daemon=True).start()

    def handle_conn(self, conn):
        with conn:
            buf = b""
            while rclpy.ok():
                data = conn.recv(65536)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        self.handle_payload(json.loads(line.decode("utf-8")))
                    except Exception:
                        self.invalid_payload_count += 1

    def handle_payload(self, payload):
        pos = payload.get("position", [math.nan, math.nan, math.nan])
        vel = payload.get("velocity", [0.0, 0.0, 0.0])
        acc = payload.get("acceleration", [0.0, 0.0, 0.0])
        yaw = float(payload.get("yaw", 0.0))
        yaw_dot = float(payload.get("yaw_dot", 0.0))
        valid = finite(pos) and finite(vel) and finite(acc) and math.isfinite(yaw) and math.isfinite(yaw_dot)
        pos_norm = norm(pos) if valid else math.inf
        vel_norm = norm(vel) if valid else math.inf
        acc_norm = norm(acc) if valid else math.inf
        valid = valid and pos_norm <= self.args.max_pos_norm and vel_norm <= self.args.max_vel_norm and acc_norm <= self.args.max_acc_norm
        self.max_pos_norm = max(self.max_pos_norm, pos_norm if math.isfinite(pos_norm) else 0.0)
        self.max_vel_norm = max(self.max_vel_norm, vel_norm if math.isfinite(vel_norm) else 0.0)
        self.max_acc_norm = max(self.max_acc_norm, acc_norm if math.isfinite(acc_norm) else 0.0)
        self.command_payload_count += 1
        if not valid:
            self.invalid_payload_count += 1
            return
        self.latest_cmd = {
            "position": [float(v) for v in pos],
            "velocity": [float(v) for v in vel],
            "acceleration": [float(v) for v in acc],
            "yaw": yaw,
            "yaw_dot": yaw_dot,
        }
        if self.first_cmd_wall is None:
            self.first_cmd_wall = time.time()
        self.latest_cmd_wall = time.time()

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
        self.cmd_pub.publish(msg)
        self.vehicle_command_count += 1

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
        self.off_pub.publish(msg)
        self.offboard_count += 1

    def publish_setpoint(self):
        if self.latest_cmd is None:
            return
        if time.time() - self.latest_cmd_wall > self.args.command_timeout_s:
            return
        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        position_ned = enu_to_ned(self.latest_cmd["position"])
        sp.position = position_ned
        sp.velocity = [math.nan, math.nan, math.nan]
        sp.acceleration = [math.nan, math.nan, math.nan]
        sp.jerk = [math.nan, math.nan, math.nan]
        sp.yaw = math.nan
        sp.yawspeed = math.nan
        self.ts_pub.publish(sp)
        self.setpoint_count += 1
        if self.setpoint_count <= 5 or self.setpoint_count % self.args.trace_stride == 0:
            self.write_trace(
                {
                    "kind": "setpoint",
                    "t": self.active_time_s(),
                    "x_enu": float(self.latest_cmd["position"][0]),
                    "y_enu": float(self.latest_cmd["position"][1]),
                    "z_enu": float(self.latest_cmd["position"][2]),
                    "x_ned": float(position_ned[0]),
                    "y_ned": float(position_ned[1]),
                    "z_ned": float(position_ned[2]),
                }
            )
        if self.setpoint_count <= 5 or self.setpoint_count % self.args.log_every == 0:
            self.jsonl.write(
                json.dumps(
                    {
                        "setpoint_count": self.setpoint_count,
                        "position_enu": self.latest_cmd["position"],
                        "position_ned": position_ned,
                        "published_to_real_fmu_in": True,
                        "sitl_only": True,
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )
            self.jsonl.flush()

    def on_timer(self):
        active_elapsed = 0.0 if self.first_cmd_wall is None else time.time() - self.first_cmd_wall
        if self.first_cmd_wall is not None and active_elapsed > self.args.duration:
            if self.args.land_at_end and not self.land_command_sent:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.land_command_sent = True
                time.sleep(0.2)
            rclpy.shutdown()
            return
        if self.first_valid_local is None or self.latest_cmd is None:
            return
        self.publish_offboard_mode()
        self.publish_setpoint()
        if active_elapsed > self.args.warmup_s and not self.mode_command_sent:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.mode_command_sent = True
        if active_elapsed > self.args.warmup_s + 0.5 and self.args.arm and not self.arm_command_sent:
            arm_param2 = 21196.0 if self.args.force_arm else 0.0
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=float(VehicleCommand.ARMING_ACTION_ARM),
                param2=arm_param2,
            )
            self.arm_command_sent = True

    def write_summary(self):
        local = self.latest_local
        status = self.latest_status
        control = self.latest_control_mode
        payload = {
            "duration_requested_s": self.args.duration,
            "duration_wall_s": time.time() - self.start_wall,
            "active_duration_s": 0.0 if self.first_cmd_wall is None else time.time() - self.first_cmd_wall,
            "command_payload_count": self.command_payload_count,
            "invalid_payload_count": self.invalid_payload_count,
            "setpoint_count": self.setpoint_count,
            "offboard_count": self.offboard_count,
            "vehicle_command_count": self.vehicle_command_count,
            "mode_command_sent": self.mode_command_sent,
            "arm_command_sent": self.arm_command_sent,
            "land_command_sent": self.land_command_sent,
            "ever_armed": self.ever_armed,
            "ever_offboard_nav": self.ever_offboard_nav,
            "ever_control_offboard": self.ever_control_offboard,
            "max_pos_norm": self.max_pos_norm,
            "max_vel_norm": self.max_vel_norm,
            "max_acc_norm": self.max_acc_norm,
            "max_displacement_m": self.max_displacement_m,
            "max_position_error_m": self.max_position_error_m,
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
                self.command_payload_count > 0
                and self.invalid_payload_count == 0
                and self.setpoint_count > 0
                and self.offboard_count > 0
                and self.mode_command_sent
                and (self.arm_command_sent or not self.args.arm)
                and (self.ever_armed or not self.args.arm)
                and self.ever_offboard_nav
                and self.ever_control_offboard
                and self.max_displacement_m >= self.args.min_displacement_m
                and local is not None
                and bool(local.xy_valid)
                and bool(local.z_valid)
                and not bool(local.dead_reckoning)
            ),
        }
        self.summary_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=19002)
    parser.add_argument("--duration", type=float, default=35.0)
    parser.add_argument("--warmup-s", type=float, default=4.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--command-timeout-s", type=float, default=1.0)
    parser.add_argument("--local-position-topic", default="/fmu/out/vehicle_local_position")
    parser.add_argument("--vehicle-status-topic", default="/fmu/out/vehicle_status_v1")
    parser.add_argument("--control-mode-topic", default="/fmu/out/vehicle_control_mode")
    parser.add_argument("--trajectory-topic", default="/fmu/in/trajectory_setpoint")
    parser.add_argument("--offboard-topic", default="/fmu/in/offboard_control_mode")
    parser.add_argument("--vehicle-command-topic", default="/fmu/in/vehicle_command")
    parser.add_argument("--target-system", type=int, default=1)
    parser.add_argument("--target-component", type=int, default=1)
    parser.add_argument("--source-system", type=int, default=1)
    parser.add_argument("--source-component", type=int, default=1)
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--force-arm", action="store_true")
    parser.add_argument("--land-at-end", action="store_true")
    parser.add_argument("--min-displacement-m", type=float, default=0.5)
    parser.add_argument("--max-pos-norm", type=float, default=30.0)
    parser.add_argument("--max-vel-norm", type=float, default=5.0)
    parser.add_argument("--max-acc-norm", type=float, default=10.0)
    parser.add_argument("--log-every", type=int, default=100)
    parser.add_argument("--jsonl", required=True)
    parser.add_argument("--summary", required=True)
    parser.add_argument("--trace-jsonl")
    parser.add_argument("--trace-stride", type=int, default=5)
    args = parser.parse_args()

    rclpy.init()
    node = Px4SitlEgoCommandBridge(args)
    try:
        rclpy.spin(node)
    finally:
        node.write_summary()
        node.jsonl.close()
        if node.trace_out:
            node.trace_out.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
