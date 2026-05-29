#!/usr/bin/env python3
"""Send a MAVLink GCS heartbeat to PX4 SITL.

This is intentionally heartbeat-only. It does not arm, change modes, or send
setpoints. The purpose is to satisfy the SITL-only ground-station link health
check while testing Offboard plumbing.
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18570)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--rate-hz", type=float, default=1.0)
    parser.add_argument("--source-system", type=int, default=255)
    parser.add_argument("--source-component", type=int, default=190)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    period = 1.0 / max(args.rate_hz, 0.1)
    deadline = time.monotonic() + max(args.duration, 0.0)
    conn = mavutil.mavlink_connection(
        f"udpout:{args.host}:{args.port}",
        source_system=args.source_system,
        source_component=args.source_component,
        autoreconnect=True,
    )

    count = 0
    while time.monotonic() < deadline:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        count += 1
        time.sleep(period)

    print(f"sent_gcs_heartbeats={count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
