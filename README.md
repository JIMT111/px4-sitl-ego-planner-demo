# PX4 SITL + ROS2 EGO-Planner Avoidance Demo

这是一个无人机软件在环闭环项目，目标是把 Gazebo、PX4 SITL、uXRCE-DDS、ROS2 和 EGO-Planner 串成一条可验证链路：

```text
Gazebo / PX4 SITL world
        -> PX4 flight-control logic
        -> uXRCE-DDS bridge
        -> ROS2 odometry + obstacle point cloud
        -> EGO-Planner B-spline trajectory
        -> ROS2 /fmu/in/trajectory_setpoint
        -> PX4 controls the simulated drone
```

## Result

The validated scenario uses a single wall between the start and goal:

- Start: `(3, 0, 1)` in ENU / ROS frame
- Goal: `(-3, 0, 1)` in ENU / ROS frame
- Wall center: `(2, 0, 1.45)`
- Wall bounds: `x=[1.85, 2.15]`, `y=[-0.6, 0.6]`, `z=[0.2, 2.8]`
- Inflated safety bounds: approximately `x=[1.751, 2.249]`, `y=[-0.699, 0.699]`, `z=[0.1, 2.9]`

RViz evidence:

![RViz SITL EGO avoidance evidence](evidence/rviz_sitl_ego_avoidance_success.png)

The sampled B-spline check is in [evidence/single_wall_x2_start3_goalneg3_sampling.txt](evidence/single_wall_x2_start3_goalneg3_sampling.txt):

```text
samples_inside_inflated_wall: 0
min_abs_y_when_x_in_wall_band: 1.302
passed: True
```

The closed-loop PX4 Offboard evidence is in [evidence/offboard_closed_loop_summary.json](evidence/offboard_closed_loop_summary.json). It records that ROS2 published real SITL `/fmu/in/trajectory_setpoint` messages and PX4 entered Offboard/armed state.

## Repository Layout

```text
config/
  advanced_param.launch.py          # EGO launch file with obstacle inflation and lambda_fitness args
  single_wall.sdf                   # Gazebo world with a simple wall
docs/
  RUNBOOK.md                        # Step-by-step reproduction commands
  INTERVIEW_NOTES.md                # Interview-ready explanation
evidence/
  offboard_closed_loop_summary.json
  single_wall_x2_start3_goalneg3_bspline.yaml
  single_wall_x2_start3_goalneg3_sampling.txt
  rviz_sitl_ego_avoidance_success.png
scripts/
  sitl_vehicle_local_position_to_odom.py
  sitl_virtual_obstacle_cloud.py
  ros2_pos_cmd_tcp_sender.py
  px4_sitl_ego_command_bridge.py
  publish_bspline_path_for_rviz.py
  sitl_gcs_heartbeat.py
  sitl_real_offboard_square.py
```

## What I Verified

- PX4 SITL and Gazebo can run the simulated x500 vehicle.
- uXRCE-DDS exposes PX4 `/fmu/out/*` and `/fmu/in/*` topics in ROS2.
- `/fmu/out/vehicle_local_position` is converted from PX4 NED to ROS ENU odometry.
- A virtual single-wall obstacle is published as `sensor_msgs/PointCloud2`.
- EGO-Planner receives odometry and obstacle cloud, then publishes a B-spline trajectory.
- The trajectory avoids the inflated wall volume.
- EGO `PositionCommand` can be bridged to PX4 `/fmu/in/trajectory_setpoint`.
- PX4 enters Offboard/armed state and moves the Gazebo vehicle in SITL.

## Safety Boundary

This repository is for SITL. The command bridge refuses to publish `/fmu/in/*` unless `ALLOW_SITL_EGO_OFFBOARD=YES` is set, and it checks for a real Pixhawk serial uXRCE-DDS Agent before publishing. Do not use this code on real hardware without a separate hardware safety review.

## Why Obstacle Inflation Matters

EGO-Planner plans a trajectory for the vehicle center point, but a real quadrotor has body radius, localization error, and tracking error. The occupancy map therefore inflates obstacles with `grid_map/obstacles_inflation`. If inflation is too small or the point cloud is too sparse, the planner can produce a path that appears valid mathematically but is unsafe physically.

In this demo:

- raw wall: `y=[-0.6, 0.6]`
- inflated wall: approximately `y=[-0.699, 0.699]`
- closest sampled trajectory point in the wall x-band: `|y|=1.302`

So the planned path has clear clearance around the wall.
