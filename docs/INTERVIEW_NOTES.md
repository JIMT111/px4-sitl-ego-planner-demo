# Interview Notes

## One-Minute Explanation

I built a PX4 SITL closed-loop planning demo. Gazebo simulates the x500 vehicle and the world. PX4 SITL runs the flight-control logic. uXRCE-DDS bridges PX4 messages into ROS2. On the ROS2 side, I convert PX4 local position from NED to ENU odometry, publish a virtual wall as `PointCloud2`, run EGO-Planner to generate a B-spline trajectory, convert EGO commands into PX4 `TrajectorySetpoint`, and send them to `/fmu/in/trajectory_setpoint` so PX4 controls the simulated drone.

## Key Topics to Know

### PX4 SITL

SITL means the flight controller runs as software on the computer, while Gazebo provides simulated physics and sensors. It is useful because planning and control logic can be validated without risking real hardware.

### uXRCE-DDS

PX4 internally uses uORB messages. uXRCE-DDS exposes selected uORB topics to ROS2 as `/fmu/out/*` and accepts commands through `/fmu/in/*`.

### NED vs ENU

PX4 local position is in NED: x forward/north, y right/east, z down. ROS/RViz planning commonly uses ENU: x east, y north, z up. The odometry bridge converts:

```text
x_enu = y_ned
y_enu = x_ned
z_enu = -z_ned
```

### EGO-Planner Inputs

EGO needs:

- odometry: current vehicle state
- point cloud: obstacle map input
- target/trigger: where to plan

It outputs B-spline trajectories and `PositionCommand`.

### Why RViz Needed a Path Publisher

EGO's raw B-spline message is not a standard RViz display type. For visualization, I sampled the B-spline and published a standard `nav_msgs/Path`, plus markers for start, goal, and wall.

### Obstacle Inflation

The planner computes the center-point path of the drone, but the drone has size and tracking error. Therefore obstacles are inflated in the occupancy grid. In this demo the raw wall is `y=[-0.6, 0.6]`; the inflated safe region is roughly `y=[-0.699, 0.699]`. The planned path has `|y|=1.302` near the wall x-band, so it clears the inflated wall.

## Failure Modes I Diagnosed

- Sparse point cloud: the planner can pass through gaps between points.
- Start on ground: EGO can report the drone is in an obstacle.
- Too little fitting weight: optimization can smooth the trajectory back into the wall.
- Wrong frame: path appears offset or mirrored in RViz.
- Topic exists but no messages: node graph looks correct but planning still has no real data.

## Final Evidence

- RViz screenshot: [../evidence/rviz_sitl_ego_avoidance_success.png](../evidence/rviz_sitl_ego_avoidance_success.png)
- Sampling proof: [../evidence/single_wall_x2_start3_goalneg3_sampling.txt](../evidence/single_wall_x2_start3_goalneg3_sampling.txt)
- PX4 Offboard closed-loop summary: [../evidence/offboard_closed_loop_summary.json](../evidence/offboard_closed_loop_summary.json)
