# Reproduction Runbook

These commands assume an Ubuntu ROS2/PX4 development machine with:

- ROS2 Jazzy sourced from `/opt/ros/jazzy/setup.bash`
- PX4 SITL source tree at `$PX4_DIR`
- `px4_msgs` workspace sourced from `$PX4_MSGS_WS/install/setup.bash`
- ROS2 EGO-Planner workspace at `$EGO_WS`

Set these paths first:

```bash
export PX4_DIR=$HOME/PX4-Autopilot
export PX4_MSGS_WS=$HOME/px4_msgs_ws
export EGO_WS=$HOME/ros2_ego_ws
export UAV_DEMO=$HOME/uav_sitl_ego_demo
```

## 1. Start PX4 SITL With Gazebo

```bash
cd "$PX4_DIR"
PX4_GZ_WORLD=single_wall HEADLESS=1 make px4_sitl gz_x500
```

Why: Gazebo simulates the vehicle and world; PX4 SITL runs the flight controller logic.

Pass: PX4 prints Gazebo world ready and spawns `x500`.

Failure: missing world file, PX4 build error, or Gazebo startup error.

## 2. Start uXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

Why: PX4 publishes and receives uORB data through uXRCE-DDS; ROS2 sees these as `/fmu/*` topics.

Pass:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep /fmu/out/vehicle_local_position
```

Failure: no `/fmu/*` topics means PX4 and the Agent are not connected.

## 3. Convert PX4 Local Position to ROS ENU Odometry

```bash
source /opt/ros/jazzy/setup.bash
source "$PX4_MSGS_WS/install/setup.bash"
python3 "$UAV_DEMO/scripts/sitl_vehicle_local_position_to_odom.py" \
  --input-topic /fmu/out/vehicle_local_position \
  --output-topic /sitl/odom_enu
```

Why: PX4 uses NED; EGO and RViz use ENU. This bridge publishes `nav_msgs/Odometry`.

Pass:

```bash
ros2 topic hz /sitl/odom_enu
```

Failure: no rate means either PX4 local position is missing or `px4_msgs` was not sourced.

## 4. Publish the Single-Wall Obstacle Cloud

```bash
source /opt/ros/jazzy/setup.bash
python3 "$UAV_DEMO/scripts/sitl_virtual_obstacle_cloud.py" \
  --topic /sitl/virtual_obstacles \
  --scenario single_wall \
  --rate-hz 5.0 \
  --step 0.1 \
  --z-max 2.8 \
  --single-wall-x 2.0 \
  --single-wall-size-y 1.2
```

Why: EGO-Planner avoids point clouds, not Gazebo visual meshes. This command creates the obstacle EGO uses.

Pass:

```bash
ros2 topic echo --once /sitl/virtual_obstacles --field width
```

Expected: `918`.

Failure: a much smaller count means sparse points; EGO can leak through point gaps.

## 5. Relay Topics Into EGO Names

Use simple ROS2 relays or equivalent launch remaps:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run topic_tools relay /sitl/odom_enu /drone_0_visual_slam/odom
```

```bash
source /opt/ros/jazzy/setup.bash
ros2 run topic_tools relay /sitl/virtual_obstacles /drone_0_pcl_render_node/cloud
```

Why: the EGO launch file expects `drone_0_visual_slam/odom` and `drone_0_pcl_render_node/cloud`.

Pass: each relayed topic has one publisher and EGO subscribers.

## 6. Start EGO-Planner

Copy [config/advanced_param.launch.py](../config/advanced_param.launch.py) into the EGO launch directory, or apply the same `obstacles_inflation` and `lambda_fitness` argument changes.

```bash
cd "$EGO_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ego_planner advanced_param.launch.py \
  odometry_topic:=visual_slam/odom \
  cloud_topic:=pcl_render_node/cloud \
  point_num:=1 \
  point0_x:=-3.0 \
  point0_y:=0.0 \
  point0_z:=1.0 \
  flight_type:=2 \
  max_vel:=1.0 \
  max_acc:=1.5 \
  planning_horizon:=8.0 \
  obj_num_set:=0 \
  lambda_fitness:=5.0 \
  obstacles_inflation:=0.099
```

Why: the target is `(-3,0,1)`. EGO takes the current odometry as the start.

Pass: logs show `plan_success=1` and `refine_success=1`.

Failure: `drone is in obstacle` usually means the start pose is on the ground or inside an inflated obstacle.

## 7. Trigger Planning

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /traj_start_trigger geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 3.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

Why: EGO waits for a start trigger before planning.

Pass:

```bash
ros2 topic echo --once /drone_0_planning/bspline
```

## 8. Visualize in RViz

```bash
source /opt/ros/jazzy/setup.bash
python3 "$UAV_DEMO/scripts/publish_bspline_path_for_rviz.py" \
  --bspline "$UAV_DEMO/evidence/single_wall_x2_start3_goalneg3_bspline.yaml" \
  --start-x 3.0 --start-y 0.0 --start-z 1.0 \
  --goal-x -3.0 --goal-y 0.0 --goal-z 1.0 \
  --wall-x 2.0 --wall-size-y 1.2
```

Why: RViz does not directly display EGO B-spline messages by default; this publishes a `nav_msgs/Path`, obstacle cloud, and start/goal markers.

Pass: RViz shows a path that bends around the wall instead of crossing it.

## 9. Bridge EGO Commands to PX4 SITL

Start the command bridge:

```bash
source /opt/ros/jazzy/setup.bash
source "$PX4_MSGS_WS/install/setup.bash"
ALLOW_SITL_EGO_OFFBOARD=YES python3 "$UAV_DEMO/scripts/px4_sitl_ego_command_bridge.py" \
  --jsonl /tmp/px4_sitl_ego_command_bridge.jsonl \
  --summary /tmp/px4_sitl_ego_command_bridge_summary.json \
  --trace-jsonl /tmp/px4_sitl_ego_trace.jsonl \
  --arm \
  --force-arm \
  --land-at-end
```

Send EGO `PositionCommand` to the bridge:

```bash
source /opt/ros/jazzy/setup.bash
source "$EGO_WS/install/setup.bash"
python3 "$UAV_DEMO/scripts/ros2_pos_cmd_tcp_sender.py" \
  --topic /drone_0_planning/pos_cmd \
  --host 127.0.0.1 \
  --port 19002 \
  --duration 40
```

Why: EGO publishes `quadrotor_msgs/PositionCommand`; PX4 SITL accepts `px4_msgs/TrajectorySetpoint`.

Pass: the summary JSON reports `passed: true`, Offboard enabled, and nonzero displacement.

Failure: no command payload means EGO did not publish `pos_cmd`; arming/offboard failures usually point to PX4 health, GCS heartbeat, or setpoint streaming issues.
