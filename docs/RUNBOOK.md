# 复现实验手册

本文档记录如何复现 PX4 SITL + ROS2 + EGO-Planner 避障闭环。命令保留原始英文接口名，解释尽量使用中文。

假设 Ubuntu 机器上已经准备好：

- ROS2 Jazzy：`/opt/ros/jazzy/setup.bash`
- PX4 SITL 源码：`$PX4_DIR`
- `px4_msgs` 工作空间：`$PX4_MSGS_WS/install/setup.bash`
- ROS2 EGO-Planner 工作空间：`$EGO_WS`

先设置路径：

```bash
export PX4_DIR=$HOME/PX4-Autopilot
export PX4_MSGS_WS=$HOME/px4_msgs_ws
export EGO_WS=$HOME/ros2_ego_ws
export UAV_DEMO=$HOME/uav_sitl_ego_demo
```

## 1. 启动 PX4 SITL 和 Gazebo

```bash
cd "$PX4_DIR"
PX4_GZ_WORLD=single_wall HEADLESS=1 make px4_sitl gz_x500
```

为什么跑：Gazebo 负责仿真世界、物理和无人机模型；PX4 SITL 负责运行真实 PX4 飞控逻辑。

通过标准：终端打印 Gazebo world ready，并成功生成 `x500` 模型。

失败说明：可能是 world 文件没有放到 PX4 的 worlds 目录、PX4 没编译好，或者 Gazebo 启动失败。

## 2. 启动 uXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

为什么跑：PX4 内部是 uORB 消息，ROS2 不能直接读。uXRCE-DDS Agent 负责把 PX4 的 uORB 消息桥接成 ROS2 的 `/fmu/*` 话题。

检查命令：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep /fmu/out/vehicle_local_position
```

通过标准：能看到 `/fmu/out/vehicle_local_position`。

失败说明：PX4 和 Agent 没连上，或者 PX4 没启动 uXRCE-DDS client。

## 3. 把 PX4 本地位置转成 ROS ENU odometry

```bash
source /opt/ros/jazzy/setup.bash
source "$PX4_MSGS_WS/install/setup.bash"
python3 "$UAV_DEMO/scripts/sitl_vehicle_local_position_to_odom.py" \
  --input-topic /fmu/out/vehicle_local_position \
  --output-topic /sitl/odom_enu
```

为什么跑：PX4 的 local position 使用 NED 坐标系，EGO/RViz 常用 ENU 坐标系。这个桥把 PX4 local position 转成 `nav_msgs/Odometry`。

检查命令：

```bash
ros2 topic hz /sitl/odom_enu
```

通过标准：`/sitl/odom_enu` 有稳定频率，通常接近 PX4 local position 发布频率。

失败说明：PX4 local position 没数据，或者没有 source `px4_msgs`。

## 4. 发布单墙障碍物点云

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

为什么跑：EGO-Planner 避障依据是 `PointCloud2` 点云，不是 Gazebo 里肉眼看到的 visual mesh。这个命令给 EGO 发布一堵墙。

检查命令：

```bash
ros2 topic echo --once /sitl/virtual_obstacles --field width
```

通过标准：输出约 `918`。

失败说明：点数太少通常代表点云太稀疏，EGO 可能从点之间漏过去。

## 5. 把话题转到 EGO 需要的名字

可以用 ROS2 `topic_tools relay`，也可以在 launch 里 remap。

```bash
source /opt/ros/jazzy/setup.bash
ros2 run topic_tools relay /sitl/odom_enu /drone_0_visual_slam/odom
```

```bash
source /opt/ros/jazzy/setup.bash
ros2 run topic_tools relay /sitl/virtual_obstacles /drone_0_pcl_render_node/cloud
```

为什么跑：本项目使用的 EGO launch 默认读取 `drone_0_visual_slam/odom` 和 `drone_0_pcl_render_node/cloud`。

通过标准：relay 后的话题各有 publisher，EGO 启动后能看到 subscriber。

失败说明：EGO 会提示没有 odom，或者规划时没有障碍物。

## 6. 启动 EGO-Planner

先把 [config/advanced_param.launch.py](../config/advanced_param.launch.py) 放到 EGO 对应 launch 目录，或手动加入相同的 `obstacles_inflation` 和 `lambda_fitness` 参数。

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

为什么跑：这里把目标点设为 `(-3,0,1)`。EGO 的起点来自当前 odometry，不是来自命令行里的 `point0`。

通过标准：日志出现 `plan_success=1`，最好同时看到 `refine_success=1`。

失败说明：如果看到 `drone is in obstacle`，通常说明当前起点在地面、在墙体里，或落入膨胀体素。

## 7. 触发规划

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /traj_start_trigger geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 3.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

为什么跑：EGO 的 FSM 会等待 trigger，收到后才从当前 odometry 起点规划到目标点。

检查命令：

```bash
ros2 topic echo --once /drone_0_planning/bspline
```

通过标准：能收到 B-spline 消息，并且控制点或采样点绕开墙体。

## 8. 在 RViz 里显示轨迹

```bash
source /opt/ros/jazzy/setup.bash
python3 "$UAV_DEMO/scripts/publish_bspline_path_for_rviz.py" \
  --bspline "$UAV_DEMO/evidence/single_wall_x2_start3_goalneg3_bspline.yaml" \
  --start-x 3.0 --start-y 0.0 --start-z 1.0 \
  --goal-x -3.0 --goal-y 0.0 --goal-z 1.0 \
  --wall-x 2.0 --wall-size-y 1.2
```

为什么跑：RViz 默认不直接显示 EGO 的 B-spline 消息。这个脚本把 B-spline 采样成标准 `nav_msgs/Path`，同时发布墙体点云、起点和终点 marker。

通过标准：RViz 中能看到路径绕过墙，而不是穿过墙中间。

## 9. 把 EGO 命令桥接到 PX4 SITL

启动命令桥：

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

把 EGO 的 `PositionCommand` 发给桥：

```bash
source /opt/ros/jazzy/setup.bash
source "$EGO_WS/install/setup.bash"
python3 "$UAV_DEMO/scripts/ros2_pos_cmd_tcp_sender.py" \
  --topic /drone_0_planning/pos_cmd \
  --host 127.0.0.1 \
  --port 19002 \
  --duration 40
```

为什么跑：EGO 输出的是 `quadrotor_msgs/PositionCommand`，PX4 SITL 接受的是 `px4_msgs/TrajectorySetpoint`。中间需要做消息类型转换和 ENU/NED 坐标转换。

通过标准：summary JSON 里 `passed: true`，PX4 进入 Offboard，armed，并且 Gazebo 中无人机产生非零位移。

失败说明：如果 command payload 为 0，说明 EGO 没有输出 `pos_cmd`；如果不能 Offboard 或 arm，通常是 PX4 健康检查、GCS heartbeat 或 setpoint 连续发布频率的问题。
