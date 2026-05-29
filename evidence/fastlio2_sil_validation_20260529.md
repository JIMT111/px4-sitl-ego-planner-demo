# FAST-LIO2 LiDAR+IMU Rosbag 离线 SIL 验证报告

Date: 2026-05-29
Host: yizhi-NB50TJ1-TK1
Scope: FAST-LIO2 offline / SIL validation only. No PX4 connection, no `/fmu/in/*` command, no arm/takeoff/offboard, no PX4 source modification.

## 1. 验证链路

```text
MID360S LiDAR+IMU rosbag
  -> ros2 bag play --clock / --loop
  -> /livox/lidar + /livox/imu
  -> FAST-LIO2 fast_lio/mapping.launch.py
  -> /Odometry + /path + /cloud_registered + /tf
  -> RViz camera_init fixed frame visualization
```

## 2. 输入 rosbag

Selected bag:

```text
/home/yizhi/drone_ws/bags/mid360s_livox_only_20260529_212055
```

`ros2 bag info` evidence:

```text
Files:             mid360s_livox_only_20260529_212055_0.mcap
Bag size:          76.2 MiB
Storage id:        mcap
ROS Distro:        jazzy
Duration:          19.514352810s
Messages:          4100
Topic: /livox/imu   Type: sensor_msgs/msg/Imu                 Count: 3904
Topic: /livox/lidar Type: livox_ros_driver2/msg/CustomMsg     Count: 196
```

Pre-record live input check:

```text
/livox/imu   average rate: about 200 Hz
/livox/lidar average rate: about 10 Hz
```

A later attempted re-record to `mid360s_livox_only_20260529_213015` hung before producing files/metadata and is not used as evidence.

## 3. FAST-LIO2 启动

Command entry:

```bash
./scripts/start_lio_candidate.sh
```

Script core command:

```bash
ros2 launch fast_lio mapping.launch.py \
  config_path:=/home/yizhi/uav/config \
  config_file:=fast_lio_mid360s.example.yaml \
  rviz:=false
```

Launch evidence:

```text
starting MID-360S LIO candidate
package=fast_lio launch=mapping.launch.py
config=/home/yizhi/uav/config/fast_lio_mid360s.example.yaml
expected odom=/Odometry
log=/home/yizhi/drone_ws/logs/mid360s_lio_candidate_20260520/start_lio_candidate_20260529_214047.log
started pid=49755
fastlio_mapping pid=49760
```

FAST-LIO2 log evidence:

```text
Node init finished.
IMU Initial Done
Initialize the map kdtree
```

## 4. Rosbag 回放

Single replay command used first:

```bash
ros2 bag play /home/yizhi/drone_ws/bags/mid360s_livox_only_20260529_212055 --clock
```

For stable observation, the same finite bag was then looped:

```bash
ros2 bag play /home/yizhi/drone_ws/bags/mid360s_livox_only_20260529_212055 --clock --loop
```

Loop replay is only for repeated observation. The source data duration remains 19.514 s.

## 5. FAST-LIO2 输出 topic

Observed topics:

```text
/Odometry
/path
/cloud_registered
/cloud_registered_body
/tf
/livox/imu
/livox/lidar
```

Topic publisher evidence:

```text
/Odometry          Type: nav_msgs/msg/Odometry       Publisher: laser_mapping
/path              Type: nav_msgs/msg/Path           Publisher: laser_mapping
/cloud_registered  Type: sensor_msgs/msg/PointCloud2 Publisher: laser_mapping
/tf                Type: tf2_msgs/msg/TFMessage      Publisher: laser_mapping
```

Frequency evidence during replay:

```text
/Odometry average rate: about 10.0 Hz
/path average rate: about 1.0 Hz
/cloud_registered average rate: about 10.0 Hz
```

## 6. Timestamp / frame_id / TF

Sample `/Odometry`:

```text
header.stamp: valid nonzero bag timestamp
header.frame_id: camera_init
child_frame_id: body
```

Sample `/path`:

```text
header.frame_id: camera_init
poses[*].header.frame_id: camera_init
```

Sample `/cloud_registered` header:

```text
header.stamp: valid nonzero bag timestamp
header.frame_id: camera_init
```

Dynamic TF:

```text
camera_init -> body
```

`/tf_static` was not published, but the required FAST-LIO2 dynamic transform `camera_init -> body` was present on `/tf`.

Note: during `--loop` replay, RViz/TF can print `TF_OLD_DATA` when the bag timestamp wraps from the end back to the beginning. This is a loop replay artifact, not a single-pass timestamp regression.

## 7. Odom 连续性 / 跳变 / 漂移判断

A 25 s odometry monitor over loop replay produced:

```text
samples=225
loop_time_resets=1
segments=2
segment_0_samples=62
segment_0_duration_header=6.101 s
segment_0_displacement_m=0.0024
segment_0_max_step_jump_m=0.0034
segment_0_max_position_norm_m=0.0291
segment_1_samples=163
segment_1_duration_header=16.200 s
segment_1_displacement_m=0.0027
segment_1_max_step_jump_m=0.0045
segment_1_max_position_norm_m=0.0308
frame_ids=[camera_init]
child_frame_ids=[body]
```

Judgment:

- Odom is continuous within each replay segment.
- No large single-step jump was observed. Maximum step jump was about 4.5 mm.
- No obvious drift was observed. Maximum position norm stayed about 3.1 cm.
- The selected valid bag has very small motion, so this is a stable/static-to-low-motion SIL result. It is not a strong aggressive-motion drift benchmark.

## 8. RViz 验证

RViz config:

```text
/home/yizhi/uav/config/rviz_nav_admission.rviz
```

RViz evidence screenshot:

```text
/home/yizhi/uav/reports/screenshots/fastlio2_sil_rviz_20260529.png
```

Observed in RViz:

- Fixed frame: `camera_init`
- Local map / registered cloud: `/cloud_registered`
- TF display enabled
- LIO odometry path display enabled through `/viz/lio_path`
- LIO body marker visible through `/viz/nav_markers`

Because the selected valid bag is low-motion, the trajectory is short, but the path topic and RViz path display are present.

## 9. 验收清单

| Requirement | Result | Evidence |
|---|---:|---|
| 1. rosbag can replay | PASS | `ros2 bag play ... --clock` returned `STATUS=0`; loop replay process active |
| 2. FAST-LIO2 can start | PASS | `start_lio_candidate_20260529_214047.log`; `fastlio_mapping` pid 49760 |
| 3. Outputs odom/path/local map | PASS | `/Odometry`, `/path`, `/cloud_registered`, `/cloud_registered_body` present |
| 4. RViz sees cloud/trajectory/map | PASS with low-motion caveat | screenshot `fastlio2_sil_rviz_20260529.png`; path display enabled and `/viz/lio_path` populated |
| 5. Check topic frequency/timestamp/frame_id/TF | PASS | odom 10 Hz, path 1 Hz, cloud 10 Hz; frame `camera_init`, child `body`, TF `camera_init -> body` |
| 6. Judge continuity/jump/drift | PASS with low-motion caveat | max step jump about 0.0045 m; max norm about 0.0308 m; no obvious drift |
| 7. Output report | PASS | this file |

## 10. 面试讲法

可以这样讲：

```text
我做的是 FAST-LIO2 的 LiDAR+IMU rosbag 离线 SIL 验证，不是 fake odom，也不是 PX4 Offboard。先确认 MID360S 的 /livox/lidar 和 /livox/imu 频率稳定，再录原始传感器 rosbag。随后停止实时 driver，保证输入只来自 rosbag replay。FAST-LIO2 启动后订阅 bag 回放的点云和 IMU，输出 /Odometry、/path、/cloud_registered 和 camera_init -> body 的 TF。我用 topic hz、echo、TF、RViz 和脚本检查频率、时间戳、frame_id、轨迹连续性和局部地图。当前包运动量较小，但 odom 连续、没有明显跳变或漂移，局部地图和路径可以在 RViz 里看到。
```

必须亲自掌握：

- FAST-LIO2 输入是 LiDAR 点云 + IMU，不是 `/Odometry`。
- `ros2 bag record` 用来录原始输入，`ros2 bag play` 用来离线回放。
- `topic list` 只说明 topic 存在，`topic hz` 才说明频率连续。
- `frame_id` 和 TF 决定 RViz 中点云、轨迹、位姿能否对齐。
- `--loop` 会造成时间戳回绕，不能把 loop 边界误判成算法时间倒退。
- 运动量不足时，不能夸大成强动态漂移验证。
