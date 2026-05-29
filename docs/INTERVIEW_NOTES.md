# 面试讲解笔记

## 一分钟项目介绍

我做了一个 PX4 SITL + ROS2 + EGO-Planner 的无人机避障闭环。Gazebo 负责仿真 x500 无人机和障碍物环境，PX4 SITL 负责运行飞控逻辑，uXRCE-DDS 把 PX4 的 uORB 消息桥接到 ROS2。ROS2 侧把 PX4 local position 从 NED 坐标转换成 ENU odometry，同时发布一堵虚拟墙的 `PointCloud2`。EGO-Planner 读取 odom 和点云后生成 B-spline 轨迹，再通过桥接节点转换成 PX4 的 `/fmu/in/trajectory_setpoint`，最终让 PX4 控制 Gazebo 里的无人机运动。

## 必须掌握的关键点

### PX4 SITL 是什么

SITL 是 Software-In-The-Loop。飞控代码运行在电脑上，Gazebo 提供物理仿真、无人机模型和环境。它的价值是可以在不上真机的情况下验证规划、控制和消息链路。

### uXRCE-DDS 的作用

PX4 内部使用 uORB 消息。ROS2 不能直接读写 uORB，所以需要 uXRCE-DDS。它把 PX4 的输出暴露成 `/fmu/out/*`，也把 ROS2 发来的控制命令接到 `/fmu/in/*`。

### NED 和 ENU 坐标转换

PX4 local position 常用 NED：

- x：前方 / north
- y：右方 / east
- z：向下为正

ROS/RViz/EGO 常用 ENU：

- x：east
- y：north
- z：向上为正

本项目的转换关系是：

```text
x_enu = y_ned
y_enu = x_ned
z_enu = -z_ned
```

这个点面试很容易被问，因为坐标系错了，轨迹会镜像、旋转或者高度反号。

### EGO-Planner 的输入和输出

EGO 的核心输入：

- odometry：无人机当前状态
- point cloud：障碍物点云
- trigger / waypoint：规划触发和目标点

EGO 的核心输出：

- B-spline trajectory
- `quadrotor_msgs/PositionCommand`

### 为什么 RViz 需要额外的 Path 发布器

EGO 原始 B-spline 消息不是 RViz 的标准显示类型。为了让人能直观看到避障路线，我把 B-spline 采样成 `nav_msgs/Path`，同时发布起点、终点和墙体 marker。这样 RViz 里可以直接看到轨迹是否绕墙。

### 为什么需要障碍物膨胀

规划器算的是无人机质心轨迹，但真实无人机有机体半径、定位误差和控制误差。如果不膨胀障碍物，质心轨迹看起来不碰撞，真实机体仍然可能撞墙。

本项目里：

- 原始墙体范围：`y=[-0.6, 0.6]`
- 膨胀后安全范围：约 `y=[-0.699, 0.699]`
- 轨迹经过墙的 x 范围时最近 `|y|=1.302`

所以轨迹没有贴墙，也没有进入膨胀墙体。

## 我实际排查过的问题

- 点云太稀疏：EGO 可能从点云缝隙里穿过去。
- 起点在地面：EGO 可能认为 `drone is in obstacle`。
- 优化参数不合适：B-spline 可能被平滑项拉回墙里。
- 坐标系错误：RViz 里轨迹和障碍物会错位。
- 话题存在但没有消息：ROS graph 看起来正常，但 planner 实际没有数据。

## 最终证据

- RViz 成功截图：[../evidence/rviz_sitl_ego_avoidance_success.png](../evidence/rviz_sitl_ego_avoidance_success.png)
- B-spline 采样验证：[../evidence/single_wall_x2_start3_goalneg3_sampling.txt](../evidence/single_wall_x2_start3_goalneg3_sampling.txt)
- PX4 Offboard 闭环 summary：[../evidence/offboard_closed_loop_summary.json](../evidence/offboard_closed_loop_summary.json)

## 简历写法

可以写成：

```text
搭建 PX4 SITL + ROS2 + EGO-Planner 无人机避障闭环，完成 uXRCE-DDS 消息桥接、NED/ENU 里程计转换、PointCloud2 障碍物建图、B-spline 轨迹规划与 /fmu/in/trajectory_setpoint Offboard 控制，并通过 RViz 与采样检测验证轨迹绕过膨胀障碍物。
```
