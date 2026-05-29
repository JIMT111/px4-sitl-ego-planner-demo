# 运行环境和版本记录

本文档用于说明别人复现本项目时需要的版本、运行条件和安全边界。核心原则是：SITL 环境和真实 Pixhawk 6C 环境要分开，不能混用 `/fmu/in/*` 控制话题。

## 已验证环境

本项目证据来自下面这组环境：

- 主机：Ubuntu 桌面环境，x86_64 机器
- ROS2：Jazzy，入口脚本 `/opt/ros/jazzy/setup.bash`
- Gazebo：Gazebo Sim `8.11.0`
- PX4 SITL：`PX4-Autopilot`，源码版本记录为 `v1.16.2-dirty`
- SITL 机型：`gz_x500`
- SITL world：本仓库的 `config/single_wall.sdf`
- uXRCE-DDS Agent：UDP 模式，`MicroXRCEAgent udp4 -p 8888`
- `px4_msgs`：SITL 使用 PX4 1.16 匹配的 `px4_msgs`
- EGO-Planner：ROS2 版本，要求存在 `ego_planner` 和 `quadrotor_msgs`
- Python：Ubuntu 24.04 / ROS2 Jazzy 默认 Python 3 环境

真实 Pixhawk 6C 曾观察到的固件版本是 PX4 `1.14.3`。这点要单独说明：本仓库的闭环证据是 PX4 1.16 SITL，不等于真实 Pixhawk 6C 已经执行 Offboard 飞行。

## 推荐目录约定

复现者不必使用完全相同的绝对路径，但下面的环境变量要指向自己的实际目录：

```bash
export PX4_DIR=$HOME/PX4-Autopilot
export PX4_MSGS_WS=$HOME/px4_msgs_ws
export EGO_WS=$HOME/ros2_ego_ws
export UAV_DEMO=$HOME/uav_sitl_ego_demo
```

如果使用本项目当时的机器，`PX4_MSGS_WS` 应指向 PX4 1.16 对应的 `px4_msgs` 工作空间，而不是 Pixhawk 6C 真机 PX4 1.14 对应的工作空间。

## 版本自查命令

复现前建议先把这些命令输出保存下来，避免“能跑但说不清版本”：

```bash
cat /etc/os-release | grep PRETTY_NAME
```

为什么跑：确认 Ubuntu 发行版。

通过标准：能看到 Ubuntu 版本。ROS2 Jazzy 通常建议配 Ubuntu 24.04。

```bash
source /opt/ros/jazzy/setup.bash
ros2 doctor --report | head -80
```

为什么跑：确认 ROS2 环境能正常 source，并查看 ROS2 基础信息。

通过标准：`ros2` 命令可用，报告里没有基础环境错误。

```bash
gz sim --versions
```

为什么跑：确认 Gazebo Sim 已安装，以及版本是否接近本项目验证过的 `8.11.0`。

通过标准：能看到 Gazebo Sim 版本；如果提示 `gz: command not found`，说明仿真运行时没装好。

```bash
MicroXRCEAgent --version
```

为什么跑：确认 uXRCE-DDS Agent 可用。

通过标准：能输出版本或帮助信息；如果命令不存在，ROS2 无法通过 Agent 收到 PX4 `/fmu/*` 话题。

```bash
git -C "$PX4_DIR" describe --tags --dirty --always
```

为什么跑：记录 PX4 SITL 源码版本。

通过标准：本项目验证过的是 `v1.16.2-dirty` 附近的 PX4 1.16 环境；如果你是 1.14 或 1.15，要同步检查 `px4_msgs`。

```bash
git -C "$PX4_MSGS_WS/src/px4_msgs" describe --tags --dirty --always
```

为什么跑：确认 ROS2 使用的 `px4_msgs` 与 PX4 SITL 版本匹配。

通过标准：PX4 1.16 SITL 搭配 PX4 1.16 `px4_msgs`。版本不匹配时，`ros2 topic list` 可能能看到话题，但 typed echo 或脚本字段解析会失败。

```bash
source /opt/ros/jazzy/setup.bash
source "$EGO_WS/install/setup.bash"
ros2 pkg prefix ego_planner
ros2 pkg prefix quadrotor_msgs
```

为什么跑：确认 EGO-Planner 和 `quadrotor_msgs` 在 ROS2 工作空间里可见。

通过标准：两个命令都能输出安装路径；缺任何一个，后面的 EGO launch 或 `PositionCommand` 订阅都会失败。

## 必须满足的运行条件

- 本仓库只用于 SITL，不用于真机 arm 或真机 Offboard。
- 启动 SITL 控制桥前，要停止真实 Pixhawk 6C 的 serial uXRCE-DDS Agent。
- SITL Agent 使用 UDP：`MicroXRCEAgent udp4 -p 8888`。
- PX4 SITL 使用 `PX4_GZ_WORLD=single_wall HEADLESS=1 make px4_sitl gz_x500`。
- `config/single_wall.sdf` 需要复制到 PX4 的 Gazebo worlds 目录。
- EGO launch 需要支持 `obstacles_inflation` 和 `lambda_fitness` 参数。
- `/sitl/odom_enu` 必须有稳定 odometry，通常接近 `100 Hz`。
- `/sitl/virtual_obstacles` 必须有稳定 `PointCloud2`，本项目使用 `5 Hz`。
- 只有显式设置 `ALLOW_SITL_EGO_OFFBOARD=YES`，桥接脚本才允许向 SITL `/fmu/in/*` 发布。

## 已验证场景参数

- 起点：`(3, 0, 1)`，ROS/ENU
- 终点：`(-3, 0, 1)`，ROS/ENU
- 墙中心：`(2, 0, 1.45)`
- 原始墙体：`x=[1.85, 2.15]`，`y=[-0.6, 0.6]`，`z=[0.2, 2.8]`
- 膨胀系数：`grid_map/obstacles_inflation=0.099`
- 验收结果：B-spline 采样点进入膨胀墙体数量为 `0`

## 面试怎么讲

可以这样说：

```text
我记录了 SITL 运行环境，而不是只放一张截图。PX4 SITL 使用 1.16 版本环境和对应 px4_msgs，通过 UDP uXRCE-DDS Agent 接到 ROS2 Jazzy；真实 Pixhawk 6C 曾是 PX4 1.14.3，所以我明确把真机环境和 SITL 环境分开。这样别人复现时能先检查版本、topic 和安全边界，再启动 EGO 到 PX4 Offboard 的闭环。
```
