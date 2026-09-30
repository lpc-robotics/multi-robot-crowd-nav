# Multi-Robot Crowd Navigation

基于 Arena-Rosnav 5、Isaac Sim 5.1 和 ROS 2 Humble 的多机器人行人环境导航平台。
本仓库从 `arena5_multi_ws` 提取全部已跟踪核心源码，提供独立构建的多机 overlay、
Nav2 导航、控制守护、同伴机器人代价地图，以及普通行人的多机器人社会力后端。

项目采用一个 Isaac Sim 进程、共享 `/clock` 和 `map`，每台 Jackal 使用独立的
`robot_N` 命名空间、Nav2 action、odometry、TF 和雷达话题。规划器可选择 NavFn
Dijkstra 或 A*，局部控制使用 DWB。各机器人独立导航；当前未实现集中式多机器人
路径协调、预约调度或死锁消解。

## 核心功能

- 配置驱动的单机、双机和四机启动，以及八机容量探测场景。
- 每台机器人的 Nav2 / 外部速度控制模式、命令守护、急停及取消隔离。
- Isaac 理想 D6 底盘、统一仿真时间、规范化雷达和角色实际位姿观测。
- 同伴机器人占据层：过滤自身机器人并处理过期同伴观测。
- `legacy_hunav`：保留原 HuNav 普通行人与六行为模式，以 `robot_1` 为社会参考。
- `multi_sfm`：使用同一时间快照中的全部机器人影响普通行人，并同步积分。
- 心理模型扩展协议、候选状态回滚和健康心跳；当前注册模型仅为 `noop`。
- WebRTC / Foxglove 可视化、轨迹对比、故障注入和运行指标采集工具。

## 源码结构

| 路径 | 内容 |
|---|---|
| `src/arena_isaac/` | Isaac 场景、机器人、行人、传感器与时钟桥接 |
| `src/arena_multi_bringup/` | 多机 launch、场景管理、Nav2 配置与行为树 |
| `src/arena_multi_control/` | 控制守护、任务 CLI、场景解析与轮子可视化 |
| `src/arena_peer_costmap/` | Nav2 同伴机器人占据插件 |
| `src/arena_multi_hunav/` | HuNav 适配、共同时间快照和心理模型接口 |
| `src/arena_multi_hunav_core/` | C++ 社会力计算服务、基准、固定版 LightSFM |
| `src/arena_multi_hunav_msgs/` | 多机社会力服务与消息接口 |
| `src/arena_people_msgs/`、`src/isaacsim_msgs/` | Isaac 行人和场景消息源码 |
| `config/` | 场景、行人、地图、世界、URDF 和任务配置 |
| `scripts/` | 构建、运行、清理、验证与发布工具 |
| `docs/`、`baseline/` | 接口、运行手册、版本和 underlay 保护清单 |
| `evidence/` | 历史报告与紧凑轨迹对比数据 |

## 环境与依赖

当前部署基于 Linux、NVIDIA GPU、Isaac Sim **5.1**、ROS 2 **Humble** 和
Nav2 **1.1.18**，ROS 与 Isaac Python 均使用 **3.11**。启动脚本默认要求所选 GPU
至少有 **8192 MiB 空闲显存**，这是启动检查阈值。

本仓库是多机 overlay，依赖已经构建的稳定 Arena5 underlay。现有主机使用：

- underlay：`/home/lpc/workspace/arena5_ws`
- ROS 环境：underlay 内的 `.conda/arena_ros`
- Isaac 环境：`/home/lpc/miniforge3/envs/isaaclab`

underlay 提供 `arena_humble_compat`、HuNav 消息/服务、Foxglove、Nav2 及原始消息包。
构建脚本复用 underlay 的 `arena_people_msgs` 和 `isaacsim_msgs`，不重复构建它们。
源码归档和恢复资料见配套仓库
[social-nav-x](https://github.com/lpc-robotics/social-nav-x/tree/3bc75dc4015bf6dd93277c312d7e0aef4ece6fab)。
该归档不包含 Conda 环境、Isaac 安装或已构建的 underlay。

`scripts/env.sh` 根据自身位置定位多机仓库，默认加载稳定 underlay 的环境。
`scripts/verify_baseline.py` 仍按 `baseline/stable_underlay.json` 中记录的原始路径、
Git 状态及文件哈希核验 underlay。因此修改 `ARENA_STABLE_WS` 本身不足以迁移基线；
在新主机使用前，应先恢复并核对匹配的 underlay 布局。不要为绕过失败直接刷新基线。

## 构建

以下示例适用于已准备好上述 underlay 的主机：

```bash
git clone https://github.com/lpc-robotics/multi-robot-crowd-nav.git
cd multi-robot-crowd-nav
source scripts/env.sh
python scripts/verify_baseline.py
scripts/build.sh
scripts/validate_offline.sh
```

构建产物写入当前仓库的 `build/`、`install/`、`log/`。启动前必须完成构建。
`validate_offline.sh` 还需要已构建的 C++ 测试程序和 ROS 消息类型。

## 启动场景

在仓库根目录执行：

```bash
source scripts/env.sh
# 双机独立 Nav2
GPU_ID=3 scripts/run_multirobot.sh config/scenarios/two_robots_dijkstra.yaml
# 或双机 + 六名普通行人的多机器人社会力场景
GPU_ID=3 scripts/run_multirobot.sh config/scenarios/two_robots_multi_sfm_6_nav2.yaml
```

每次选择一个场景；省略 `GPU_ID` 时自动选择空闲显存最多的 GPU。
常用场景及当前配置速度如下：

| 场景文件（`config/scenarios/`） | 用途 | 线速度 / 角速度上限 |
|---|---|---|
| `one_robot.yaml` | 单机 smoke | 0.5 m/s / 1.0 rad/s |
| `two_robots_dijkstra.yaml`、`two_robots_astar.yaml` | 双机导航算法对照 | 0.5 / 1.0 |
| `two_robots_external.yaml` | 双机外部控制 | 0.5 / 1.0 |
| `four_robots.yaml` | 四机独立导航 | 0.5 / 1.0 |
| `two_robots_hunav_regular.yaml`、`two_robots_hunav_six.yaml` | 原 HuNav 接入 | 0.5 / 1.0 |
| `two_robots_multi_sfm_1_nav2.yaml` | 双机 + 一名普通行人 | 0.5 / 1.0 |
| `two_robots_multi_sfm_6_nav2.yaml` | 双机 + 六名普通行人 | 0.26 / 1.0 |
| `two_robots_multi_sfm_1_external.yaml`、`two_robots_multi_sfm_6_external.yaml` | 普通行人 + 外部控制 | 0.5 / 1.0 |
| `eight_robots_capacity.yaml` | 八机容量探测 | 0.5 / 1.0 |

默认开发端口为 WebRTC TCP/UDP `49130/48030`、Foxglove `8795`，ROS domain 为
`71`。通过 `ARENA_WEBRTC_IP`、`ARENA_WEBRTC_SIGNAL_PORT`、`ARENA_WEBRTC_MEDIA_PORT`、
`ARENA_FOXGLOVE_PORT` 和 `ROS_DOMAIN_ID` 覆盖配置。
可用 `LIVESTREAM=false FOXGLOVE=false` 关闭可视化：

```bash
LIVESTREAM=false FOXGLOVE=false scripts/run_multirobot.sh config/scenarios/two_robots_dijkstra.yaml
```

开启 WebRTC 时 NVIDIA 可能生成 `NvStreamer-*.etli` 诊断文件；已设置 Git 忽略。
部署版 `scripts/delivery/` 使用原主机的固定 release 路径和端口；新构建请使用上面的
`scripts/run_multirobot.sh`。Windows 连接说明见 [WINDOWS_VISUALIZATION.md](docs/WINDOWS_VISUALIZATION.md)。

## 任务与控制

在第二个终端进入同一仓库并加载环境：

```bash
source scripts/env.sh
multi_nav_goal robot_1 12.0 3.0 0.0
multi_nav_waypoints robot_2 12.0,7.0,0.0 3.0,7.0,3.14159
multi_nav_cancel robot_1
ros2 service call /robot_1/emergency_stop std_srvs/srv/SetBool '{data: true}'
```

external 场景通过 `/{robot_N}/cmd_vel_external` 输入速度；Nav2 场景忽略该输入。
停止当前工作区登记的运行实例：

```bash
scripts/cleanup.sh
```

## 测试与历史证据

发布前检查包含 55 项 Python 测试、72 项场景/地图/Nav2 合同检查、8 项 HuNav
配置检查，以及固定版 LightSFM 的 5 个文件哈希。构建后使用
`scripts/validate_offline.sh` 执行完整离线流程。详细运行测试和指标命令见
[RUNBOOK.md](docs/RUNBOOK.md)。

已有 GPU 记录覆盖原平台的 1/2/4 机器人场景；八机记录显示容量限制，不能视为
通过导航验收。多 SFM 的既有验收范围是两台机器人、1/6 名普通行人及当时
`0.26 m/s` 的速度配置。后续修改为 `0.5 m/s` 的场景需独立核验，不能直接沿用旧结果。

验收源码摘要和历史证据保持原样。复制源码和发布本仓库不会刷新 GPU 验收；
`verify_multi_sfm_release.py` 会拒绝源码摘要不匹配的旧记录。相关资料：

- [多 SFM 架构与运动/时间合同](docs/MULTI_SFM.md)
- [接口定义](docs/INTERFACES.md)
- [原平台 GPU 记录](evidence/VALIDATION.md)
- [多 SFM 历史证据](evidence/multi_sfm/README.md)
- [八机容量报告](docs/EIGHT_ROBOT_CAPACITY_REPORT.md)
- [源码导入说明](SOURCE_IMPORT.md)

## 授权与来源

包级授权按各 `package.xml` 和保留的上游声明执行。私有拷贝的 LightSFM 保留
[BSD-3-Clause LICENSE](src/arena_multi_hunav_core/vendor/lightsfm/LICENSE) 和
[固定来源清单](src/arena_multi_hunav_core/vendor/lightsfm/SOURCE.json)。
本仓库导入不改变上游或既有源码授权。
