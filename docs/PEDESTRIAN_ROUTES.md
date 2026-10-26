# 行人多目标点循环与折返

默认动态心理场景自2026-10-22起使用`multi_regular_8_psychology_multigoal.yaml`，
四人循环、四人四点折返；原六人起点保留，横向路线跨度扩大至4.5米，右侧新增两人。
原`multi_regular_6_psychology.yaml`保留供历史对照及增强传播场景使用。
具体布局见[八人多目标心理场景](PSYCHOLOGY_MULTIGOAL_20261022.md)。
CURIOUS/SCARED期间暂停原队列，恢复REGULAR后
按实际位置继续，临时机器人目标不会写入队列。旧路线与原普通行人验收保持原样。
心理验收需同时检查allow_task_progress，不能直接使用原无暂停周期速度门槛。

该功能用于 `multi_sfm` 普通行人。每个人有自己的目标点列表与模式，仍由现有社会力计算
目标吸引、行人避让、逐机器人 FOV 社会力及独立近距离排斥。目标点是依次到达的位置，
不是必须严格贴合的折线轨迹；途中避让可以使角色偏离连线。

## 启动

```bash
cd /home/lpc/workspace/arena5_multi_ws
source scripts/env.sh
scripts/build.sh
scripts/validate_offline.sh
scripts/run_multirobot.sh config/scenarios/two_robots_multi_sfm_6_multigoal_nav2.yaml
```

此入口使用新配置 `config/hunav/multi_regular_6_multigoal.yaml`，前三人循环、后三人折返，
每人四个基础目标。出生点、角色模型、行人半径 0.4 m、最大速度 1 m/s、目标半径 0.3 m
沿用当前六人场景。双机 Nav2 上限为 0.26 m/s、1 rad/s，物理步长 0.05 s，行人积分步长
0.025 s。旧 `two_robots_multi_sfm_6_nav2.yaml` 保留原两点运动。

Windows/WebRTC 和 Foxglove 的环境变量沿用 [WINDOWS_VISUALIZATION.md](WINDOWS_VISUALIZATION.md)。
`command.txt` 是原有用户文件，本次没有改写；在原命令中仅替换场景参数即可体验新路线。

## 配置规则

在每个人现有的 `hunav_loader.ros__parameters.person_N` 下设置：

```yaml
route_mode: ping_pong
goal_radius: 0.3
cyclic_goals: true
goals: [g0, g1, g2, g3]
g0: {x: 12.2, y: 4.5, h: 0.0}
g1: {x: 14.2, y: 5.5, h: 0.0}
g2: {x: 15.5, y: 8.5, h: 0.0}
g3: {x: 17.0, y: 10.0, h: 0.0}
```

| 模式 | 每个周期的目标队列 |
|---|---|
| `loop` | g0 → g1 → g2 → g3，然后回到 g0 |
| `ping_pong` | g0 → g1 → g2 → g3 → g2 → g1，然后回到 g0 |
| 省略 `route_mode` | 保留原 `goals` 顺序，由原 `cyclic_goals` 决定循环或逐个消费 |

第一个目标始终是列表第一个点。出生点不会自动成为目标；希望循环回到出生位置时，需将其
明确写入目标列表。目标 `h` 保持原有配置兼容，运动朝向仍由社会力积分后的速度决定。
运行中不热更新路线，编辑 YAML 后重新启动场景生效。

显式模式至少需要两个目标。每个目标名必须存在，基础列表中的名称和坐标不能重复。
展开后的相邻目标（包括循环连接处）距离必须严格大于 `2 * goal_radius`，防止同一区域内
反复切换。两点折返与两点循环等价；三点以上的折返只反向重复中间点，不连续重复端点。
显式模式自动使用循环队列，`cyclic_goals` 可省略；同时设置为 false 或字符串会报配置冲突。

坐标和半径必须有限，半径为正。启动预检使用实际 `world` 文件检查出生点与目标点：
`radius <= x <= width-radius`，`radius <= y <= height-radius`。错误会指出行人和目标。
半径检查使用与 ROS 消息传输一致的 float32 精度，避免边界值在预检通过后被内核拒绝。
未指定新模式的旧配置不应用新增的重复目标/区域重叠限制，保留原有队列语义。

纯 Python `compile_route()` 返回不可变的目标名、二维位置和循环标记。
`load_people(path, *, world_bounds=None)` 保持 `(Agents, models)` 返回值；离线与 launch
预检传入场地尺寸，直接调用时可以省略。现有 ROS 类型和话题不变，`Agent.goals[0]` 仍是
当前目标。折返队列包含重复的中间目标，所以检查访问顺序应比较完整队列，不能只按当前
目标坐标猜测折返方向。

## 默认布局

| 行人 | 模式 | g0 → g1 → g2 → g3，单位 m |
|---|---|---|
| person_1 | loop | (8,9) → (10,10.5) → (11,6.5) → (8,4.5) |
| person_2 | loop | (9.4,4.5) → (11.4,3.8) → (12,8) → (9.4,9) |
| person_3 | loop | (10.8,9) → (12.8,10.5) → (13.5,6) → (10.8,4.5) |
| person_4 | ping_pong | (12.2,4.5) → (14.2,5.5) → (15.5,8.5) → (17,10) |
| person_5 | ping_pong | (13.6,9) → (15.6,10.5) → (17,7.5) → (18.5,5) |
| person_6 | ping_pong | (15,4.5) → (17,3.8) → (19,6) → (20,9) |

![配置目标与出生位置；连线不是实际测量轨迹](../evidence/multigoal_20261010/routes.png)

路线局部交叉，避让沿用原社会力模型。该模型不是任意布局下的无碰撞或最终到达保证；
修改目标坐标后需要重新验证。当前功能不包含随机选点、到点停留或复杂地图全局寻路。

## 验证

CPU 检查启动并清理专用服务，其服务名与正式后端隔离，不驱动机器人：

```bash
source scripts/env.sh
ROS_DOMAIN_ID=79 python scripts/validate_multi_sfm_routes_cpu.py \
  config/scenarios/two_robots_multi_sfm_6_multigoal_nav2.yaml \
  --output evidence/runs/multigoal/cpu_routes.json
```

CPU 用静止机器人验证 180 秒行人积分、完整访问顺序、至少两轮路线、速度/加速度/边界。
它还从 `backup-multigoal-20261010` 读取原加载器，对旧一人、六人配置分别运行 30 秒成对
计算，比较每步经 ROS 编解码后的字段完全相同（不比较 CDR 对齐填充字节）。
此检查依赖包含该标签的开发 Git 仓库。

Isaac 场景启动后，在相同 ROS 域的另一个终端运行只读检查：

```bash
source scripts/env.sh
python scripts/validate_multi_sfm_routes.py \
  config/scenarios/two_robots_multi_sfm_6_multigoal_nav2.yaml \
  --duration-sim 180 --duration-wall 1800 \
  --output evidence/runs/multigoal/isaac_routes_30min.json
```

检查在后端就绪后开始，从当前队列相位累计完整周期，要求前 180 秒仿真时间内每人至少
完成两轮，并持续健康运行至少 1800 秒墙钟时间。最大等待运行时间为 5400 秒，可通过
`--wall-timeout` 调整；不会以墙钟时间代替仿真时间。

脚本逐步比较目标队列与到达半径，缺步、乱序、跳点、提前切换、数值错误或后端换代均失败。
`actual_people` 与权威轨迹按时间戳对齐，要求显示误差 ≤0.25 m、人机实际净距 ≥0.10 m、
行人速度 ≤1 m/s。它只订阅数据，不发布运动指令。首个完整队列用于相位对齐，不能用于
证明探针启动前的访问顺序。

双机导航回归可在上述检查期间执行：

```bash
python scripts/benchmark_navigation.py \
  config/scenarios/two_robots_multi_sfm_6_multigoal_nav2.yaml \
  config/tasks/two_robot_lanes.yaml \
  --output evidence/runs/multigoal/navigation.json
```

路线报告记录源码文件摘要（包含新增未跟踪源码）、输入配置哈希、访问序列、切换时间、
周期计数与失败原因。验收期间修改 src/scripts/config 会使报告失败；文档可独立更新。
本次证据位于 [multigoal_20261010](../evidence/multigoal_20261010/README.md)，不沿用旧版
Isaac 验收结论，也不代表自动完成了正式发布包的全部发布门禁。

## 回退

只需恢复旧场景时，结束当前启动进程并重新启动旧六人场景；新代码仍兼容原配置。
完整代码和 build/install 回退见 [RECOVERY.md](RECOVERY.md) 的本次基线快照说明。
