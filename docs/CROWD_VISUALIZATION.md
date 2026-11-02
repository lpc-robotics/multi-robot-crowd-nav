# Crowd visualization

Read-only display for Isaac Sim 5.1 and Foxglove. Both switches default off.
The psychology model, C++ motion core, and existing scene parameters are unchanged.

## Start

```bash
source scripts/env.sh
./scripts/build.sh
./scripts/run_multirobot.sh config/scenarios/two_robots_multi_sfm_14_multigoal_psychology_v2_nav2.yaml crowd_viz:=true isaac_overlay:=true auto_demo:=true
# Foxglove only: crowd_viz:=true isaac_overlay:=false
# Original execution: crowd_viz:=false isaac_overlay:=false
ros2 param set /crowd_visualizer focus_id 14
```

Import `config/foxglove/crowd_visualization.json`. Use the existing Foxglove
connection (development default 8795; explicit delivery override 8765).
Set layout variable `person` to the pedestrian ID to change curves. This selects
curves independently of the ROS `focus_id` parameter controlling arrows.
`/multirobot/viz/status` reports the arrow focus and freshness.

Actual animation pose/velocity is published at at most 10 Hz; model states,
stimuli and markers at most 5 Hz wall time. Repeated source timestamps do not
produce new curve samples. Actual velocity is the existing Character Graph
finite difference, not the core continuous velocity. Core and actual samples
retain their own simulation timestamps; no interpolation or exact synchronization
is claimed. Psychology is a simulated latent activation, not measured emotion.
Stimulus samples use input time; state/core samples use integration-end time.

Markers use actual positions. Up to two robot and three human arrows point into
the selected person. Blue/red represents dominant curiosity/fear channel.
Human strength is weight times cue; robot strength is visibility times novelty
or risk. Robot cues are not additive causal contributions. Display threshold
0.02 never alters the model. Remembered targets are not current visual stimuli.
Missing data is not zero activation. V1 surprise has `has_surprise=false`; filter
surprise curves by this field when using V1. One second without progressing input
marks it stale; arrows disappear. On epoch/time rollback caches are cleared.

The Isaac module owns a SceneView containing text and line arrowheads. It does
not create USD geometry, change materials, or clear other modules' debug lines.
Its subscription and graphics resources are released on shutdown. The first
version requires an active viewport; unavailable extensions disable this layer.

## Optional recording and playback

No recording starts automatically. In a sourced terminal:

```bash
ros2 bag record --use-sim-time -o /tmp/crowd-viz-trial /multirobot/viz/actual /multirobot/viz/core /multirobot/viz/psychology /multirobot/viz/stimulus /multirobot/viz/markers /multirobot/viz/status /map /tf /tf_static /robot_1/odom /robot_2/odom
```

The visualization outputs use reliable bounded queues. Standard rosbag QoS
adaptation supports them. `--use-sim-time` waits for `/clock` and stores bag times
in the same simulation time domain as the header stamps. Use a fresh terminal and an unused ROS domain for
playback; do not replay clocks/TF into a live experiment:

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=89
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8796 -p address:=127.0.0.1 -p use_sim_time:=true
# In another terminal with the same environment/domain:
ros2 bag play /tmp/crowd-viz-trial --clock
```

Use simulation header timestamps in plots. Display sampling can miss short
transitions; original psychology logs remain the complete event audit source.
An older bag recorded without `--use-sim-time` may mix wall-clock bag times and
simulation header stamps; successful message replay alone does not verify the
client's visible plot time range.
For offline static-transform/map replay, keep rosbag's recorded durability QoS.

## Protected V2 and recovery

Protected HEAD: b5d931cc01af2ae6e7ba9c33685a2afaef2a80d3.
Backup: `/home/lpc/workspace/arena5_multi_backups/20261029-before-crowd-viz`.
The verified archive contains the complete pre-development workspace; it does
not contain external Conda environments or underlay link targets.

Before recovery stop this workspace's processes, preserve visualization changes
and new evidence, and use a fresh terminal. Check SHA256SUMS in the backup.
For source recovery switch to `backup/pre-crowd-viz-20261029`; move build/install
aside rather than reusing visualization-generated interfaces, then build again.
For full recovery preserve the current workspace under another name, extract
workspace.tar into /home/lpc/workspace (original absolute path), and run the
snapshot verifier against manifest.json. Never overwrite the only copy of new
work or delete the verified backup. Existing V2 validation tags remain unchanged.

## Validation status

See `evidence/crowd_visualization/validation.json` for checks actually executed.
GPU/WebRTC, Foxglove client layout rendering, 30-minute performance acceptance
and runtime rollback must not be inferred from CPU or build tests.

2026-10-30：功能源码 `3fb4893`，9 包构建、171 项 Python 测试及 32 项原 C++
测试通过；六人 noop、八人 V1 与独立 V2 恢复演练通过。14 人静止机器人场景
的 Isaac/双端组合各通过 10 分钟，显示节点平均 CPU 分别为单核的 7.57%/7.59%。
SceneView 更新 P95 上界为 0.4 ms，不代表完整仿真帧耗时。

**尚未全部验收通过**：最终源码的 30 分钟移动机器人检查中止；关闭显示的
性能基线也未就绪，因此不能确认 RTF 下降比例。Foxglove 桌面端布局渲染和
Windows WebRTC 客户端未实测。详见
[完整验收记录](../evidence/crowd_visualization/README.md)，不要将历史版本的
长时通过记录当作当前版本通过。

## Long-run validation command

Run in a fresh terminal with isolated domain/ports and a free GPU. The existing
experiment driver owns and cleans up its processes, including failures.

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=89 ARENA_WEBRTC_SIGNAL_PORT=49139 ARENA_WEBRTC_MEDIA_PORT=48039 ARENA_FOXGLOVE_PORT=8799
python scripts/validate_psychology_v2_experiment.py config/scenarios/two_robots_multi_sfm_14_multigoal_psychology_v2_nav2.yaml --output-dir evidence/crowd_visualization/longrun --duration-wall 1800 --crowd-viz --isaac-overlay --livestream --foxglove
```

The visual overlay appears in UI-composited screenshots; camera render-product
captures do not contain it. With overlay enabled, ARENA_SCREENSHOT captures the
window compositor. The repaired single-person UI screenshot is
`evidence/crowd_visualization/isaac_ui_fixed.png`.

## 中文使用与恢复说明

数据链路为：原心理状态/实际行人姿态/机器人里程计 → 独立
`crowd_visualizer` 节点 → Foxglove 与 Isaac SceneView。可视化节点只订阅
原数据，只向 `/multirobot/viz/*` 发布，不回写心理状态、控制指令或场景。
Isaac 仅增加一个可选模块，启动失败会记录告警并继续原仿真。

| 话题后缀 | 含义 |
| --- | --- |
| `actual` | Character Graph 实际位置、朝向及有限差分速度 |
| `core` | 运动核心提交的位置与速度，供与实际动画区分 |
| `psychology` | 行人 ID、行为模式、阶段、目标及 C/F/S 心理激活 |
| `stimulus` | 直接刺激与人际传播输入，使用积分前输入时间戳 |
| `markers` | 所有人标签、位置，以及焦点行人的最多 5 条刺激箭头 |
| `status` | 焦点 ID、数据新鲜度及暂停/过期状态 |

显示颜色：灰色常态、蓝色好奇、红色恐惧、黄色惊讶、紫色主动挡路。
箭头由刺激来源指向接收行人；最多保留 2 条机器人刺激、3 条人际刺激，
选择强度较高的输入。无箭头不代表完全没有刺激，也可能低于显示阈值。
箭头表示模型当前输入，不是独立测量或因果归因结果。
旧的 `noop` 场景没有心理事务，显示 N/A 标签并保留实际运动数据；
不会为它补出 C/F/S 曲线。V1 的 S 同样标为不可用。

为控制开销，实际运动最多 10 Hz，心理量和图形最多 5 Hz；只处理最新数据，
使用有限队列并复用图形对象。默认不写新日志、不录包。可视化采样可能略过很短
的状态变化，研究审计仍使用原始心理日志。

默认不启用可视化。原有启动命令保持原样；需要显示时追加
`crowd_viz:=true isaac_overlay:=true`。只看 Foxglove 时只追加
`crowd_viz:=true`。导入 `config/foxglove/crowd_visualization.json` 后，修改
布局变量 `person` 选择曲线对象；箭头对象用
`ros2 param set /crowd_visualizer focus_id 14` 切换。

Foxglove 桌面端导入布局后，以 Foxglove WebSocket 连接仿真主机的
`ws://<主机地址>:8795`（若启动时覆盖端口，以覆盖值为准）。确认 3D 面板
显示 `/multirobot/viz/markers`，曲线按 Header stamp 绘制，状态面板的两个
fresh 字段为 true。桌面端版本未提供，实际布局导入与渲染仍需在该客户端核验。
布局使用原生面板，无需安装扩展。Header stamp 的含义见
[Foxglove 官方 Plot 文档](https://docs.foxglove.dev/docs/visualization/panels/plot)。

实际位置/速度与模型心理量是不同来源。C/F/S 是仿真内部激活，不是真人测量。
直接刺激与传播输入使用输入时刻，心理状态使用提交时刻；不要把两者误认为同步测量。
当前实际长时验收与待完成项目均记录在
`evidence/crowd_visualization/validation.json`，不同代码提交的证据不能混用。

### 恢复到开发前最新 V2 源码

先停止使用当前工作区的进程，保存当前修改和新证据，并打开没有加载 overlay 的新终端。
确认工作树修改已保存后执行。以下操作保留新构建产物，不删除它们：

```bash
cd /home/lpc/workspace/arena5_multi_ws
git status --short
saved_build="runtime/before-v2-restore-$(date +%Y%m%dT%H%M%S)"
mkdir -p "$saved_build"
mv build "$saved_build/build"
mv install "$saved_build/install"
git switch --detach backup/pre-crowd-viz-20261029
./scripts/build.sh
```

恢复点必须为 `b5d931cc01af2ae6e7ba9c33685a2afaef2a80d3`，不是旧的 `f170bc7`。
恢复可视化开发时，保存本轮 V2 构建产物，切回 `feature/crowd-visualization` 并重新构建。
不要同时加载新旧 overlay。关闭显示开关仅停止显示功能，不等于源码已经恢复为旧版本。

### 完整恢复开发前工作区

先停止工作区进程；把校验脚本和本说明保存到工作区外。以下命令保留当前整个目录：

```bash
viz_backup=/home/lpc/workspace/arena5_multi_backups/20261029-before-crowd-viz
(cd "$viz_backup" && sha256sum -c SHA256SUMS)
cp /home/lpc/workspace/arena5_multi_ws/scripts/verify_workspace_snapshot.py /tmp/verify_before_crowd_viz.py
cd /home/lpc/workspace
mv arena5_multi_ws "arena5_multi_ws.after-viz.$(date +%Y%m%dT%H%M%S)"
tar --acls --xattrs --same-permissions -xpf "$viz_backup/workspace.tar" -C /home/lpc/workspace
python3 /tmp/verify_before_crowd_viz.py /home/lpc/workspace/arena5_multi_ws "$viz_backup/manifest.json"
```

完整归档包含源码、Git、build/install 和归档前证据；不包含外部 Conda 环境与符号链接目标。
必须恢复到原绝对路径。新开发成果保留在移走的目录中，不能覆盖或删除唯一备份。
