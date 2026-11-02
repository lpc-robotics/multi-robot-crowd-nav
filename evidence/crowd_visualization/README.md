# 可视化开发验收记录

运行源码提交：`3fb489300df738372661f1ba5ddee384a92a3c88`。
开发前最新 V2：`b5d931cc01af2ae6e7ba9c33685a2afaef2a80d3`，
保护标签 `backup/pre-crowd-viz-20261029`。完整归档位于
`/home/lpc/workspace/arena5_multi_backups/20261029-before-crowd-viz`。

最新汇总以 [validation.json](validation.json) 为准。不能将不同提交的通过结果
合并成“最终版本全部通过”。所有失败记录保留，未放宽原有安全阈值。

| 检查 | 证据与边界 |
| --- | --- |
| 构建与回归 | 9 包构建，171 项 Python 测试通过；32 项原 C++ 测试通过，运动核心未改动 |
| V2 恢复演练 | [rollback_rehearsal.json](rollback_rehearsal.json)：独立目录恢复最新 V2、全新构建及 60 秒实际运行通过；未原地覆盖开发目录 |
| 无心理模型兼容 | [noop_six_fixed/visualization.json](noop_six_fixed/visualization.json)：6 人实际位置与 N/A 标签；无虚构心理样本 |
| V1 兼容 | [v1_eight/visualization.json](v1_eight/visualization.json)：8 人、S 不可用、实际几何与显示输出通过 |
| Isaac 图形 | [isaac_fourteen_ui.png](isaac_fourteen_ui.png) 为最终源码实际 UI 截图；单人刺激箭头见 [isaac_ui_fixed.png](isaac_ui_fixed.png) |
| 叠加层更新耗时 | [final_serial_overlay_metrics.json](final_serial_overlay_metrics.json)：只计 SceneView 对象更新，不含完整 ROS 反序列化、GPU 渲染和整帧时间 |
| Foxglove 数据链路 | [websocket.json](websocket.json)：实际 WebSocket 接收并解码 6 个话题、14 人数据；不是桌面端面板渲染验收 |
| 仿真时间录放 | [recording_sim_time.json](recording_sim_time.json)、[replay_sim_time.json](replay_sim_time.json)：137 个实际样本的 bag/header 时间差不超过 0.05 秒，独立域回放时钟与 header 时间域一致；客户端画面仍未实测 |
| 手动录包与回放 | [replay.json](replay.json)、[recording_info.txt](recording_info.txt)：11 话题录包，独立 ROS 域回放通过 |

`longrun_batched` 是早期 `8fab9bb` 的 30 分钟功能通过记录，但该版本的
可视化 CPU 开销未达到目标，不能用它代替最终版验收。
`final_longrun`（`cfaf91a`）因 lidar_stale 中止；`final_serial_longrun`
（`3fb4893`）因时钟暂停保护引起 unhealthy 中止。
`perf_off` 在关闭显示时发生 post_spawn_odometry_timeout，未取得有效性能样本；
当时汇总脚本也因零时长除法退出，已修正。失败原因之间尚无已证明的因果关系。

性能对照使用相同 14 人场景、静止机器人、相同 GPU 与端口，排除前 60 秒
预热，不连接桌面客户端。四种组合中均保留原 Foxglove bridge 和 WebRTC
服务以便控制变量；Isaac 开关隐含启用显示节点，因此无客户端时 Isaac-only
与 both 的服务端配置等价。百分比以单个 CPU 核为 100%，不是整机占用百分比。

最终版补测见 [performance_matrix.json](performance_matrix.json)：

| 组合 | 有效时长 | 显示节点平均 CPU | 结果 |
| --- | --- | --- | --- |
| 关闭显示 | 0 | 无有效样本 | 启动保护超时 |
| 仅 Foxglove 数据 | 0 | 无有效样本 | 启动保护超时 |
| Isaac 显示 | 600 秒 | 单核的 7.57% | 健康与几何验收通过 |
| 双端启用 | 600 秒 | 单核的 7.59% | 健康与几何验收通过 |

两个有效组合各 513 个性能样本，显示节点 CPU 低于 10% 目标。关闭显示的
基线不可用，因此无法计算 RTF 下降比例，四组合整体验收不通过。
最终源码的 30 分钟移动机器人验收也未通过；不能用这两次静止机器人
10 分钟通过代替。原因仍未定位，不据此认定故障由显示造成或与显示无关。

高频 JSONL、运行目录及 rosbag 保留在本机，不放入 Git。原始文件清单
与 SHA-256 保存为 `raw_artifacts.json`。失败尝试也保留以便追溯。
`tools/` 中保存本次检查使用的辅助脚本；固定输出目录拒绝覆盖，复测应使用新目录。

Foxglove 桌面端具体版本、布局导入/渲染及 Windows WebRTC 客户端仍未实测。
完整使用及恢复步骤见 [开发说明](../../docs/CROWD_VISUALIZATION.md)。
