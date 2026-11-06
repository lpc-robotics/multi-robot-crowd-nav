# SCARED 专项证据

实验日期2026-11-03；主说明见[专项设计文档](../../docs/SCARED_STIMULUS_20261103.md)。
实际通过轮的运行源码固定为5a82112；新增后台路线后未改心理SIS系数、阈值或运动核心。

| 记录 | 含义 |
|---|---|
| `actual_intrusive_passed/` | 压力组实际角色、心理、几何、事务及有限机器人协议验收 |
| `actual_considerate_passed/` | 同一源码、种子、人群与心理条件的礼让对照，未触发SCARED |
| `cpu_intrusive_layout02/` | 最终布局，持续注意压力组，协议结束后继续120秒仿真 |
| `cpu_considerate_layout02/` | 相同布局和心理条件的礼让对照，另继续120秒仿真 |
| `cpu_body_fov_layout02/` | 相同压力控制配原200°感知的未触发诊断；不要求展示成功 |
| `build_03.txt`、`offline_05.txt` | overlay重建、187项Python与32项C++检查通过 |
| `intrusive_figures/` | 压力组完整心理时间窗、模式及实际轨迹，包含恢复阶段 |
| `comparison_figures_final/` | 最终压力/礼让对比图，共用地图尺度并保留完整心理时间窗 |
| `matched_comparison.json` | 复核源码、个体有效参数、模型和时钟一致的描述性配对比较 |

`cpu_intrusive_01`只验证了早期协议/几何和免疫状态，其passed字段不能当作心理展示通过。
它及`cpu_intrusive_02/03`均未触发SCARED；02/03增加触发要求后判为失败。
`cpu_attention_04`证实持续注意假设下可自然累积恐惧；后续又检查并调整了背景路线。
带`final`但不带`layout02`的CPU记录是路线调整前的阶段记录，不替代最终布局证据。

`actual_intrusive_01_failed/`保留启动前空人群消息被误判的失败，并发现原新增背景路线相交；
`actual_intrusive_02_failed/`保留详细状态1Hz与错误0.6s超时不匹配造成的失败。
修正为既有示范节点使用的2秒详细状态契约，Bool健康心跳仍保留0.6秒门槛，
几何、显示误差、运动、同步和事务保护没有放宽。
首轮`focused_tests_01.txt`是调用测试时覆盖了PYTHONPATH，导致安装消息不可见的失败；
完整环境下的后续离线回归通过。所有原始失败报告均保留。

完整JSONL和CSV在Git忽略的`evidence/runs/scared_stimulus_20261103/`。
各轮`trace_inventory.json`记录路径、字节数和SHA-256；实际轮的source_manifest.json
包含精确源码、心理配置、个体覆盖、实验台账及刺激文件摘要。
潜变量与实际可观测轨迹分开导出；图中的c/f/s不能当作真人心理测量。
最终导出为`intrusive_export/`、`considerate_export/`，各自CSV哈希在通过目录的
`export_manifest.json`中。`make_matched_comparison.py`可由归档的两个报告包重新生成比较，
输出使用独占创建以保护原记录。正式绘图工具为`scripts/plot_scared_stimulus.py`；
`candidates/`保留图形布局修正前的代码，与早期图片记录的处理器哈希对应。
5/6号行人在远处，不是同暴露个体类型对照；本展示不估计真实恐惧或互动的发生比例。
