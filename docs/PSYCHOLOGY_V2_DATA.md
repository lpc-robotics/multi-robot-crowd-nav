# V2 参数辨识与可观测数据接口

本版没有拟合优化器，也没有真人数据拟合结果。参数台账为
`config/psychology/parameter_registry_v2.yaml`；运行
`python scripts/validate_psychology_v2_registry.py` 检查覆盖和有效个体配置。
79项心理配置字段与相关运动、几何、时序及实验设置共137项。83项被列为未来候选，
候选资格不表示可在同一实验中同时辨识。合法范围来自代码约束，不是统计先验/置信区间。

`alpha`直接刺激、`mu`恢复、`beta`传播、行为阈值和运动系数应分阶段辨识。
先固定表达尺度再估计beta，避免乘积耦合；身体尺寸/坐标变换应实测，不能缩小包络改善拟合。
轨迹不能独立验证c/f/s代表真人心理；需要独立行为标注、刺激暴露和未触发对照。
冷却时间需要重复机会，固定两名玩闹者/每人一次的展示不能估计真实发生比例或冷却。
心理FOV、物理社会力FOV、基础期望速度、硬速度上限均为不同参数ID。
两类心理不响应者不参与SIS直接激活和传播；其解析配置中的心理系数在该类型下不生效，
不能利用这些人的零心理状态估计恢复或传播系数。个体类型必须随完整实验记录保存。
旧 `curiosity_stop_band` 实际仅用于停留验收容差，台账明确标为非运行死区。

V2启动时自动将源码哈希、原始配置、台账和解析后的逐人参数保存到运行目录
`source_manifest.json` 和 `inputs/`。源码未提交会记录dirty状态及全部源文件哈希；
不得在事后用当前代码冒充旧实验的来源。旧实验缺少这一清单时导出器明确拒绝，
既有原始证据仍保留。完整运行或失败运行都可以导出。

在已source工作区环境的终端运行：

```bash
python scripts/export_psychology_v2_data.py /绝对路径/运行目录 --experiment-id trial_001 --output-dir /绝对路径/新导出目录 --actual /绝对路径/验收.actual.jsonl
python scripts/validate_psychology_v2_data.py /绝对路径/新导出目录
```

`--actual` 可省略；省略意味着没有真实显示观测，不能当作实际几何验收。
输出目录必须不存在，防止覆盖实验。

| 文件 | 内容与语义 |
|---|---|
| observations.csv | 实验/个体/实体类型、原始纳秒时间、坐标系、x/y/vx/vy/body_yaw、来源、质量、速度来源、插值和缺失标记 |
| events.csv | 起止时间、标签、目标、标注来源、置信度、事件类型和原因，包含不触发/退出/超时/记录结束右删失 |
| latents.csv | 仿真独有的c/f/s、profile/mode/phase，单独保存，真人数据不需要该文件 |
| manifest.json | 运行来源快照、输入/输出哈希、采样来源、时钟对齐和划分政策 |

来源为 `core_committed`、`robot_observation`、`isaac_actual` 或未来 `measured`。
核心人位置使用提交时间，机器人插值观测使用请求输入时间，两者相差一次积分步长，
不得错误地贴同一时间戳。Isaac速度是实际图变换位置的有限差分，离散写入造成的峰值
不能冒充核心连续速度；核心速度/加速度限制单独验证。导出器不再增加插值。适配器已对机器人里程计进行时间重采样；原schema3日志未保存
逐条插值端点与精确采样命中标记，所以机器人行的 `interpolated=unknown`，速度来源
明确为 `odometry_timeline_resampled_world_frame`。不能将它当成未处理的原始里程计。
`interpolated`允许true/false/unknown；manifest明确米、米/秒、弧度、纳秒单位。
若提供latents.csv，校验器还检查潜变量范围、类别与逐人时间顺序。

真人数据可独立填写相同CSV并提供来源清单，不要求模型参数或c/f/s。
缺失数值留空，同时在 `missing_fields` 用分号列出字段；质量和速度来源必填。
真人标签使用标注者/协议来源，不能沿用 `simulator_model` 作为真实情绪标签。
模型导出标签置信度留空，不伪造心理后验概率。

未来训练/验证应同时按参与者与完整实验分组，不能把相邻帧分到两侧。
保留完整未触发和失败实验；调参后的配置另建版本，并重新执行几何与时序验证。

匹配对照使用 `two_robots_multi_sfm_14_multigoal_psychology_v2_control_nav2.yaml`，
仅关闭 `playful_enabled`。完成两轮自动演示后可运行
`python scripts/compare_psychology_v2_runs.py 主运行目录 对照运行目录 --output 新比较报告.json`。
工具检查源码/有效个体参数/场景一致性，再报告导航用时、互动通道内速度、停止时长、
横向偏移和路径长度差值。不会预设“挡路一定增加延误”，也不将单次仿真差值作为统计结论。

绘图入口：`python scripts/plot_psychology_v2.py 导出目录 --output-dir 新图目录 --agents 13 14`。
生成PNG/SVG，分别显示c/f/s、五种行为模式、实际/核心轨迹来源和全体人群的完整模式区间。
没有触发的行人仍保留，图中潜变量明确标为未拟合仿真激活，不能用于宣称真人心理测量。

对照验收不要求玩闹者完成全部惊讶观察；目标丢失/超时仍是需要保存的有效结果。
使用 `validate_psychology_v2_experiment.py` 运行对照时，显式传入
`--require-gaze --require-playful`（两个列表均为空），主场景默认要求仍保留。
结束后运行：

```bash
python scripts/validate_psychology_v2_control.py 验收报告.json 运行目录/source_manifest.json --output 新对照范围报告.json
```

该工具验证配置确已关闭挡路、无THREATENING、全部导航完成、几何/运动/时长及健康门槛；
不会豁免观测中断、RPC/事务失败或碰撞。原始探针报告保持不变，新报告列明不同验收范围。
比较工具还报告互动所在导航段及互动前的耗时差，区分仿真总历时与机器人目标耗时之和；
异步Nav2/Isaac的单次同种子比较不是逐位重放，不能据此作统计因果或发生率结论。
