# V2 三个专项场景证据（2026-11-05）

所有失败轮次保留，不能用局部行为成功替代完整运行通过。原始高频轨迹保存在工作空间忽略目录 `evidence/runs/v2_specialists_20261105/`，各紧凑报告附 SHA-256 清单；源码标签不包含这些大文件。

- `build_01.txt`：9包 overlay 构建。
- `offline_04.txt`：196项 Python、32项 C++及配置/旧版检查通过。
- `legacy_cpu_report.json`：旧 V2 单人实际C++积分闭环通过，新增开关默认关闭。
- `playful_cpu_02_report.json`：较远占位点、较慢机器人得到8秒核心占位；规定机器人运动，不是Isaac证据。
- `registry_04.json`：138项台账、80项心理配置字段、83项未来拟合候选；无已拟合参数。
- `legacy_config_preservation.json`：旧61个配置（排除扩展的主参数台账）逐字保持。
- `config_invariants_final.json`：有效全局心理参数不变；两名玩闹者只覆盖7项交互工程参数；免疫/静态路线任务相同，仅类型不同。
- `playful_01_failed`：目标个人记忆丢失，人工提前结束，无完整验收结果。
- `playful_02_failed`：两人实际8秒占位，但后续健康超时，整轮失败。
- `playful_03_failed`：完整导航及几何通过，但14号在0.04737m/s被0.05m/s规则视作停止，未到位退出；专项失败。
- `immune_01_failed`、`immune_02_failed`：零心理量及几何检查不代表完整通过；健康/同步间隔超时。
- `passed/static`：静态障碍响应实际验收通过，所有潜变量为零，静态排斥非零、机器人社会力为零。

旧检查中 `config_raw_comparison_initial.json` 按原始YAML比较，因模板省略默认 `observation_display_margin_s=0.15` 得到不一致；`config_invariants.json` 和最终版改为有效参数比较，确认该值未改变。保留这个检查修正过程。

初次静态通过在GPU1、源码5c10e51；后续重跑在GPU0、源码8b51642。后者只新增玩闹专项低速判定覆盖与测试，不改变免疫/静态模型实现或配置。环境不同，不能把两条轨迹差异全部解释为个体类型因果效应；速度不变性有独立C++固定输入测试。

健康消息时序超限尚未确证根因；较短的专项通过不等于长时间稳定性验证。保护阈值没有放宽，其他用户的进程没有修改。

最终补充：
- `passed/playful`：GPU0，运行源码8b51642，完整两轮实际通过，每人实际占位7.95秒。
- `immune_03_failed`：完整两轮导航成功，但健康消息超时，仍失败。免疫专项没有完整运行通过的结论。
- `offline_outcomes_final.txt`：203项Python、32项C++；新record策略在真实探针检查方法中验证低净距、重叠及恢复保留。此前实际运行报告不回写。
- `docs/CROWD_RESPONSE_ACCEPTANCE_20261105.md`：用户要求的验收分层；新增脚本 `interaction_outcomes.py` 只影响观测/报告，不修改运动或心理模型。
