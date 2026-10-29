# V2 最终验收证据索引

实验开始于2026-10-28，2026-10-29完成整理。运行模型、核心、场景固定于
`7cffd0eddc4c55aa414338d609b5648c71d3d70d`。结论与解释边界见
[实际验收文档](../../docs/DYNAMIC_PSYCHOLOGY_V2_VALIDATION_20261028.md)。

| 目录/文件 | 用途 |
|---|---|
| `seed11_passed/`、`seed12_passed/`、`seed13_passed/` | 三个主场景种子的验收、导航、配置、轨迹哈希及最终导出清单 |
| `control_completed/acceptance.json` | 原通用探针报告，保留13号惊讶提前退出对应的未满足要求，仍为失败 |
| `control_completed/ablation_acceptance.json` | 禁用挡路对照的独立范围验收；原物理、同步与事务门槛不变 |
| `*/all_surprise_gaze_final.json` | 正常完成惊讶事件的实际停稳和朝向审计；提前退出另行保留 |
| `matched_comparison.json` | 匹配实验的逐导航段、速度、路径、停留及仿真历时比较，不作统计因果结论 |
| `*_figures/` | 全人群行为模式及13、14号的潜变量、轨迹图；PNG和SVG |
| `final_build_20261029.txt`、`final_offline_20261029.txt` | 最终overlay重建、161项Python和32项C++检查 |
| `final_registry_20261029.txt` | 137项参数、83项未来拟合候选，当前无已拟合参数 |
| `final_runtime_equivalence.json` | 冻结运行结束后的修改仅限离线数据、分析、校验及测试，并记录文件哈希 |
| `p4_review.json` | 审查发现、处理结果和阶段完成状态 |

失败试验保留在`*_failed/`和`*_record/`及相应标准输出文件中；
`faults_single_config_record/`是修正单人故障测试调用配置后的通过记录。
`candidates/`保留冻结运行期间使用的分析候选和历史修订，`SHA256.json`记录摘要；
当前正式工具位于工作区`scripts/`和`src/arena_multi_hunav/arena_multi_hunav/`。

高频轨迹与CSV位于被Git忽略的`evidence/runs/psychology_v2_20261028/`。
`trace_inventory.json`记录原始轨迹路径、大小与SHA-256；最终导出分别为
`seed11_export_final/`、`seed12_export_final/`、`seed13_export_final/`、
`control_export_final/`，各自输出哈希记录于`export_manifest_final.json`。
Git标签只保护源码和上述精简证据；迁移完整实验时必须同时保存这些原始数据目录。
