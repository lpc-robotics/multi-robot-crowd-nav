# 发布与恢复

开发目录是 Git 仓库，稳定工作区只作为 underlay。`baseline/stable_underlay.json` 保存
11 个嵌套仓库的提交、分支和完整工作树状态，以及原 DWB、DWB 0.8、MPC 入口和关键
安装文件哈希。任何启动和发布前都运行 `scripts/verify_baseline.py`；失败时停止并重新
核查，不能覆盖用户修改或更新基线来绕过错误。

发布脚本只接受干净提交，将源码快照、配置、文档、工具和一个按最终绝对路径构建的
overlay 写入：

```bash
scripts/create_release.sh
```

最终入口固定指向具体 release ID，不使用可移动的 `latest`。发布目录写保护并包含
`RELEASE.json` 和 `SHA256SUMS`。其 overlay 直接构建到最终路径，因此把
`/home/lpc/workspace/arena5_multi_ws` 临时改名后仍能运行。

恢复原平台时先运行多机 release 的 `scripts/cleanup.sh`，打开一个未 source 多机 overlay
的新终端，执行稳定工作区原入口。无需卸载或重建 underlay：

```bash
cd /home/lpc/workspace/arena5_ws
source scripts/env.sh
scripts/run_six_behaviors.sh
```

移除主入口只影响多机增量；已有三个单机入口和既有发布包不在多机发布过程中修改。

## 2026-10-07 近距离/FOV 更新的完整回退点

修改前工作树干净，原分支 `feature/multi-sfm-pedestrians`，HEAD 为
`f330dfcd29c6a1f711e3f091798e36852fac742d`；基线标签为
`backup-near-fov-20261007`，更新在 `feature/near-fov-decoupling` 分支。

完整备份位于：

```text
/home/lpc/bk/arena5_multi_backups/20261007T024151Z-before-near-fov
```

`workspace.tar` 包含整个 arena5_multi_ws，包括 `.git`、隐藏目录、忽略文件、
build/install、日志与实验证据；使用 pax 格式保存权限、所有者、文件时间、符号链接、
ACL 和扩展属性。修改前已确认源目录在备份期间不变，并在 `verification/arena5_multi_ws`
解包后完成 9,629 个条目的文件哈希/大小、权限、UID/GID、文件及链接 mtime、链接目标
和目录集合比对。目录时间、ACL 和扩展属性由归档保存，不在 JSON 比对项中。
`verification.json` 和 `SHA256SUMS` 保存验证结果及归档摘要。首次未通过验证的
`20261007T024119Z-before-near-fov` 快照不可用于恢复。

只查看或回退代码时，可在干净工作树使用 `git switch --detach backup-near-fov-20261007`；
此操作不会回退忽略文件或 build/install，需要重新构建。要完整恢复原工作空间，
在关闭使用此工作空间的进程后，从未加载其 overlay 的新终端操作，先保留更新后的
整个目录，再恢复到原绝对路径，以保持安装目录中的绝对路径有效：

```bash
backup=/home/lpc/bk/arena5_multi_backups/20261007T024151Z-before-near-fov
cd "$backup"
sha256sum -c SHA256SUMS
# 在移动当前工作区前，把新增的只读校验工具保留到 /tmp。
cp /home/lpc/workspace/arena5_multi_ws/scripts/verify_workspace_snapshot.py /tmp/verify_arena5_snapshot.py
cd /home/lpc/workspace
mv arena5_multi_ws "arena5_multi_ws.after-near-fov.$(date +%Y%m%dT%H%M%S)"
tar --acls --xattrs --same-permissions -xpf "$backup/workspace.tar" -C /home/lpc/workspace
python /tmp/verify_arena5_snapshot.py /home/lpc/workspace/arena5_multi_ws "$backup/manifest.json"
git --no-optional-locks -C /home/lpc/workspace/arena5_multi_ws rev-parse HEAD
git --no-optional-locks -C /home/lpc/workspace/arena5_multi_ws status --porcelain=v1 --untracked-files=all
```

校验应返回 `passed: true`，HEAD 应为上述 f330dfc 提交，工作树状态为空。
恢复后的 `.git` 来自完整快照，新增标签和开发分支保留在已移走的更新目录中。
整个恢复过程不修改稳定 underlay 或既有发布包。当前改动的 CPU/ROS 验证记录在
`evidence/near_fov_20261007/`；旧版 Isaac 验收记录不作为新源码的验收证明。
