# Source import

Source workspace: `/home/lpc/workspace/arena5_multi_ws`.
Current source branch: `feature/near-fov-decoupling`.
Current source commit: `cd0ab0713abd3b11e4a49d0b39ad09388c8f8706`.
Last source synchronization: 2026-10-07 (Asia/Shanghai).
Initial snapshot: `f330dfcd29c6a1f711e3f091798e36852fac742d`, 2026-09-30.

This independent repository publishes the tracked core source from that commit:
all nine ROS packages, configuration, tests, build/run/validation scripts,
baseline manifests, documentation, and compact historical evidence.

The README is rewritten for this repository. Repository contribution rules and
additional generated-file ignore patterns are added. All imported `src/`,
`scripts/`, and `config/` files retain their bytes and executable modes, except
seven generated notebook checkpoint files excluded from version control.

This repository starts with one owner-authored snapshot commit. Subsequent
source updates are cherry-picked onto that history; the independent development
ancestry remains in the original archive. Original source commit references in
historical evidence refer to that archive, not this repository.

Generated build/install trees, logs, NvStreamer traces, caches, binary releases,
full backups, external environments and NVIDIA installations are excluded.
The stable Arena5 underlay remains an external dependency. Historical validation
claims remain tied to their original source hashes and scenario scopes.

## 2026-10-07 update

Imported source commits (all authored by the repository owner):

| Original source commit | Change |
|---|---|
| `0d31e7c` | Decouple physical near repulsion from psychology modifiers |
| `45dbbad` | Configurable pair-wise human–robot social-force FOV |
| `cd0ab07` | Assessment, validation evidence, and complete recovery documentation |

All tracked source, configuration, scripts, documentation, and evidence match
the current source tip except this repository's README, ignore rules and added
publication metadata. The untracked `command.txt` is excluded as a command
transcript. The new validation `.log` files are compact committed test/build
records; generated runtime logs remain excluded.

The evidence manifest includes an installed server binary hash from the
original validation host. That binary is not committed; rebuild it locally.
The original baseline tag is not imported. For the force-scan replay in this
repository, pass `--baseline-ref 5c0bfe86b7d9254f09b9ea92535b924f25e5d2a3`.
Its core source matches the original `f330dfc` baseline. Historical backup
paths and restore records in `docs/RECOVERY.md` remain references to the original
development host and are not a full-workspace backup bundled in this repository.

Publication rechecks in this independent repository: 7 overlay packages rebuilt,
30 underlay checks, 72 scenario checks, 8 HuNav configuration checks, 40 Python
tests, 21 C++ tests, 5 LightSFM file hashes, and 120 CPU trajectory trials passed.
The 303 unchanged imported files matched the source tip in content and mode;
34 committed near/FOV evidence hashes matched their manifest. No new Isaac/GPU
simulation was run for this synchronization.
