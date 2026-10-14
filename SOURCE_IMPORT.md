# Source import

Source workspace: `/home/lpc/workspace/arena5_multi_ws`.
Current source branch: `feature/dynamic-psychology-v1`.
Current source commit: `b1829039da31357c5df0e9cd15728b1d383b6852`.
Last source synchronization: 2026-10-14 (Asia/Shanghai).
Initial snapshot: `f330dfcd29c6a1f711e3f091798e36852fac742d`, 2026-09-30.

This independent repository publishes the tracked core source from that commit:
all nine ROS packages, configuration, tests, build/run/validation scripts,
baseline manifests, documentation, and compact historical evidence.

The README is rewritten for this repository. Repository contribution rules and
additional generated-file ignore patterns are added. All imported `src/`,
`scripts/`, and `config/` files retain their bytes and executable modes, except
seven generated notebook checkpoint files excluded from version control.

This repository starts with one owner-authored snapshot commit. Subsequent
source updates are imported onto that history; the independent development
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
publication metadata. The then-untracked `command.txt` was excluded as a command
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

## 2026-10-14 update

Imported the tracked snapshot at `b1829039da31357c5df0e9cd15728b1d383b6852`, including:

| Original source commit | Change |
|---|---|
| `1301b05` | Preserve the validated multi-goal implementation and evidence |
| `e422603` | Robot-driven dual-SIS psychology, task-return avoidance and documentation |
| `b182903` | Regression evidence and 30-minute actual Isaac acceptance |

This update is a new owner-authored commit on the existing independent `main`
history. It does not merge the development repository's ancestry or alter that
workspace. All existing reachable commit authors and committers were audited.

419 imported files match the source snapshot byte-for-byte and retain their
executable modes. Repository-specific README, ignore rules, AGENTS.md and this
import record are retained or updated separately. Seven generated notebook
checkpoint files remain excluded. Seventeen newly imported `.log` files are
excluded from publication; the original copies and recorded hashes remain in
the development workspace. Existing historical evidence is preserved. The now
tracked `command.txt` is included with simple manual and automatic launch commands.

Publication rechecks in this repository passed: 7 overlay packages rebuilt,
116 Python tests, 29 C++ tests, 95 scenario checks, 8 HuNav configuration checks,
30 underlay checks, environment reentry and 5 LightSFM hashes. The six reports
referenced by the original psychology acceptance record match their recorded
hashes. See [publication.json](evidence/publication_20261014/publication.json)
for imported-file hashes and local build/validation log hashes.

No Isaac/GPU simulation was rerun for this publication. The original 30-minute
result belongs to the development workspace's recorded source hash, configuration
and runtime profile. Excluding notebook checkpoints changes the aggregate source
inventory hash even though the imported runtime code is identical. Do not rewrite
the historical acceptance record or rerun its finalizer merely to relabel this
publication as a new GPU-validated release. Absolute paths and original commit
IDs inside historical evidence refer to the development host; raw high-frequency
traces, build products, runtime logs and full backups are not bundled here.
