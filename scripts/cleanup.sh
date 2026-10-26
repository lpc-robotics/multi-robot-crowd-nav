#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARENA_MULTI_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/process_group.sh"
ARENA_MULTI_STATE_ROOT="${ARENA_MULTI_STATE_ROOT:-$ARENA_MULTI_WS}"
ARENA_MULTI_LOG_ROOT="${ARENA_MULTI_LOG_ROOT:-$ARENA_MULTI_STATE_ROOT/logs/runs}"
found=0
while IFS= read -r pid_file; do
    pid="$(tr -d '[:space:]' < "$pid_file")"
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    group_rows="$(ps -eo pid=,pgid=,cmd= | awk -v group="$pid" '$2 == group {print}')"
    [[ -n "$group_rows" ]] || continue
    if [[ "$group_rows" != *"arena_multi_bringup"* && "$group_rows" != *"run_isaacsim"* && "$group_rows" != *"$ARENA_MULTI_WS"* ]]; then
        echo "Refusing PGID $pid because it has no Arena multi process." >&2
        continue
    fi
    arena_stop_group "$pid"
    found=1
done < <(find "$ARENA_MULTI_LOG_ROOT" -type f -name launch.pid 2>/dev/null | sort)
if (( found == 0 )); then
    echo "No running Arena multi launch found."
fi
