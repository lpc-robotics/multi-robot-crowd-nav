#!/usr/bin/env bash
# Only call this for a process group created or verified by this workspace.
arena_group_alive() {
    ps -eo pgid=,stat= | awk -v group="$1" \
        '$1 == group && $2 !~ /^Z/ {alive=1} END {exit !alive}'
}

arena_stop_group() {
    local group="$1" signal attempt
    for signal in INT TERM KILL; do
        arena_group_alive "$group" || return 0
        echo "Stopping Arena process group $group with SIG$signal"
        kill -"$signal" -- "-$group" 2>/dev/null || true
        for (( attempt=0; attempt<40; attempt++ )); do
            arena_group_alive "$group" || return 0
            sleep 0.25
        done
    done
    echo "Arena process group $group has not exited." >&2
    return 1
}
