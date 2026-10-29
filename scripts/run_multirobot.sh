#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
source "$SCRIPT_DIR/process_group.sh"
SCENARIO="${1:-$ARENA_MULTI_WS/config/scenarios/two_robots_dijkstra.yaml}"
if [[ ! -f "$SCENARIO" ]]; then
    echo "Scenario file does not exist: $SCENARIO" >&2
    exit 2
fi
shift $(( $# > 0 ? 1 : 0 ))
"$SCRIPT_DIR/verify_baseline.py"
if [[ ! -f "$ARENA_MULTI_WS/install/local_setup.bash" ]]; then
    echo "Build the isolated overlay first: scripts/build.sh" >&2
    exit 1
fi
set +u
source "$ARENA_MULTI_WS/install/local_setup.bash"
set -u

python3 "$SCRIPT_DIR/check_active_run.py"
export ARENA_CPU_THREADS="${ARENA_CPU_THREADS:-8}"
if ! [[ "$ARENA_CPU_THREADS" =~ ^[1-9][0-9]*$ ]] || (( ARENA_CPU_THREADS > 256 )); then
    echo "ARENA_CPU_THREADS must be an integer in [1, 256]." >&2
    exit 2
fi
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-$ARENA_CPU_THREADS}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-$ARENA_CPU_THREADS}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-$ARENA_CPU_THREADS}"

if ! command -v nvidia-smi >/dev/null || ! nvidia-smi --query-gpu=index,memory.free --format=csv,noheader,nounits >/dev/null 2>&1; then
    echo "NVIDIA runtime is unavailable; Isaac validation cannot start in this environment." >&2
    exit 1
fi
if [[ -z "${GPU_ID:-}" ]]; then
    GPU_ID="$(nvidia-smi --query-gpu=index,memory.free --format=csv,noheader,nounits | awk -F, '{gsub(/ /,"",$1);gsub(/ /,"",$2);print $2,$1}' | sort -nr | awk 'NR==1{print $2}')"
fi
GPU_FREE_MIB="$(nvidia-smi --id="$GPU_ID" --query-gpu=memory.free --format=csv,noheader,nounits | tr -d ' ')"
MIN_FREE_MIB="${MULTIROBOT_MIN_FREE_MIB:-8192}"
if (( GPU_FREE_MIB < MIN_FREE_MIB )); then
    echo "GPU $GPU_ID has ${GPU_FREE_MIB} MiB free; ${MIN_FREE_MIB} MiB required." >&2
    exit 1
fi
ARENA_RESOLVED_CPU_AFFINITY="$(python3 "$SCRIPT_DIR/gpu_cpu_affinity.py" "$GPU_ID")"
taskset -pc "$ARENA_RESOLVED_CPU_AFFINITY" "$$" >/dev/null
for port in "$ARENA_WEBRTC_SIGNAL_PORT" "$ARENA_FOXGLOVE_PORT"; do
    if ss -ltn "sport = :$port" | tail -n +2 | grep -q .; then
        echo "TCP port is already in use: $port" >&2
        exit 1
    fi
done
if ss -lun "sport = :$ARENA_WEBRTC_MEDIA_PORT" | tail -n +2 | grep -q .; then
    echo "UDP port is already in use: $ARENA_WEBRTC_MEDIA_PORT" >&2
    exit 1
fi

export GPU_ID CUDA_VISIBLE_DEVICES="$GPU_ID" ARENA_RENDER_GPU="$GPU_ID" ARENA_INTERNAL_GPU=0
export ARENA_NORMALIZED_SCAN=true ARENA_DEPTH_CLEARING=false ARENA_IDEAL_CHASSIS=true
RUN_ID="$(date +%Y%m%d_%H%M%S)_$(basename "$SCENARIO" .yaml)_gpu${GPU_ID}"
ARENA_MULTI_LOG_ROOT="${ARENA_MULTI_LOG_ROOT:-$ARENA_MULTI_STATE_ROOT/logs/runs}"
export ARENA_MULTI_RUN_DIR="$ARENA_MULTI_LOG_ROOT/$RUN_ID"
mkdir -p "$ARENA_MULTI_RUN_DIR/ros"
export ROS_LOG_DIR="$ARENA_MULTI_RUN_DIR/ros"
export ARENA_KIT_LOG="$ARENA_MULTI_RUN_DIR/isaac_kit.log"
cp "$SCENARIO" "$ARENA_MULTI_RUN_DIR/scenario.yaml"
BRINGUP_SHARE="$(ros2 pkg prefix --share arena_multi_bringup)"
cp "$BRINGUP_SHARE/config/nav2_multirobot.yaml" "$ARENA_MULTI_RUN_DIR/nav2_multirobot.yaml"
cp "$ARENA_MULTI_WS/config/world/arena.yaml" "$ARENA_MULTI_RUN_DIR/world.yaml"
cp "$ARENA_MULTI_WS/config/maps/map.yaml" "$ARENA_MULTI_RUN_DIR/map.yaml"
find "$ARENA_MULTI_WS/config" -type f -print0 | sort -z | xargs -0 sha256sum \
    > "$ARENA_MULTI_RUN_DIR/config_sha256.txt"
if git -C "$ARENA_MULTI_WS" rev-parse HEAD >/dev/null 2>&1; then
    SOURCE_COMMIT="$(git -C "$ARENA_MULTI_WS" rev-parse HEAD)"
elif [[ -f "$ARENA_MULTI_WS/RELEASE.json" ]]; then
    SOURCE_COMMIT="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["source_commit"])' "$ARENA_MULTI_WS/RELEASE.json")"
else
    SOURCE_COMMIT="unknown"
fi
{
    echo "source_commit=$SOURCE_COMMIT"
    echo "scenario=$(readlink -f "$SCENARIO")"
    echo "scenario_sha256=$(sha256sum "$SCENARIO" | awk '{print $1}')"
    echo "gpu_id=$GPU_ID"
    echo "gpu_free_mib=$GPU_FREE_MIB"
    echo "ros_domain_id=$ROS_DOMAIN_ID"
    echo "isaac_cpu_threads=$ARENA_CPU_THREADS"
    echo "cpu_affinity=$(awk '/Cpus_allowed_list/ {print $2}' /proc/self/status)"
    echo "random_seed=$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1])).get("random_seed", 1))' "$SCENARIO")"
} > "$ARENA_MULTI_RUN_DIR/runtime_manifest.txt"
python3 "$SCRIPT_DIR/capture_psychology_v2_manifest.py" "$SCENARIO" "$ARENA_MULTI_RUN_DIR"

echo "Starting Arena5 multi-robot scenario $(basename "$SCENARIO") on GPU $GPU_ID"
echo "Logs: $ARENA_MULTI_RUN_DIR"
echo "Isaac CPU worker limit: $ARENA_CPU_THREADS"
setsid ros2 launch arena_multi_bringup multirobot.launch.py \
    scenario:="$(readlink -f "$SCENARIO")" \
    livestream:="${LIVESTREAM:-true}" \
    foxglove:="${FOXGLOVE:-true}" \
    "$@" &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$ARENA_MULTI_RUN_DIR/launch.pid"
finish_launch() {
    local status=$?
    trap - EXIT INT TERM
    # ros2 launch may exit while children remain; check the entire group.
    arena_stop_group "$LAUNCH_PID" || status=1
    wait "$LAUNCH_PID" 2>/dev/null || true
    echo "$status" > "$ARENA_MULTI_RUN_DIR/launch.exit_code"
}
trap finish_launch EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
set +e
wait "$LAUNCH_PID"
status=$?
set -e
exit "$status"
