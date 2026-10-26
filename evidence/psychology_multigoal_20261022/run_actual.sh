#!/usr/bin/env bash
set -Eeuo pipefail
cd /home/lpc/workspace/arena5_multi_ws
export ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 GPU_ID=1 LIVESTREAM=false FOXGLOVE=false
export ARENA_WEBRTC_SIGNAL_PORT=49142 ARENA_WEBRTC_MEDIA_PORT=48042 ARENA_FOXGLOVE_PORT=8798
export ARENA_MULTI_STATE_ROOT=$PWD/evidence/runs/psychology_multigoal_20261022/state
export ARENA_MULTI_LOG_ROOT=$PWD/evidence/runs/psychology_multigoal_20261022/actual
source scripts/env.sh >/dev/null
./scripts/run_multirobot.sh config/scenarios/two_robots_multi_sfm_8_multigoal_psychology_nav2.yaml auto_demo:=true > evidence/runs/psychology_multigoal_20261022/actual_launch.log 2>&1 &
scene_pid=$!
cleanup() {
  ./scripts/cleanup.sh > evidence/runs/psychology_multigoal_20261022/cleanup.log 2>&1 || true
  wait "$scene_pid" 2>/dev/null || true
}
trap cleanup EXIT
python scripts/validate_psychology_runtime.py config/scenarios/two_robots_multi_sfm_8_multigoal_psychology_nav2.yaml --duration-wall 600 --output evidence/runs/psychology_multigoal_20261022/actual.json > evidence/runs/psychology_multigoal_20261022/actual_probe.log 2>&1
