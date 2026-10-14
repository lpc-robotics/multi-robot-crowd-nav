#!/usr/bin/env bash
set -Eeuo pipefail
cd /home/lpc/workspace/arena5_multi_ws
export ROS_DOMAIN_ID=85 ROS_LOCALHOST_ONLY=1 GPU_ID=3 LIVESTREAM=false FOXGLOVE=false
export ARENA_WEBRTC_SIGNAL_PORT=49135 ARENA_WEBRTC_MEDIA_PORT=48035 ARENA_FOXGLOVE_PORT=8796
export ARENA_MULTI_STATE_ROOT=$PWD/runtime/psychology_validation
export ARENA_MULTI_LOG_ROOT=$PWD/logs/dynamic_psychology_20261013/verified_endurance
source scripts/env.sh >/dev/null
./scripts/run_multirobot.sh config/scenarios/two_robots_multi_sfm_6_multigoal_psychology_nav2.yaml auto_demo:=true > /tmp/arena-psychology-numa.log 2>&1 &
scene_pid=$!
python evidence/dynamic_psychology_20261012/validation/timing_monitor.py --output evidence/dynamic_psychology_20261012/validation/timing.json > /tmp/arena-psychology-timing.log 2>&1 &
monitor_pid=$!
cleanup() {
  ./scripts/cleanup.sh > /tmp/arena-psychology-numa-cleanup.log 2>&1 || true
  kill -INT "$monitor_pid" 2>/dev/null || true
  wait "$monitor_pid" 2>/dev/null || true
  wait "$scene_pid" 2>/dev/null || true
}
trap cleanup EXIT
python scripts/validate_psychology_runtime.py config/scenarios/two_robots_multi_sfm_6_multigoal_psychology_nav2.yaml --duration-wall 1800 --output evidence/dynamic_psychology_20261012/validation/isaac_six_people_30min.json > /tmp/arena-psychology-numa-probe.log 2>&1
