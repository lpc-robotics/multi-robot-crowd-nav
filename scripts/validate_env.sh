#!/usr/bin/env bash
# Repeated activation must preserve the caller's ROS discovery boundary.
set -Eeuo pipefail
validation_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for validation_localhost in 0 1; do
    (
        export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY="$validation_localhost"
        source "$validation_root/scripts/env.sh" >/dev/null
        source "$validation_root/scripts/env.sh" >/dev/null
        [[ "$ROS_DOMAIN_ID" == 89 && "$ROS_LOCALHOST_ONLY" == "$validation_localhost" ]]
    )
done
echo "MULTIROBOT_ENV_REENTRY_OK"
