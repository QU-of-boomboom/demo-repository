#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MISSION_PROFILE="$root/mission_profile_1drone_2cars.json"
exec "${PYTHON:-python3}" "$root/swarm_cooperation/swarm_cooperation/demo.py" \
  --mode none --rounds 1 --quiet --output "$root/.cache/cars/demo"
