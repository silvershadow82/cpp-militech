#!/usr/bin/env bash
# One SITL follow run with a MAVLink feed for QGroundControl, so the flight can be watched live.
#
#   coursework/tools/run_qgc_session.sh coursework/config/scenarios/circle.json
#
# Differs from run_sitl_scenario.sh in one way: SITL gets a second MAVLink serial pointed at
# QGroundControl's default UDP port. QGC picks the vehicle up on its own; nothing else is needed.
#
# Environment:
#   ARDUPILOT_DIR  ArduPilot checkout with SITL built
#   ARDUCOPTER     SITL binary (default $ARDUPILOT_DIR/build/sitl/bin/arducopter)
#   BUILD_DIR      CMake build with follow_app and follow_check_run (default build/coursework)
#   PYTHON         Python with pymavlink (default python3)
#   QGC_PORT       where QGroundControl listens (default 14550)
#   OUT_DIR        where logs go (default build/qgc-runs/<scenario>-<time>)
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
COURSEWORK=$(cd "$HERE/.." && pwd)
REPO=$(cd "$COURSEWORK/.." && pwd)
SCENARIO_ARG=${1:?usage: run_qgc_session.sh <scenario.json>}
SCENARIO=$(cd "$(dirname "$SCENARIO_ARG")" && pwd)/$(basename "$SCENARIO_ARG")
NAME=$(basename "$SCENARIO" .json)
ARDUPILOT_DIR=${ARDUPILOT_DIR:?set ARDUPILOT_DIR to an ArduPilot checkout with SITL built}
ARDUCOPTER=${ARDUCOPTER:-$ARDUPILOT_DIR/build/sitl/bin/arducopter}
BUILD_DIR=${BUILD_DIR:-$REPO/build/coursework}
PYTHON=${PYTHON:-python3}
QGC_PORT=${QGC_PORT:-14550}
OUT_DIR=${OUT_DIR:-$REPO/build/qgc-runs/$NAME-$(date +%Y%m%d-%H%M%S)}

[ -x "$ARDUCOPTER" ] || { echo "no SITL binary at $ARDUCOPTER" >&2; exit 2; }
for bin in follow_app follow_check_run; do
  [ -x "$BUILD_DIR/$bin" ] || { echo "no $bin in $BUILD_DIR: build it first" >&2; exit 2; }
done
"$PYTHON" -c 'import pymavlink' 2>/dev/null || { echo "$PYTHON cannot import pymavlink (try: make venv)" >&2; exit 2; }

# A stale SITL holding tcp:5760 is the usual failure here, and it is easy to cause: SITL prints
# "Waiting for connection ...." and looks hung until the pilot stand-in connects, so it gets started
# twice. Say so plainly rather than letting the second one fail with "Address already in use".
if command -v lsof >/dev/null 2>&1 && lsof -nP -iTCP:5760 -sTCP:LISTEN >/dev/null 2>&1; then
  echo "tcp:5760 is already in use -- an earlier SITL is still running." >&2
  echo "  lsof -nP -iTCP:5760 -sTCP:LISTEN     # find it" >&2
  echo "  pkill -f 'build/sitl/bin/arducopter' # stop it" >&2
  exit 2
fi

mkdir -p "$OUT_DIR"
# SERIAL5_PROTOCOL 2 is required: without it SITL opens the port but speaks no MAVLink on it, and
# QGroundControl sits there showing nothing.
printf 'SERIAL5_PROTOCOL 2\n' > "$OUT_DIR/qgc.param"

PIDS=()
cleanup() {
  for ((i = ${#PIDS[@]} - 1; i >= 0; i--)); do
    local pid=${PIDS[i]}
    kill "$pid" 2>/dev/null || continue
    for _ in $(seq 1 30); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -9 "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT

echo "== $NAME: QGroundControl feed on udp:$QGC_PORT, logs in $OUT_DIR"
(cd "$OUT_DIR" && exec "$ARDUCOPTER" --model quad --speedup 1 -w \
  --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$HERE/ardupilot/follow.param,$HERE/ardupilot/sitl.param,$OUT_DIR/qgc.param" \
  --serial4 udpclient:127.0.0.1:14560 \
  --serial5 "udpclient:127.0.0.1:$QGC_PORT" \
  --home -35.363261,149.165230,584,353) >"$OUT_DIR/sitl.log" 2>&1 &
PIDS+=($!)
sleep 3

"$PYTHON" "$HERE/sitl_operator.py" --connect tcp:127.0.0.1:5760 >"$OUT_DIR/operator.log" 2>&1 &
PIDS+=($!)
sleep 2

"$BUILD_DIR/follow_app" --sim --scenario "$SCENARIO" --config "$COURSEWORK/config/follow.json" \
  --link udp:14560 --log "$OUT_DIR/run.csv" >"$OUT_DIR/app.log" 2>&1
echo "follow_app exited: $?"

echo "== states"
tail -n +2 "$OUT_DIR/run.csv" | cut -d, -f2 | uniq | tr '\n' ' '
echo

status=0
"$BUILD_DIR/follow_check_run" --log "$OUT_DIR/run.csv" --scenario "$SCENARIO" \
  --config "$COURSEWORK/config/follow.json" --tolerance-scale 1.5 || status=$?
exit "$status"
