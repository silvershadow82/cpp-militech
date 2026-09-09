#!/usr/bin/env bash
# Піднімає uart_sim.py і запускає simulation11 на цій машині.
#
#   run_local.sh <build-dir> [seconds] [mavlink-remote]
#
# simulation11 йде на передньому плані, тож видно весь лог і працює Ctrl+C.
# Вихідний код ненульовий, якщо ThreadSanitizer знайшов гонку.
set -uo pipefail

BUILD=${1:?usage: run_local.sh <build-dir> [seconds] [mavlink-remote]}
SECS=${2:-25}
REMOTE=${3:-}
PORT=${MAVLINK_PORT:-14055}

HERE=$(cd "$(dirname "$0")" && pwd)
BIN="$BUILD/simulation11"
[ -x "$BIN" ] || { echo "no binary at $BIN - run 'make build' first" >&2; exit 1; }

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"; kill %1 2>/dev/null' EXIT

python3 "$HERE/uart_sim.py" 0 "$SECS" > "$WORK/sim.log" 2>&1 &

PTY=""
for _ in $(seq 1 100); do
    PTY=$(grep -m1 '^PTY=' "$WORK/sim.log" 2>/dev/null | cut -d= -f2)
    [ -n "$PTY" ] && break
    sleep 0.1
done
[ -n "$PTY" ] || { echo "uart_sim did not report a PTY:" >&2; cat "$WORK/sim.log" >&2; exit 1; }

echo "== uart_sim on $PTY, mission ${SECS}s${REMOTE:+, MAVLink -> $REMOTE}"

if [ -n "$REMOTE" ]; then
    "$BIN" --uart "$PTY" --mavlink-port "$PORT" --mavlink-remote "$REMOTE" 2>&1 | tee "$WORK/app.log"
else
    "$BIN" --uart "$PTY" --mavlink-port "$PORT" 2>&1 | tee "$WORK/app.log"
fi

races=$(grep -c 'WARNING: ThreadSanitizer' "$WORK/app.log" || true)
inbound=$(grep -c 'rx_poll' "$WORK/app.log" || true)
echo "== summary: tsan-races=$races  inbound-mavlink-ticks=$inbound"
[ "$races" -eq 0 ] || { echo "== FAIL: ThreadSanitizer reported $races race(s)" >&2; exit 1; }
