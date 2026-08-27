#!/usr/bin/env bash
# Ганяє simulation11 проти MAVLink-чекера в Linux-контейнері й друкує його звіт.
#
#   run_checker.sh [seconds]
#
# Змінні: CHECKER (шлях до бінарника), PLATFORM, IMAGE, QGC (host:port, куди
# додатково дублювати телеметрію через tools/mavlink_fanout.py).
#
# Усе живе в одному контейнері: опублікований UDP-порт Docker Desktop не
# повертає COMMAND_ACK, тож чекер, дрон і стенд спілкуються через localhost.
set -uo pipefail

SECS=${1:-20}
CHECKER=${CHECKER:-$HOME/Downloads/checker-linux-arm64}
PLATFORM=${PLATFORM:-linux/arm64}
IMAGE=${IMAGE:-debian:bookworm-slim}
QGC=${QGC:-}

[ -f "$CHECKER" ] || { echo "checker not found: $CHECKER (set CHECKER=...)" >&2; exit 1; }

HERE=$(cd "$(dirname "$0")" && pwd)
HW=$(dirname "$HERE")
REPO=$(cd "$HW/.." && pwd)
CK_DIR=$(cd "$(dirname "$CHECKER")" && pwd)
CK_BIN=$(basename "$CHECKER")
HW_NAME=$(basename "$HW")

# Каталог збірки лежить у репозиторії, тож повторні запуски інкрементальні.
docker run --rm --platform "$PLATFORM" \
    -v "$REPO:/src" -v "$CK_DIR:/ck:ro" \
    -w "/src/$HW_NAME" \
    -e SECS="$SECS" -e CK_BIN="$CK_BIN" -e QGC="$QGC" -e HW_NAME="$HW_NAME" \
    "$IMAGE" bash -eu -c '
apt-get update -qq >/dev/null
apt-get install -y -qq build-essential cmake python3 >/dev/null 2>&1

B=/src/$HW_NAME/build-linux
cmake -S . -B $B -DCMAKE_BUILD_TYPE=Debug > /tmp/cfg.log 2>&1 || { tail -20 /tmp/cfg.log; exit 1; }
ok=0
for i in 1 2 3; do
    if cmake --build $B -j"$(nproc)" > /tmp/bld.log 2>&1; then ok=1; break; fi
    # У цьому оточенні gcc зрідка падає з internal compiler error - просто повторюємо.
    grep -q "internal compiler error" /tmp/bld.log || { grep -iE "error" /tmp/bld.log | head -20; exit 1; }
    echo "(retry $i after gcc ICE)"
done
[ $ok -eq 1 ] || { echo "build failed"; exit 1; }

cd $B
/ck/$CK_BIN 14560 > /tmp/ck.log 2>&1 &
CK=$!

TARGET=127.0.0.1:14560
if [ -n "$QGC" ]; then
    python3 /src/$HW_NAME/tools/mavlink_fanout.py 14540 127.0.0.1:14560 "$QGC" > /tmp/relay.log 2>&1 &
    TARGET=127.0.0.1:14540
fi

python3 /src/$HW_NAME/tools/uart_sim.py 0 "$SECS" > /tmp/uart.log 2>&1 &
UP=$!
PTY=""
for i in $(seq 1 100); do
    PTY=$(grep -m1 "^PTY=" /tmp/uart.log 2>/dev/null | cut -d= -f2) && [ -n "$PTY" ] && break
    sleep 0.1
done
[ -n "$PTY" ] || { echo "no PTY from uart_sim"; cat /tmp/uart.log; exit 1; }

./simulation11 --uart "$PTY" --mavlink-port 14055 --mavlink-remote "$TARGET" > /tmp/app.log 2>&1 &
AP=$!
wait $UP
kill -INT $AP 2>/dev/null || true
sleep 1
kill -INT $CK 2>/dev/null || true
wait $CK 2>/dev/null || true

echo
cat /tmp/ck.log
grep -q "Усі перевірки пройдено" /tmp/ck.log
'
