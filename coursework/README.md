# follow_app — build and simulation

Visual target follow over MAVLink: a camera (or a simulated one) tracks a target, and the vehicle
turns to face it and holds a set distance. See [PROJECT.md](PROJECT.md) for the goal and
[COMPONENTS.md](COMPONENTS.md) for the hardware.

The app has two modes:

- `--sim` — a simulated camera against **ArduPilot SITL**. No camera, no OpenCV, no Raspberry Pi.
  This is the mode to use for checking the project, and it is what the rest of this file covers.
- `--hw` — a Raspberry Pi camera and a real tracker. Requires a build with `FOLLOW_WITH_OPENCV=ON`
  and real hardware; see [docs/VISION.md](docs/VISION.md).

All commands below are run **from the repository root**, not from `coursework/`.

## What the simulation covers

`--sim` runs the whole flight loop against a real ArduCopter: MAVLink over UDP, the pilot arming and
switching to GUIDED, the estimator, the supervisor's state machine, the controller, and the velocity
setpoints on the wire. A scenario supplies the target's motion in place of a camera.

It does **not** exercise the camera path — the OpenCV tracker, the GStreamer pipeline and the
framebuffer overlay are `--hw` only. A passing simulation says nothing about those.

## Source layout

The module follows `homework_11`'s structure: role directories, PascalCase filenames matching the
class, `I`-prefixed interfaces, and includes with no project prefix (`#include "control/Core.h"`).

```
include/
  Types.h  ControlLoop.h  MissionProcessor.h  HwMissionProcessor.h  StatCollector.h
  models/     data and pure math: Angles, Frames, AttitudeHistory, Config, CameraModel
  control/    the decision stack: Core, FollowController, TargetEstimator, Supervisor
  comms/      links and protocol: ByteLink, Links, MavlinkClient, MavlinkIo
  providers/  observation sources: FrameSource, PiCameraSource, CameraTrackerSource, SimVision
  vision/     OpenCV-only: Tracker, Overlay, Calibration
  config/  util/  sim/
```

Namespaces mirror the directories under a `follow::` root (`follow::models`, `follow::control`,
`follow::comms`, ...). The root is kept deliberately: the top-level `CMakeLists.txt` builds
`homework_11` and `coursework` in one project, and `homework_11` already defines a global
`namespace comms` with its own `MavLink`/`SerialLink`/`SocketLink`. Without the `follow::` root
those would collide.

### follow_core must stay pure

**`models/` and `control/` — and therefore the `follow_core` target — contain no threads, no I/O,
no JSON, no MAVLink, no OpenCV and no clock reads.** Everything there is a pure function of its
arguments, which is what makes the estimator, controller and supervisor testable without a vehicle.

Anything that needs a thread, a socket, a file or the clock belongs outside them: `ControlLoop.h`
and the `MissionProcessor`s at include root own the threads, `comms/` owns the sockets, `config/`
owns the file parsing, `providers/` owns the cameras.

Two practical consequences:

- `FOLLOW_WITH_OPENCV=OFF` is the default and must keep building. If an OpenCV include reaches
  `models/` or `control/`, the default build, the devcontainer gcc-13 build and the aarch64
  cross-build all break at once.
- `include/control/Core.h` is hand-formatted and is the one file never passed to clang-format.
  Everything else is clang-format clean under `--style=file:.devcontainer/.clang-format`.

## Prerequisites

**A C++20 toolchain and CMake ≥ 3.20.** The default build needs nothing else — no OpenCV.

**ArduPilot SITL**, built once:

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf copter
```

This produces `build/sitl/bin/arducopter`. Point `ARDUPILOT_DIR` at the checkout.

**Python with `pymavlink`**, for the pilot stand-in. A virtualenv keeps it out of the system Python:

```bash
python3 -m venv build/sitl-venv
build/sitl-venv/bin/pip install pymavlink
```

**QGroundControl** is optional, and only for watching a run — see below.

## Build and test

```bash
cmake -S coursework -B build/coursework          # FOLLOW_WITH_OPENCV defaults to OFF
cmake --build build/coursework -j8
ctest --test-dir build/coursework
```

That builds `follow_app` and `follow_check_run` and runs the unit and integration suites.

To build the vision adapter as well (needs OpenCV; not required for simulation):

```bash
cmake -S coursework -B build/vision -DFOLLOW_WITH_OPENCV=ON
cmake --build build/vision -j8
ctest --test-dir build/vision
```

## Run one scenario

```bash
export ARDUPILOT_DIR=/path/to/ardupilot
export PYTHON=$PWD/build/sitl-venv/bin/python

coursework/tools/run_sitl_scenario.sh coursework/config/scenarios/stationary.json
```

The script starts ArduCopter SITL, starts `follow_app --sim`, and runs `sitl_operator.py` as the
pilot: it arms, takes off in LOITER, hovers, then flips the mode switch to GUIDED, which is what
engages follow. When the scenario's duration has elapsed the app exits and `follow_check_run`
validates the run log against the scenario's expectations.

Output is one line:

```text
PASS stationary (1361 steps, tolerance x1.5)
```

Exit code 0 is a pass, 1 a fail, 2 a setup error. Logs land in
`build/sitl-runs/<scenario>-<timestamp>/`: `run.csv` (the run log), `app.log`, `sitl.log`,
`operator.log` and `check.txt`.

Tolerances are widened 1.5× for SITL, because a real autopilot's response is not the ideal model the
scenario expectations were written against.

Other environment variables the script accepts: `ARDUCOPTER` (binary path), `BUILD_DIR` (default
`build/coursework`), `OUT_DIR`, `TIMEOUT_S` (default 300).

## Run all scenarios

```bash
export ARDUPILOT_DIR=/path/to/ardupilot
export PYTHON=$PWD/build/sitl-venv/bin/python

for s in coursework/config/scenarios/*.json; do
  coursework/tools/run_sitl_scenario.sh "$s" || echo "FAILED: $s"
done
```

Each scenario starts its own SITL instance and takes a minute or two, so the full set is roughly
fifteen minutes.

| Scenario | What it checks |
| --- | --- |
| `stationary` | Target stands still 3 m ahead. |
| `stationary_known_size` | Known 1.7 m height gives metric distance, so the vehicle closes to `d_set`. |
| `walk_line` | Target walks away at 1.0 m/s; the proportional lag must settle, not grow. |
| `stop_and_go` | Target walks, stops, walks, stops. |
| `circle` | Target walks a 6 m circle at 1 m/s around a point 9 m ahead. |
| `yaw_only` | Bring-up stage 2: `enable_vx = false`, only yaw is commanded. |
| `fast_dash` | Target runs sideways at 8 m/s and stays inside the 128° view. |
| `pass_by` | Target is followed, then runs past the vehicle at 5 m/s and leaves the view. |
| `occlusion_short` | Target hidden for 1 s and found again. |
| `occlusion_long` | Target hidden for 5 s, longer than `lost_timeout`. |
| `lock_miss` | Target outside the centre lock box: the lock misses and nothing moves. |

The occlusion and `lock_miss` scenarios are the ones that exercise the Lost/Reacquire path, so they
are the most sensitive to changes in the estimator or the tracker adapter.

## Watch a run in QGroundControl

The scenario runner does not expose a link for a ground station: `follow_app` uses SERIAL4 and the
pilot stand-in holds SERIAL0's TCP port. To watch a run, start SITL by hand with an extra MAVLink
serial pointed at QGroundControl's default UDP port.

> **SITL will print `Waiting for connection ....` and appear to hang.** That is normal: it does not
> start running until something connects to TCP 5760, which is the pilot stand-in in the next step.
> Do not start SITL a second time — the second instance fails with
> `bind failed on port 5760 - Address already in use` and overwrites the first one's log.

```bash
export AP=/path/to/ardupilot
export REPO=$PWD                                  # repository root
export PYTHON=$REPO/build/sitl-venv/bin/python

# Nothing should be holding SITL's port from an earlier run.
lsof -nP -iTCP:5760 -sTCP:LISTEN || echo "port 5760 free"

mkdir -p /tmp/qgc-run
printf 'SERIAL5_PROTOCOL 2\n' > /tmp/qgc-run/qgc.param

( cd /tmp/qgc-run && exec $AP/build/sitl/bin/arducopter --model quad --speedup 1 -w \
    --defaults "$AP/Tools/autotest/default_params/copter.parm,$REPO/coursework/tools/ardupilot/follow.param,$REPO/coursework/tools/ardupilot/sitl.param,/tmp/qgc-run/qgc.param" \
    --serial4 udpclient:127.0.0.1:14560 \
    --serial5 udpclient:127.0.0.1:14550 \
    --home -35.363261,149.165230,584,353 ) > /tmp/qgc-run/sitl.log 2>&1 &
```

SITL writes `eeprom.bin` and a `logs/` directory into its working directory, which is why it runs in
a subshell in `/tmp/qgc-run` rather than in the repository.

`SERIAL5_PROTOCOL 2` is required — without it SITL opens the port but speaks no MAVLink on it.

Now start the pilot stand-in, which connects to TCP 5760 and lets SITL proceed, and then the app:

```bash
cd $REPO
$PYTHON coursework/tools/sitl_operator.py --connect tcp:127.0.0.1:5760 &

build/coursework/follow_app --sim \
  --scenario coursework/config/scenarios/walk_line.json \
  --config coursework/config/follow.json \
  --link udp:14560 --log /tmp/qgc-run/run.csv
```

QGroundControl listens on UDP 14550 by default and picks the vehicle up on its own. You will see it
arm, climb, switch to GUIDED, and then yaw and translate as follow engages.

Stop the background processes when finished:

```bash
pkill -f sitl_operator.py; pkill -f 'build/sitl/bin/arducopter'
```

## Checking a run log by hand

`run.csv` can be re-checked without re-flying:

```bash
build/coursework/follow_check_run \
  --log build/sitl-runs/<run>/run.csv \
  --scenario coursework/config/scenarios/stationary.json \
  --config coursework/config/follow.json \
  --tolerance-scale 1.5
```

## Running follow_app directly

The runner script is a convenience. The app itself:

```text
follow_app --sim --scenario FILE [--config FILE] [--link SPEC] [--log FILE]
  --scenario  scenario JSON (coursework/config/scenarios/*.json)
  --config    follow.json (default config/follow.json)
  --link      udp:PORT, udp:PORT:HOST:PORT or uart:DEVICE:BAUD (default udp:14560 for --sim)
  --log       CSV run log (default follow_run.csv)
```

With no autopilot on the link the app reports `FC: mavlink link wait failed` and stays in `NoFc`,
writing no setpoint at all — the run log's `vx` and `yaw_rate` columns stay empty. That is the safe
default rather than an error: it waits for a flight controller instead of commanding a vehicle it
cannot see.

## Troubleshooting

**`bind failed on port 5760 - Address already in use`** — a SITL instance from an earlier run is
still alive. SITL prints `Waiting for connection ....` and sits there until the pilot stand-in
connects, so a healthy instance looks hung and is easy to start twice. Find and stop the old one:

```bash
lsof -nP -iTCP:5760 -sTCP:LISTEN     # shows the PID holding the port
pkill -f 'build/sitl/bin/arducopter' # or kill <PID> for just the one
```

Note that the second instance overwrites the first one's `sitl.log` before it exits, so the log may
show the bind failure while the working instance is the one still running.

**`<python> cannot import pymavlink`** — `PYTHON` is not pointing at the virtualenv. Use an absolute
path: `export PYTHON=$PWD/build/sitl-venv/bin/python`.

**`no SITL binary at ...`** — `ARDUPILOT_DIR` is unset or SITL is not built. See Prerequisites.

**`sitl_operator.py exited early`** — check `operator.log` in the run directory. The usual cause is
arming being refused because SITL has not got a GPS fix yet; the operator waits up to 120 s
(`--arm-timeout`).

**The vehicle arms and hovers but never follows** — follow engages only in GUIDED. Check
`operator.log` for the mode switch, and `run.csv`'s `state` column: `NoFc` means no MAVLink from the
autopilot at all, `Idle` means it is connected but has not entered GUIDED.

**It stays in `Idle` even though the vehicle is already in GUIDED** — engagement is triggered by the
*transition* into GUIDED, not by being in it (`Supervisor.cpp`: `becameGuided`). If you switch to
GUIDED in QGroundControl before starting `follow_app`, the app never sees the edge. Switch to LOITER
and back to GUIDED with the app running. This is deliberate: it means restarting the app next to an
already-armed vehicle cannot make it take off after a target on its own.
