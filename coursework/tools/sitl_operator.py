#!/usr/bin/env python3
"""Pilot stand-in for SITL runs of follow_app.

Does what the pilot does with the transmitter, through RC_CHANNELS_OVERRIDE on SITL's SERIAL0:
arm, take off in LOITER with the throttle stick, hover, then flip the mode switch (channel 6) to
GUIDED to engage follow. Keeps holding the sticks until terminated (SIGTERM/SIGINT), then flips
back to LOITER.

    sitl_operator.py [--connect tcp:127.0.0.1:5760] [--altitude 2.0] [--settle 3.0]

Needs pymavlink. Exit code 1 if arming, takeoff or the mode switch fails.
"""
import argparse
import signal
import sys
import threading
import time

from pymavlink import mavutil

# Channel 6 PWM for the three switch positions (follow.param: FLTMODE1/4/6).
SWITCH_ALT_HOLD, SWITCH_LOITER, SWITCH_GUIDED = 1000, 1555, 1800
THROTTLE_LOW, THROTTLE_HOLD, THROTTLE_CLIMB = 1000, 1500, 1650
MODE_LOITER, MODE_GUIDED = 5, 4


def log(message):
    print(f"{time.strftime('%H:%M:%S')} operator: {message}", flush=True)


class Operator:
    def __init__(self, connect):
        self.sticks = {"throttle": THROTTLE_LOW, "switch": SWITCH_LOITER}
        self.stop = threading.Event()
        self.fc_heartbeat = None
        self.mav = self.connect(connect)

    @staticmethod
    def connect(address):
        for _ in range(60):
            try:
                mav = mavutil.mavlink_connection(address, source_system=255)
                if mav.wait_heartbeat(timeout=5):
                    log(f"connected to system {mav.target_system}")
                    return mav
            except OSError:
                pass
            time.sleep(1)
        raise RuntimeError(f"no heartbeat on {address}")

    def send_sticks(self):
        # 10 Hz, well inside RC_OVERRIDE_TIME (3 s). 0 releases a channel back to the RC input.
        while not self.stop.is_set():
            self.mav.mav.rc_channels_override_send(
                self.mav.target_system, self.mav.target_component,
                1500, 1500, self.sticks["throttle"], 1500, 0, self.sticks["switch"], 0, 0)
            time.sleep(0.1)

    def pump(self, seconds):
        """Reads messages for `seconds`, logging STATUSTEXT. Returns the latest LOCAL_POSITION_NED."""
        end = time.time() + seconds
        position = None
        while time.time() < end:
            msg = self.mav.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                time.sleep(0.01)  # recv_match returns at once when SITL has closed the socket
                continue
            if msg.get_type() == "STATUSTEXT":
                log(f"FC: {msg.text}")
            elif msg.get_type() == "LOCAL_POSITION_NED":
                position = msg
            elif msg.get_type() == "HEARTBEAT" and msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                # ArduPilot also forwards follow_app's own HEARTBEAT (component 191) to this link.
                self.fc_heartbeat = msg
        return position

    def mode(self):
        return self.fc_heartbeat.custom_mode if self.fc_heartbeat else None

    def armed(self):
        return bool(self.fc_heartbeat and self.fc_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def run(self, altitude, settle, arm_timeout):
        threading.Thread(target=self.send_sticks, daemon=True).start()
        self.mav.mav.request_data_stream_send(self.mav.target_system, self.mav.target_component,
                                              mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)

        deadline = time.time() + arm_timeout
        while not self.armed():
            if time.time() > deadline:
                raise RuntimeError(f"not armed after {arm_timeout:.0f} s")
            self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                           mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            self.pump(2.0)
        log(f"armed in mode {self.mode()}")

        self.sticks["throttle"] = THROTTLE_CLIMB
        deadline = time.time() + 20
        height = 0.0
        while height < altitude - 0.3:
            if time.time() > deadline:
                raise RuntimeError(f"takeoff stalled at {height:.1f} m")
            position = self.pump(0.1)
            if position is not None:
                height = -position.z
        self.sticks["throttle"] = THROTTLE_HOLD
        position = self.pump(settle)
        log(f"hovering at {-position.z if position else height:.1f} m in mode {self.mode()}")
        if self.mode() != MODE_LOITER:
            raise RuntimeError(f"expected LOITER before engaging, FC is in mode {self.mode()}")

        self.sticks["switch"] = SWITCH_GUIDED
        deadline = time.time() + 5
        while self.mode() != MODE_GUIDED:
            if time.time() > deadline:
                raise RuntimeError(f"mode switch did not select GUIDED (mode {self.mode()})")
            self.pump(0.1)
        log("switch in GUIDED: follow engaged")

        while not self.stop.is_set():
            self.pump(0.5)
        self.sticks["switch"] = SWITCH_LOITER
        time.sleep(0.3)
        log("switch back in LOITER")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connect", default="tcp:127.0.0.1:5760", help="SITL SERIAL0 (default tcp:127.0.0.1:5760)")
    parser.add_argument("--altitude", type=float, default=2.0, help="takeoff height in metres (default 2)")
    parser.add_argument("--settle", type=float, default=3.0, help="seconds to hover in LOITER before engaging")
    parser.add_argument("--arm-timeout", type=float, default=120.0, help="seconds to wait for GPS lock and arming")
    args = parser.parse_args()

    operator = Operator(args.connect)
    signal.signal(signal.SIGTERM, lambda *_: operator.stop.set())
    signal.signal(signal.SIGINT, lambda *_: operator.stop.set())
    try:
        operator.run(args.altitude, args.settle, args.arm_timeout)
    except RuntimeError as error:
        log(f"FAILED: {error}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
