#!/usr/bin/env python3
"""Fail-closed CLI adapter for the lift_uno firmware (firmware/lift_uno).

`jog u|d` sends one fixed 12800-pulse move. Up stops early when the ultrasonic reads below the
firmware height limit (10 cm). The firmware has no stop, status or position feedback, so those
actions are refused. Earlier jog firmware took `arm <pulses>`; lift_uno ignores that and would
run a full 12800-pulse move on the following `u`/`d`, so pulse counts are no longer accepted.
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from field_video.backends import (  # noqa: E402 — 펌웨어 문구는 진행기와 한 곳에서 관리
    LIFT_END_LINES, LIFT_FIRMWARE_ID, LIFT_LINES, LIFT_PULSES_PER_COMMAND, LIFT_STOP_DISTANCE_RE)

LIFT_PORT = "/dev/lift_uno"
UNSUPPORTED = {"stop", "status", "home", "move_to", "load", "unload"}
BANNER_TIMEOUT_SEC = 6.0


def emit(**fields):
    print(json.dumps(fields, sort_keys=True, separators=(",", ":"), ensure_ascii=False))


def frames_for(action, direction=None):
    if action == "jog":
        if direction not in ("d", "u"):
            raise ValueError("jog direction must be d or u")
        return [direction.encode("ascii")]  # 한 글자. 펌웨어는 줄바꿈을 쓰지 않는다
    if action in UNSUPPORTED:
        return []
    raise ValueError(f"unknown action: {action}")


def end_state(line):
    """펌웨어 끝 줄 → 어댑터 상태."""
    if line == LIFT_LINES["up"]["height_limit"]:
        return "HEIGHT_LIMIT"
    if line in LIFT_END_LINES:
        return "COMPLETE"
    return None


def read_until(conn, predicate, timeout):
    deadline = time.monotonic() + timeout
    lines = []
    while time.monotonic() < deadline:
        raw = conn.readline()
        if not raw:
            time.sleep(0.01)
            continue
        line = raw.decode("ascii", errors="replace").strip()
        lines.append(line)
        if predicate(line):
            return lines, line
    return lines, None


def main():
    parser = argparse.ArgumentParser(description="lift_uno up/down adapter (12800 pulses per command)")
    parser.add_argument("action", choices=["jog", *sorted(UNSUPPORTED)])
    parser.add_argument("direction", nargs="?", choices=["d", "u"])
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--port", default="")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--response-timeout", type=float, default=10.0,
                        help="seconds to wait for the end line (a full move takes about 4-6 s)")
    args = parser.parse_args()

    if args.action in UNSUPPORTED:
        emit(
            state="UNSUPPORTED",
            action=args.action,
            reason="lift_uno_firmware_has_no_stop_status_or_position_feedback",
        )
        return 2
    try:
        frames = frames_for(args.action, args.direction)
    except ValueError as exc:
        parser.error(str(exc))

    if not args.execute:
        if args.port:
            parser.error("dry-run must not receive a serial port")
        emit(
            state="DRY_RUN",
            action=args.action,
            frames=[frame.decode("ascii", errors="replace") for frame in frames],
            pulses=LIFT_PULSES_PER_COMMAND,
            ultrasonic_height_limit=args.direction == "u",
            position_verified=False,
        )
        return 0

    if args.port != LIFT_PORT:
        parser.error("physical mode requires exact udev alias --port /dev/lift_uno")
    port = Path(args.port)
    if not port.exists() or not port.is_char_device():
        sys.exit("/dev/lift_uno is not a real character device")
    if args.response_timeout <= 0:
        parser.error("response timeout must be positive")

    import fcntl
    import serial

    lock_path = Path("/tmp/packagu_lift_uno.lock")
    with lock_path.open("w", encoding="ascii") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            sys.exit("lift serial owner already active")
        with serial.Serial(args.port, args.baud, timeout=0.05) as conn:
            # 포트를 열면 보드가 리셋된다. 부팅 안내 줄로 펌웨어를 확인한 뒤에만 명령을 보낸다
            startup, banner_end = read_until(conn, lambda l: LIFT_STOP_DISTANCE_RE.match(l) is not None,
                                             BANNER_TIMEOUT_SEC)
            if banner_end is None or LIFT_FIRMWARE_ID not in startup:
                emit(state="REJECTED", action=args.action, reason="lift_uno_banner_not_seen", startup=startup)
                return 1
            for frame in frames:
                written = conn.write(frame)
                conn.flush()
                if written != len(frame):
                    sys.exit(f"short serial write {written}/{len(frame)}")
            responses, end = read_until(conn, lambda l: l in LIFT_END_LINES, args.response_timeout)
            state = end_state(end) if end else "NO_END_LINE"
            emit(
                state=state,
                action=args.action,
                direction=args.direction,
                startup=startup,
                responses=[line for line in responses if not line.startswith("Echo:")],
                last_echo=next((line for line in reversed(responses) if line.startswith("Echo:")), None),
                position_verified=False,
            )
            return 0 if state in ("COMPLETE", "HEIGHT_LIMIT") else 1


if __name__ == "__main__":
    raise SystemExit(main())
