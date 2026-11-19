#!/usr/bin/env python3
"""선택 설치된 fine jog 펌웨어에서만 +/- 160펄스 한 번. 기본은 장치 없는 계획 출력."""
import argparse
import fcntl
import json
from pathlib import Path
import time

import yaml

FIRMWARE_BANNER = "Fine jog v2: +/- 160 pulses, ramp 200-500 pps"


def wait_line(conn, expected, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = conn.readline().decode("ascii", errors="replace").strip()
        if line == expected:
            return
        if line == "Jog blocked: height limit":
            raise RuntimeError("up jog blocked by the existing ultrasonic height limit")
    raise RuntimeError("response missing: %s (no automatic retry)" % expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("direction", choices=("up", "down"))
    parser.add_argument("--count", type=int, choices=range(1, 6), default=1,
                        help="160펄스 구간 수(1~5). 각 완료 응답 뒤에만 다음 구간을 보낸다")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    args = parser.parse_args()
    result = {"direction": args.direction, "pulses": 160 * args.count, "chunks": args.count,
              "completed_chunks": 0, "distance_cm": None, "executed": False}
    if not args.execute:
        print(json.dumps(result))
        return 0
    import serial
    cfg = yaml.safe_load(Path(args.config).read_text())
    # 기존 터미널·진행기와 같은 잠금. 새 펌웨어 부팅 줄을 확인한 뒤에만 이동 문자를 보낸다.
    with open("/tmp/packagu_lift_uno.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with serial.Serial(cfg["lift"]["port"], 115200, timeout=0.1, exclusive=True) as conn:
            wait_line(conn, FIRMWARE_BANNER, 6.0)
            for _ in range(args.count):
                written = conn.write(b"+" if args.direction == "up" else b"-")
                conn.flush()
                if written != 1:
                    raise RuntimeError("short serial write; no automatic retry")
                wait_line(conn, "Jog %s complete" % args.direction, 3.0)
                result["completed_chunks"] += 1
            result["executed"] = True
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
