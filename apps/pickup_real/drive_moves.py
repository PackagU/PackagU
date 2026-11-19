#!/usr/bin/env python3
"""회전·직진 순서 실행(실제 odom 기준). 예: drive_moves.py R90 F0.30 R90 F1.18 R90
L/R = 왼쪽/오른쪽 회전(도), F/B = 전진/후진(m). 하나라도 실패하면 그 자리에서 멈추고 gate 소프트웨어 정지.
"""
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import yaml  # noqa: E402

from straight_pickup import RealStraightBase, Stop  # noqa: E402


def parse(tokens):
    moves = []
    for t in tokens:
        kind, val = t[0].upper(), float(t[1:])
        if kind not in "LRFB" or not (0 < val <= (180 if kind in "LR" else 1.2)):
            raise SystemExit("bad move %s" % t)
        moves.append(("turn", math.radians(val if kind == "L" else -val)) if kind in "LR"
                     else ("straight", val if kind == "F" else -val))
    return moves


def main():
    moves = parse(sys.argv[1:])
    cfg = yaml.safe_load((HERE / "config.yaml").read_text())
    s = cfg["straight"]

    def log(kind, **f):
        if kind != "base_out":
            print(kind, f, flush=True)

    base = RealStraightBase(cfg, log)
    t0 = time.monotonic()
    try:
        base.prepare()
        for kind, v in moves:
            if kind == "turn":
                r = base.turn(v)
                print("turn %+.1f -> %s" % (math.degrees(v), r.get("turned_deg")), flush=True)
                time.sleep(1.0)   # 회전 뒤 흔들림이 가라앉기 전 직진하면 yaw 변화로 바로 멈춘다(192호 2026-11-01)
            else:
                speed = 0.05   # 팔레트 운반: 0.03 은 하중에서 출발 못 함(stall), 0.05 는 됨(192호 2026-11-01)
                r = base.straight(v, speed, 10.0, 0.08, hold_heading=True)   # 팔레트를 든 이동은 한쪽으로 휜다(1.2 m 에 3°)
                print("straight %+.3f -> %s %s" % (v, r.get("forward_m"), r.get("reason")), flush=True)
                if not r.get("ok"):
                    raise Stop("straight: %s" % r.get("reason"))
        print("RESULT OK %.1fs" % (time.monotonic() - t0), flush=True)
        return 0
    except Stop as exc:
        print("RESULT STOP", exc, flush=True)
        return 1
    finally:
        if getattr(base, "ready", False):
            base.stop()


if __name__ == "__main__":
    raise SystemExit(main())
