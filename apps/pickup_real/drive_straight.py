#!/usr/bin/env python3
"""직진/후진 한 번(실제 odom 기준). 후진이면 라이다 뒤쪽 여유를 먼저 본다. 끝나면 gate 소프트웨어 정지.

사용: drive_straight.py --distance -0.35
"""
import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import yaml  # noqa: E402

from straight_pickup import RealStraightBase, Stop  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--distance", type=float, required=True)
    ap.add_argument("--config", default=str(HERE / "config.yaml"))
    a = ap.parse_args()
    cfg = yaml.safe_load(Path(a.config).read_text())
    s = cfg["straight"]

    def log(kind, **f):
        if kind != "base_out":
            print(kind, f, flush=True)

    base = RealStraightBase(cfg, log)
    try:
        if a.distance < 0:
            rear = (base.probe().get("rear") or {})
            need = -a.distance + float(s["rear_margin_m"])
            if rear.get("nearest_m") is not None and rear["nearest_m"] < need:
                raise Stop("rear clearance %.2f m < %.2f m" % (rear["nearest_m"], need))
            if rear.get("nearest_m") is None and rear.get("reason") != "no rear points (open or out of range)":
                raise Stop("rear clearance unknown: %s" % rear.get("reason"))
        base.prepare()
        speed = float(s["speed_m_s"]) if a.distance > 0 else float(s["retreat_speed_m_s"])
        r = base.straight(a.distance, speed, float(s["move_max_yaw_deg"]), float(s["move_max_lateral_m"]))
        print("RESULT", "OK" if r.get("ok") else "STOP", {k: r.get(k) for k in ("reason", "forward_m", "lateral_m", "yaw_change_deg")}, flush=True)
        return 0 if r.get("ok") else 1
    except Stop as exc:
        print("RESULT STOP", exc, flush=True)
        return 1
    finally:
        if getattr(base, "ready", False):
            base.stop()


if __name__ == "__main__":
    raise SystemExit(main())
