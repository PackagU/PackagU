#!/usr/bin/env python3
"""들고 후진한 팔레트를 원래 자리에 내려놓고 기준 위치로 돌아온다(직진만, 실제 odom 기준).

1 점검: scan·odom·gate, 마지막 후진 거리 + 여유만큼 뒤가 비었는지(라이다)
2 직진 --forward m (들어 올린 자리로)
3 지게팔 --lower-pulses 만큼 하강(lift_lower_exact.py: d 반복 + 나머지는 리셋으로 끊기)
4 직진 후진 --retreat m (포크를 빼고 기준 위치로)
벗어나면(yaw·좌우·센서 끊김) 그 자리에서 멈춘다. 끝나면 gate 소프트웨어 정지.

사용(Jetson): ~/pickup_venv/bin/python apps/pickup_real/put_back.py --forward 0.3516 --lower-pulses 52800 --retreat 1.0531
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import yaml  # noqa: E402

from straight_pickup import LOG_ROOT, STATE_FILE, RealStraightBase, Stop  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--forward", type=float, required=True)
    ap.add_argument("--lower-pulses", type=int, required=True)
    lift_mode = ap.add_mutually_exclusive_group()
    lift_mode.add_argument("--counted-lift", action="store_true")
    lift_mode.add_argument("--whole-step-lift", action="store_true")
    ap.add_argument("--retreat", type=float, required=True)
    ap.add_argument("--config", default=str(HERE / "config.yaml"))
    a = ap.parse_args()
    if not (0 <= a.forward <= 0.6 and 0 < a.retreat <= 1.2 and 0 < a.lower_pulses <= 80000):
        raise SystemExit("STOP: argument out of range")
    cfg = yaml.safe_load(Path(a.config).read_text())
    s = cfg["straight"]
    run_dir = LOG_ROOT / time.strftime("%Y%m%d_%H%M%S_put_back")
    run_dir.mkdir(parents=True, exist_ok=True)
    ev = (run_dir / "events.jsonl").open("a", encoding="utf-8")
    t0 = time.monotonic()

    def log(kind, **f):
        rec = dict({"t": round(time.monotonic() - t0, 2), "kind": kind}, **f)
        ev.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
        ev.flush()
        if kind != "base_out":
            print("[%6.1f] %-10s %s" % (rec["t"], kind, json.dumps(f, ensure_ascii=False, default=str)[:300]), flush=True)

    base = RealStraightBase(cfg, lambda kind, **f: log(kind, **f))
    try:
        p = base.probe()
        log("probe", **{k: p.get(k) for k in ("ok", "scan_hz", "scan_age_s", "odom_hz", "drive_ready", "rear")})
        if not p.get("ok") or not p.get("drive_ready"):
            raise Stop("sensor probe: %s" % p.get("reason"))
        rear_info = p.get("rear") or {}
        rear = rear_info.get("nearest_m")
        need = a.retreat - a.forward + float(s["rear_margin_m"])
        if rear is None:
            # straight_pickup 과 같은 규칙: 뒤쪽 띠 안에 점이 하나도 없으면 열린 공간, 그 밖의 이유는 정지
            if rear_info.get("reason") != "no rear points (open or out of range)":
                raise Stop("rear clearance unknown: %s" % rear_info.get("reason"))
        elif rear < need:
            raise Stop("rear clearance %.2f m < %.2f m" % (rear, need))

        def drive(d, why, carrying=False):
            base.prepare()
            speed = float(s["speed_m_s"]) if d > 0 else float(s["retreat_speed_m_s"])
            log("move", commanded_m=round(d, 4), why=why)
            if carrying:   # 팔레트를 든 직진은 왼쪽으로 틀어진다(192호 2026-11-01) → 방향 유지 보정
                r = base.straight(d, speed, 5.0, 0.05, hold_heading=True)
            else:
                r = base.straight(d, speed, float(s["move_max_yaw_deg"]), float(s["move_max_lateral_m"]))
            log("moved", **{k: r.get(k) for k in ("ok", "reason", "forward_m", "lateral_m", "yaw_change_deg", "peak_yaw_deg")})
            base.stop()
            if not r.get("ok"):
                raise Stop("straight %s: %s" % (why, r.get("reason")))
            return r

        from lift_carry_protocol import PULSES
        lower_pulses = a.lower_pulses
        if a.whole_step_lift:
            lower_pulses -= lower_pulses % PULSES
            if not lower_pulses:
                raise Stop("recorded rise is smaller than one firmware down step")
            log("lift_plan", rise_pulses=a.lower_pulses, down_pulses=lower_pulses,
                residual_pulses=a.lower_pulses - lower_pulses)
        if a.forward > 0.005:
            drive(a.forward, "back to the lift spot", carrying=True)
        log("lift", action="lower", pulses=lower_pulses)
        lower_cmd = [sys.executable, str(HERE / "lift_lower_exact.py"), "--pulses", str(lower_pulses),
                     "--out", str(LOG_ROOT)]
        if a.counted_lift:
            lower_cmd.append("--counted")
        done = subprocess.run(lower_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              universal_newlines=True)
        lines = [l for l in done.stdout.splitlines() if "Echo:" not in l]
        for l in lines:
            print("  " + l, flush=True)
        if done.returncode != 0 or not any(l.startswith("OK lowered") for l in lines):
            raise Stop("lift lower failed (rc %d)" % done.returncode)
        if a.whole_step_lift and lower_pulses != a.lower_pulses:
            raise Stop("approximate descent completed; confirm pallet rests on floor before withdrawing forks")
        time.sleep(1.0)
        drive(-a.retreat, "retreat to reference")
        STATE_FILE.write_text(json.dumps({"stage": "parcel_put_back", "at": time.time(), "run_dir": str(run_dir),
                                          "tool": "put_back", "lift": "entry height (estimated)"}))
        print("RESULT OK put back and retreated ->", run_dir, flush=True)
        return 0
    except Stop as exc:
        log("STOP", reason=str(exc))
        print("RESULT STOP", exc, "->", run_dir, flush=True)
        return 1
    finally:
        if getattr(base, "ready", False):
            base.stop()


if __name__ == "__main__":
    raise SystemExit(main())
