#!/usr/bin/env python3
"""지게팔 `u` 를 초음파 정지(높이 제한)까지 반복. 시리얼 줄 앞글자가 빠져도("Eco:", "p complete") 끝을 알아본다.

끝 판정(한 번의 u): "complete" 포함 → 완료, "stopped" 포함 → 높이 제한, "Motor ready" 포함 → 재부팅(정지),
UP_TIMEOUT_S 안에 아무 끝 줄도 없으면 정지. 상승량은 완료 12800 + 높이 제한 때 (거리 줄 수 x 320).

사용: lift_up_until.py [--max-up 5]
"""
import argparse
import fcntl
import json
import re
import threading
import time
from pathlib import Path

PULSES, CHECK = 12800, 320
UP_TIMEOUT_S = 9.0
ECHO = re.compile(r"(?:E?c?h?o?):?\s*\d+\s*us\s*\|\s*Distance:\s*(-?\d+(?:\.\d+)?)\s*cm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-up", type=int, default=5)
    ap.add_argument("--port", default="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[2] / "logs/pickup_real"))
    a = ap.parse_args()
    import serial
    out = Path(a.out) / time.strftime("lift_up_%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    logf = (out / "serial.log").open("w", encoding="utf-8")
    lock = open("/tmp/packagu_lift_uno.lock", "w", encoding="ascii")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    conn = serial.Serial(a.port, 115200, timeout=0.05, exclusive=True)
    lines, cv, stop = [], threading.Condition(), [False]
    t0 = time.monotonic()

    def reader():
        while not stop[0]:
            raw = conn.readline()
            if raw:
                line = raw.decode("utf-8", "replace").strip()
                t = time.monotonic()
                logf.write("%.3f %s\n" % (t - t0, line))
                logf.flush()
                with cv:
                    lines.append((t, line))
                    cv.notify_all()

    threading.Thread(target=reader, daemon=True).start()

    def wait(since, pred, timeout):
        end = time.monotonic() + timeout
        with cv:
            while True:
                for t, l in lines:
                    if t >= since and pred(l):
                        return t, l
                if time.monotonic() >= end:
                    return None
                cv.wait(end - time.monotonic())

    rec = {"ups": [], "pulses": 0}
    try:
        if wait(t0, lambda l: "stop distance" in l, 6.0) is None:
            raise SystemExit("STOP: banner not seen")
        time.sleep(0.3)
        for k in range(1, a.max_up + 1):
            s = time.monotonic()
            conn.write(b"u")
            conn.flush()
            st = wait(s, lambda l: "Up start" in l or "p start" in l, 2.0)
            if st is None:
                raise SystemExit("STOP: no Up start")
            end = wait(st[0] + 0.01, lambda l: "complete" in l or "stopped" in l or "Motor ready" in l, UP_TIMEOUT_S)
            echoes = [l for t, l in lines if t > st[0] and ECHO.search(l)]
            cms = [float(ECHO.search(l).group(1)) for l in echoes]
            if end is None:
                rec["ups"].append({"k": k, "result": "no end line", "echoes": len(echoes)})
                raise SystemExit("STOP: up %d no end line in %.0f s (echoes %d)" % (k, UP_TIMEOUT_S, len(echoes)))
            if "Motor ready" in end[1]:
                rec["ups"].append({"k": k, "result": "reboot", "echoes": len(echoes), "pulses_est": len(echoes) * CHECK})
                rec["pulses"] += len(echoes) * CHECK
                raise SystemExit("STOP: board rebooted during up %d" % k)
            if "stopped" in end[1]:
                p = max(0, len(echoes) - 1) * CHECK
                rec["pulses"] += p
                rec["ups"].append({"k": k, "result": "height_limit", "pulses": p, "last_cm": cms[-1] if cms else None,
                                   "seconds": round(end[0] - st[0], 3)})
                print("up %d: height limit after %.2f s (~%d pulses, last %s cm)" % (k, end[0] - st[0], p, cms[-1] if cms else None), flush=True)
                rec["ok"] = True
                print("OK lifted to height limit, this run %d pulses" % rec["pulses"], flush=True)
                return 0
            rec["pulses"] += PULSES
            rec["ups"].append({"k": k, "result": "complete", "seconds": round(end[0] - st[0], 3), "echoes": len(echoes),
                               "min_cm": min(cms) if cms else None})
            print("up %d: complete %.2f s (echoes %d, min %s cm)" % (k, end[0] - st[0], len(echoes), min(cms) if cms else None), flush=True)
            time.sleep(0.2)
        raise SystemExit("STOP: no height limit within %d up commands" % a.max_up)
    finally:
        (out / "result.json").write_text(json.dumps(rec, indent=1))
        print("LOG", out, "pulses", rec["pulses"], flush=True)
        stop[0] = True
        time.sleep(0.1)
        conn.close()


if __name__ == "__main__":
    raise SystemExit(main())
