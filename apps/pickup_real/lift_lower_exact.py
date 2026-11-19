#!/usr/bin/env python3
"""지게팔을 정해진 펄스 수만큼 내린다(현재 펌웨어 `d` = 12800 펄스 고정, 정지 명령 없음).

1) 전체 `d` 를 필요한 만큼 보내며 "Down start"→"Down complete" 시간으로 1펄스 시간을 실측한다.
2) 남은 펄스는 `d` 를 보낸 뒤 그 시간만큼 기다렸다가 보드를 리셋(DTR)해 끊는다. 리셋하면 펄스가 멈춘다.
   리셋 뒤 부팅 안내("Motor ready")가 다시 보여야 성공. 짐이 없을 때만 쓴다(하강은 초음파 확인 없음).

사용: lift_lower_exact.py --pulses 46400 [--port ...] [--dry-run]
"""
import argparse
import fcntl
import json
import threading
import time
from pathlib import Path
from lift_carry_protocol import suffix_line,is_reboot

PULSES = 12800
BANNER = "Motor ready"


class Board:
    def __init__(self, port, log):
        import serial
        self.lock_file = open("/tmp/packagu_lift_uno.lock", "w", encoding="ascii")
        fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)  # 다른 지게팔 사용자가 있으면 여기서 실패
        self.conn = serial.Serial(port, 115200, timeout=0.05, exclusive=True)
        self.lines, self.log, self.cv = [], log, threading.Condition()
        self.stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while not self.stop:
            raw = self.conn.readline()
            if raw:
                line = raw.decode("utf-8", "replace").strip()
                t = time.monotonic()
                self.log(t, line)
                with self.cv:
                    self.lines.append((t, line))
                    self.cv.notify_all()

    def wait(self, since, pred, timeout):
        end = time.monotonic() + timeout
        with self.cv:
            while True:
                for t, line in self.lines:
                    if t >= since and pred(line):
                        return t, line
                left = end - time.monotonic()
                if left <= 0:
                    return None
                self.cv.wait(left)

    def send(self, c):
        self.conn.write(c.encode("ascii"))
        self.conn.flush()

    def reset(self):
        self.conn.dtr = False
        time.sleep(0.02)
        self.conn.dtr = True  # DTR 내려감 → Uno 자동 리셋 회로


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pulses", type=int, required=True)
    ap.add_argument("--port", default="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[2] / "logs/pickup_real"))
    ap.add_argument("--latency-s", type=float, default=0.008, help="Down start 줄 수신 지연 보정")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--counted", action="store_true", help="require firmware pulse count acknowledgements")
    a = ap.parse_args()
    if not 0 < a.pulses <= 80000:
        ap.error("--pulses must be between 1 and 80000")
    full, rest = divmod(a.pulses, PULSES)
    print("PLAN full_d=%d rest_pulses=%d" % (full, rest), flush=True)
    if a.dry_run:
        return 0
    out = Path(a.out) / time.strftime("lift_lower_%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    logf = (out / "serial.log").open("w", encoding="utf-8")
    t0 = time.monotonic()

    def log(t, line):
        logf.write("%.4f %s\n" % (t - t0, line))
        logf.flush()
        print("  UNO> " + line, flush=True)

    b = Board(a.port, log)
    rec = {"target_pulses": a.pulses, "full_d": full, "rest_pulses": rest, "moves": []}
    try:
        if b.wait(t0, lambda l: "stop distance" in l, 6.0) is None:
            raise SystemExit("STOP: banner not seen")
        if a.counted:
            if b.wait(t0, lambda l: l == "Counted lift v1", 1.0) is None:
                raise SystemExit("STOP: counted lift firmware required")
            remaining = a.pulses
            while remaining:
                chunk = min(remaining, PULSES)
                expected = "Down pulses: %d" % chunk
                sent = time.monotonic()
                b.send("x%d\n" % chunk)
                ack = b.wait(sent, lambda l: l.startswith("Down pulses") or is_reboot(l), 8.0)
                if ack is None or ack[1] != expected:
                    raise SystemExit("STOP: counted descent unconfirmed (%s)" % (ack,))
                rec["moves"].append({"pulses": chunk, "confirmed": True})
                remaining -= chunk
            rec["ok"] = True
            print("OK lowered", a.pulses, "board-counted pulses", flush=True)
            return 0
        time.sleep(0.3)
        per = []
        for i in range(full):
            s = time.monotonic()
            b.send("d")
            st = b.wait(s, lambda l: suffix_line(l,"Down start"), 2.0)
            if st is None:
                raise SystemExit("STOP: no Down start")
            en = b.wait(st[0], lambda l: suffix_line(l,"Down complete") or is_reboot(l), 8.0)
            if en is None or not suffix_line(en[1],"Down complete"):
                raise SystemExit("STOP: Down complete not seen (%s)" % (en,))
            dt = en[0] - st[0]
            per.append(dt / PULSES)
            rec["moves"].append({"n": i + 1, "pulses": PULSES, "seconds": round(dt, 4)})
            print("d %d: %.3f s (%.2f us/pulse)" % (i + 1, dt, dt / PULSES * 1e6), flush=True)
            time.sleep(0.3)
        if rest:
            p = sum(per) / len(per) if per else 0.000321
            wait_s = max(0.0, rest * p - a.latency_s)
            s = time.monotonic()
            b.send("d")
            st = b.wait(s, lambda l: suffix_line(l,"Down start"), 2.0)
            if st is None:
                raise SystemExit("STOP: no Down start (partial)")
            time.sleep(max(0.0, st[0] + wait_s - time.monotonic()))
            cut = time.monotonic()
            b.reset()
            # 리셋 뒤 부팅 줄은 글자가 빠져 올 수 있다("Motr redy", 192호 2026-11-01) → 부팅 안내 줄 중 하나라도
            boot = b.wait(cut, lambda l: BANNER in l or "redy" in l or "ready" in l or "Up (ultra" in l or "stop distance" in l, 6.0)
            done = b.wait(st[0], lambda l: suffix_line(l,"Down complete"), 0.0)
            ok = boot is not None and done is None
            est = round(((cut - st[0]) + a.latency_s) / p) if p else None
            rec["moves"].append({"n": full + 1, "partial": True, "us_per_pulse": round(p * 1e6, 3),
                                 "cut_after_s": round(cut - st[0], 4), "est_pulses": est,
                                 "reset_seen": boot is not None, "ok": ok})
            print("partial: cut %.3f s after Down start, est %s pulses, reset_seen=%s" %
                  (cut - st[0], est, boot is not None), flush=True)
            if not ok:
                raise SystemExit("STOP: partial cut not confirmed")
        rec["ok"] = True
        print("OK lowered", a.pulses, "pulses (estimated)", flush=True)
        return 0
    finally:
        (out / "result.json").write_text(json.dumps(rec, indent=1))
        print("LOG", out, flush=True)
        b.stop = True
        time.sleep(0.1)
        b.conn.close()


if __name__ == "__main__":
    raise SystemExit(main())
