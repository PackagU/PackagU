#!/usr/bin/env python3
"""lift_uno firmware ↔ Jetson code contract; no serial port is opened.

The Jetson side (rehearsal backend, terminal, adapter) waits for exact firmware lines. This test
checks those lines and the motion constants against firmware/lift_uno/lift_uno.ino.
"""

import contextlib
import importlib.util
import io
import queue
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
FIRMWARE = ROOT / "firmware" / "lift_uno" / "lift_uno.ino"
sys.path.insert(0, str(SCRIPTS))

from field_video import backends  # noqa: E402


def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_adapter():
    adapter = load("lift_uno_adapter")
    assert adapter.frames_for("jog", "u") == [b"u"]
    assert adapter.frames_for("jog", "d") == [b"d"]
    for action in ("stop", "status", "home", "move_to", "load", "unload"):
        assert adapter.frames_for(action) == [], action
    try:
        adapter.frames_for("jog", "x")
    except ValueError:
        pass
    else:
        raise AssertionError("bad jog direction accepted")
    assert adapter.end_state("Up complete") == "COMPLETE"
    assert adapter.end_state("Down complete") == "COMPLETE"
    assert adapter.end_state("Up stopped: distance below stop threshold") == "HEIGHT_LIMIT"
    assert adapter.end_state("Up start") is None


def literals(source):
    out = []
    for line in source.splitlines():
        parts = line.split('"')
        out.extend(parts[1::2])
    return out


def check_firmware_source():
    source = FIRMWARE.read_text(encoding="utf-8")
    for token in (
        "int PUL = 7;", "int DIR = 6;", "int ENA = 5;",
        "TRIG_PIN = 9;", "ECHO_PIN = 10;",
        "STOP_DISTANCE_CM = 10.0;", "UP_CHECK_EVERY_STEPS = 320;",
        "Serial.begin(115200);", "pulseIn(ECHO_PIN, HIGH, 30000UL)", "duration / 58.0",
        "distanceCm > 0 && distanceCm < STOP_DISTANCE_CM", "delayMicroseconds(10);",
    ):
        assert token in source, token
    # 기본 코드의 12800 펄스 이동 + 2026-10-29 속도(302 µs)를 상승·하강 모두 유지
    assert source.count("for (int i = 0; i < 12800; i++)") == 2
    assert source.count("delayMicroseconds(302);") == 2
    assert "delayMicroseconds(365)" not in source
    assert backends.LIFT_PULSES_PER_COMMAND == 12800

    found = literals(source)
    expected = [backends.LIFT_FIRMWARE_ID, *backends.LIFT_END_LINES,
                *(line for lines in backends.LIFT_LINES.values() for line in lines.values())]
    for line in expected:
        assert line in found, f"Jetson waits for {line!r} but the firmware never prints it"
    # 여러 print 로 이어 붙는 줄은 실제 모양을 만들어 정규식과 대조한다
    assert "Up stop distance: " in found and " cm" in found and "Serial.print(STOP_DISTANCE_CM, 1);" in source
    match = backends.LIFT_STOP_DISTANCE_RE.match("Up stop distance: " + "10.0" + " cm")
    assert match and float(match.group(1)) == 10.0
    assert "Echo: " in found and " us | Distance: " in found and "Serial.print(distanceCm, 2);" in source
    match = backends.LIFT_ECHO_RE.match("Echo: " + "1740" + " us | Distance: " + "30.00" + " cm")
    assert match and float(match.group(1)) == 30.0
    assert backends.parse_ultrasonic("Up stop distance: 10.0 cm") is None


def check_terminal():
    try:
        import serial  # noqa: F401 — the terminal needs pyserial; CI may not have it
    except ImportError:
        return "terminal skipped (pyserial missing)"
    terminal = load("lift_uno_position_terminal")

    class Conn:
        def __init__(self, lines):
            self.q = queue.Queue()
            for line in lines:
                self.q.put(line)

        def readline(self):
            try:
                return (self.q.get(timeout=0.05) + "\n").encode()
            except queue.Empty:
                return b""

    with contextlib.redirect_stdout(io.StringIO()):  # reader_loop 이 받은 줄을 화면에 찍는다
        reader_checks(terminal, Conn)
    return "terminal banner check"


def reader_checks(terminal, Conn):
    finished, banner_ok, idle = threading.Event(), threading.Event(), threading.Event()
    banner = ["Motor ready", backends.LIFT_FIRMWARE_ID, "Up stop distance: 10.0 cm", "Up start", "Up complete"]
    thread = threading.Thread(target=terminal.reader_loop, args=(Conn(banner), finished, banner_ok, idle),
                              daemon=True)
    thread.start()
    assert banner_ok.wait(2.0) and idle.wait(2.0)
    finished.set()
    thread.join(1.0)

    finished, banner_ok, idle = threading.Event(), threading.Event(), threading.Event()
    wrong = ["READY UNO_TB6600_POSITION_GUARD_V1", "Up stop distance: 10.0 cm"]
    thread = threading.Thread(target=terminal.reader_loop, args=(Conn(wrong), finished, banner_ok, idle),
                              daemon=True)
    thread.start()
    assert not banner_ok.wait(0.5), "terminal accepted a board without the lift_uno banner"
    finished.set()
    thread.join(1.0)


def main():
    check_adapter()
    check_firmware_source()
    note = check_terminal()
    print(f"PASS lift_uno: firmware source matches Jetson line contract; adapter u/d only; {note}")


if __name__ == "__main__":
    main()
