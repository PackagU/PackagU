#!/usr/bin/env python3
"""Guarded terminal for the lift_uno sketch (firmware/lift_uno).

u = up 12800 pulses (the firmware stops early when the ultrasonic reads below its height limit),
d = down 12800 pulses. The firmware has no stop/status command and does not read serial while it
moves, so this terminal checks the boot banner, sends one command at a time, and waits for that
command's end line before it accepts the next one. The file name is kept for existing checklists.
"""

import argparse
import os
import sys
import threading
import time

import serial
from serial.tools import list_ports

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from field_video.backends import (  # noqa: E402 — 펌웨어 문구는 진행기와 한 곳에서 관리
    LIFT_END_LINES, LIFT_FIRMWARE_ID, LIFT_NO_STOP_NOTE, LIFT_STOP_DISTANCE_RE)

BANNER_TIMEOUT_SEC = 6.0


LIDAR_VID_PID = (0x10C4, 0xEA60)
KNOWN_ARDUINO_VIDS = {0x2341, 0x2A03}


def available_ports():
    return list(list_ports.comports())


def port_line(port):
    vid_pid = "unknown"
    if port.vid is not None and port.pid is not None:
        vid_pid = f"{port.vid:04x}:{port.pid:04x}"
    return f"{port.device} vid:pid={vid_pid} serial={port.serial_number or 'unknown'} {port.description}"


def print_ports(ports):
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(port_line(port))


def choose_port(requested, allow_unknown):
    ports = available_ports()
    if requested:
        requested_real = os.path.realpath(requested)
        matches = [port for port in ports if os.path.realpath(port.device) == requested_real]
        if not matches:
            raise RuntimeError(f"Requested port is not present: {requested}")
        selected = matches[0]
    else:
        matches = [port for port in ports if port.vid in KNOWN_ARDUINO_VIDS]
        if len(matches) != 1:
            raise RuntimeError("Expected exactly one known Arduino port; pass --port after checking --list")
        selected = matches[0]

    if (selected.vid, selected.pid) == LIDAR_VID_PID:
        raise RuntimeError("Refusing known CP2102 LiDAR port; do not use it for the lift Arduino")
    if selected.vid not in KNOWN_ARDUINO_VIDS and not allow_unknown:
        raise RuntimeError("Unknown USB serial adapter; verify the board, then add --allow-unknown")
    return requested if requested else selected.device


def reader_loop(connection, finished, banner_ok, idle):
    firmware_seen = False
    while not finished.is_set():
        try:
            raw = connection.readline()
        except serial.SerialException as exc:
            print(f"\nSERIAL ERROR: {exc}")
            finished.set()
            return
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").rstrip()
        print(f"\nUNO> {line}")
        if line == LIFT_FIRMWARE_ID:
            firmware_seen = True
        elif firmware_seen and LIFT_STOP_DISTANCE_RE.match(line):
            banner_ok.set()
        elif line in LIFT_END_LINES:
            idle.set()


def run_terminal(device):
    if not os.access(device, os.R_OK | os.W_OK):
        username = os.environ.get("USER", "<user>")
        raise RuntimeError(
            f"No read/write permission for {device}. "
            f"Run 'sudo usermod -aG dialout {username}', then fully reconnect SSH."
        )
    print(f"Opening {device} at 115200 baud. Opening USB resets the board.")
    print("Commands: u=up 12800 pulses (ultrasonic height limit), d=down 12800 pulses, q=quit")
    print(f"No stop command: {LIFT_NO_STOP_NOTE}")
    # 2026-10-21: 촬영 리허설 진행기·lift_uno_adapter.py 와 같은 잠금. 같은 보드를 두 프로그램이 열지 않는다.
    import fcntl
    lock = open("/tmp/packagu_lift_uno.lock", "w", encoding="ascii")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError("Another lift serial owner is active (rehearsal runner or adapter). Close it first.")
    with lock, serial.Serial(device, 115200, timeout=0.2, exclusive=True) as connection:
        finished, banner_ok, idle = threading.Event(), threading.Event(), threading.Event()
        idle.set()
        reader = threading.Thread(target=reader_loop, args=(connection, finished, banner_ok, idle), daemon=True)
        reader.start()
        if not banner_ok.wait(BANNER_TIMEOUT_SEC):
            finished.set()
            reader.join(timeout=1.0)
            raise RuntimeError("lift_uno boot banner not seen: wrong board or firmware. No command was sent.")

        try:
            while not finished.is_set():
                command = input("lift> ").strip()
                if command.lower() == "q":
                    break
                if command == "":
                    continue
                if command.lower() not in {"u", "d"}:
                    print("Rejected locally. lift_uno accepts only u or d (no stop/status/position). q quits.")
                    continue
                if not idle.is_set():
                    print("Rejected locally: the previous move has not printed its end line yet.")
                    continue
                idle.clear()
                connection.write(command.lower().encode("ascii"))
                connection.flush()
        finally:
            finished.set()
            reader.join(timeout=1.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true", help="list ports without opening one")
    parser.add_argument("--port", help="exact Arduino serial device, for example /dev/ttyACM0")
    parser.add_argument("--allow-unknown", action="store_true", help="allow a verified clone USB adapter")
    parser.add_argument("--execute", action="store_true", help="open the port for an intentional live test")
    args = parser.parse_args()

    if args.list:
        print_ports(available_ports())
        return 0
    if not args.execute:
        print("Dry safety mode: no serial port opened. Use --list first, then --execute --port DEVICE.")
        return 0

    try:
        device = choose_port(args.port, args.allow_unknown)
        run_terminal(device)
    except (RuntimeError, serial.SerialException) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
