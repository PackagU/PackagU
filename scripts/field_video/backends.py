"""촬영 리허설 진행기의 장치 연결부.

실제 연결 3가지와, 같은 인터페이스의 가짜 연결(장비 없는 연습·오프라인 시험용)을 둔다.

- FieldStack : 기존 scripts/fieldctl 과 docker exec 로 Nav2·위치·정지·녹화·지도 전환
- ButtonWeb  : Jetson 버튼 웹사이트(8091) HTTP API. 팔 자세·누르기 값은 웹사이트가 가진 그대로
- LiftUno    : 지게팔 보드(firmware/lift_uno) 시리얼. 진행기가 끝까지 포트를 연다
- Ultrasonic : 아두이노에 연결한 초음파 거리. 지게팔 Uno 와 같은 포트(줄을 나눠 읽음) 또는 별도 포트

Jetson 호스트(Ubuntu 20.04, Python 3.8)에서 표준 라이브러리 + pyserial 만으로 돈다.
"""

import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
CONTAINER_HELPER_DIR = "/tmp"


class BackendError(RuntimeError):
    """장치·명령 실패. 메시지는 운영자에게 그대로 보인다."""


# ---------------------------------------------------------------- field stack
class FieldStack:
    """fieldctl 과 컨테이너 명령. 모든 출력은 화면과 단계 로그에 함께 남긴다."""

    def __init__(self, container=None, root=ROOT, output=None):
        self.container = container or os.environ.get("PACKAGU_CONTAINER_NAME", "ros2_humble")
        self.root = Path(root)
        self.fieldctl = str(self.root / "scripts" / "fieldctl")
        self.output = output or (lambda line: print(line, flush=True))
        self.env = dict(os.environ, PACKAGU_CONTAINER_NAME=self.container)
        self._cache = {}
        self.background = {}

    # -- low level
    def _stream(self, argv, timeout=None):
        """명령을 실행하며 줄 단위로 보여 준다. Ctrl+C 는 자식에게도 가므로 끝날 때까지 기다린다."""
        proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=self.env,
                                cwd=str(self.root), universal_newlines=True, bufsize=1)
        lines = []
        started = time.monotonic()
        try:
            for line in proc.stdout:
                line = line.rstrip("\n")
                lines.append(line)
                self.output("  │ " + line)
                if timeout and time.monotonic() - started > timeout:
                    proc.send_signal(signal.SIGINT)
            return proc.wait(), lines
        except KeyboardInterrupt:
            self.emergency_gate_stop()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.terminate()
            raise

    def emergency_gate_stop(self):
        """Ctrl+C 직후 다른 경로로 gate 정지를 먼저 건다. 새 세션이라 다음 Ctrl+C 에 끊기지 않는다."""
        self.output("  ■ Ctrl+C — gate 정지 요청 먼저 보냄")
        try:
            subprocess.run([self.fieldctl, "stop"], env=self.env, cwd=str(self.root), timeout=20,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        except (subprocess.TimeoutExpired, OSError):
            self.output("  !! gate 정지 확인 실패 — 물리 비상정지를 누르세요")

    def ctl(self, *args, timeout=None):
        rc, lines = self._stream([self.fieldctl, *[str(a) for a in args]], timeout=timeout)
        return rc, lines

    def exec_capture(self, command, timeout=30):
        argv = ["docker", "exec", "-i", "-w", "/ros2_ws", self.container, "bash", "-lc",
                "source /opt/ros/humble/setup.bash && source install/setup.bash && " + command]
        try:
            done = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  universal_newlines=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            raise BackendError(f"container command timed out: {command}")
        return done.returncode, done.stdout

    def container_running(self):
        done = subprocess.run(["docker", "ps", "--format", "{{.Names}}"], stdout=subprocess.PIPE,
                              universal_newlines=True)
        return self.container in done.stdout.split()

    def read_json(self, path):
        if path not in self._cache:
            rc, out = self.exec_capture(f"cat '{path}'")
            if rc != 0:
                raise BackendError(f"cannot read {path} in {self.container}")
            self._cache[path] = json.loads(out)
        return self._cache[path]

    def waypoint(self, name):
        data = self.read_json("/ros2_ws/maps/field/waypoints.json")
        try:
            wp = data["waypoints"][name]
        except KeyError:
            raise BackendError(f"waypoint '{name}' is not registered in waypoints.json")
        return {"floor": wp["floor"].upper(), "x": float(wp["x"]), "y": float(wp["y"]),
                "yaw": float(wp["yaw_rad"])}

    def pinned_map(self, floor):
        pins = self.read_json("/ros2_ws/maps/field/map_pins.json")
        try:
            return pins["floors"][floor]["yaml"]
        except KeyError:
            raise BackendError(f"no pinned map for {floor} in map_pins.json")

    # -- status
    def status(self):
        done = subprocess.run([self.fieldctl, "status"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              universal_newlines=True, env=self.env, cwd=str(self.root), timeout=40)
        first = next((l for l in done.stdout.splitlines() if l.startswith("base=")), "")
        info = dict(part.split("=", 1) for part in first.split() if "=" in part)
        info["raw"] = done.stdout
        return info

    def arm_node_running(self):
        return bool(self.node_list() & {"/arm_sequence", "/packagu_arm_sequence"})

    def node_list(self):
        rc, out = self.exec_capture("ros2 node list", timeout=20)
        return set(out.split()) if rc == 0 else set()

    # -- motion authority
    def stop(self):
        rc, _ = self.ctl("stop")
        if rc != 0:
            raise BackendError("software stop was not acknowledged — 물리 비상정지를 확인하세요")

    def resume(self):
        rc, _ = self.ctl("resume")
        if rc != 0:
            raise BackendError("resume failed (nav_safety_gate 응답 없음)")

    def cancel_all(self):
        self.ctl("cancel", "--all")

    def ensure_idle(self):
        """살아 있는 Nav2 goal 이 없고(있으면 취소 완료까지 대기) /cmd_vel 에 다른 발행자가 없음을 확인."""
        data = self._run_helper("ros_nav_idle.py", [], "NAV_IDLE_RESULT ")
        if not data.get("ok"):
            raise BackendError(f"not safe to drive directly: {data.get('reason')}")
        return data

    def goal(self, name, floor):
        rc, lines = self.ctl("goal", name, floor)
        status = next((l.split("status=")[-1] for l in lines if "goal result:" in l), None)
        return rc == 0, status or ("error" if rc else "unknown")

    def pose_set(self, x, y, yaw):
        rc, _ = self.ctl("pose", "set", f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}")
        if rc != 0:
            raise BackendError("AMCL did not accept the initial pose")

    def map_guard(self, floor):
        rc, _ = self.ctl("map", "guard", floor)
        return rc == 0

    # -- recording
    def record_start(self, name):
        rc, _ = self.ctl("record", "start", name)
        if rc != 0:
            raise BackendError("rosbag recorder did not start")
        rc, _ = self.ctl("record", "check")
        return rc == 0

    def record_stop(self):
        self.ctl("record", "stop")

    # -- helpers copied into the container (works even if its /ros2_ws/scripts is another checkout)
    def _helper(self, name):
        target = f"{CONTAINER_HELPER_DIR}/packagu_field_video_{name}"
        done = subprocess.run(["docker", "cp", str(HERE / name), f"{self.container}:{target}"],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        if done.returncode != 0:
            raise BackendError(f"docker cp {name} failed: {done.stdout.strip()}")
        return target

    def _run_helper(self, name, args, tag):
        target = self._helper(name)
        tty = ["-t"] if sys.stdin.isatty() and sys.stdout.isatty() else []
        argv = ["docker", "exec", "-i", *tty, "-w", "/ros2_ws", self.container, "bash", "-lc",
                "source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 "
                + target + " " + " ".join(args)]
        rc, lines = self._stream(argv)
        result = next((l.split(tag, 1)[1] for l in reversed(lines) if tag in l), None)
        data = json.loads(result) if result else {"ok": False, "reason": "no result line"}
        data["rc"] = rc
        return data

    def straight(self, distance, speed):
        data = self._run_helper("ros_straight_move.py",
                                [f"--distance {distance:.3f}", f"--speed {speed:.3f}"], "STRAIGHT_RESULT ")
        return bool(data.get("ok")), data

    def straight_until(self, max_distance, speed, check, poll=0.05):
        """최대 max_distance 까지 곧게 가다가 check() 가 'stop' 이면 멈춘다.

        check() 는 None(계속) / 'stop'(정상 정지) / 'fail:<이유>'(즉시 정지·실패)를 돌려준다.
        도우미는 표준입력 STOP 또는 EOF 로 멈추므로, 진행기가 죽어도 로봇은 선다.
        """
        target = self._helper("ros_straight_move.py")
        argv = ["docker", "exec", "-i", "-w", "/ros2_ws", self.container, "bash", "-lc",
                "source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 " + target
                + f" --distance {max_distance:.3f} --speed {speed:.3f} --stdin-stop"]
        proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                env=self.env, universal_newlines=True, bufsize=1)
        verdict = {"reason": None}
        done = threading.Event()

        def send_stop():
            try:
                proc.stdin.write("STOP\n")
                proc.stdin.flush()
            except (BrokenPipeError, ValueError, OSError):
                pass

        def monitor():
            while not done.is_set() and proc.poll() is None:
                result = check()
                if result:
                    verdict["reason"] = result
                    send_stop()
                    return
                time.sleep(poll)

        threading.Thread(target=monitor, daemon=True).start()
        lines = []
        try:
            for line in proc.stdout:
                line = line.rstrip("\n")
                lines.append(line)
                self.output("  │ " + line)
            proc.wait()
        except KeyboardInterrupt:
            self.emergency_gate_stop()  # 다른 경로로 gate 정지를 먼저
            send_stop()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.terminate()
            raise
        finally:
            done.set()
            try:
                proc.stdin.close()
            except (BrokenPipeError, OSError):
                pass
        result = next((l.split("STRAIGHT_RESULT ", 1)[1] for l in reversed(lines) if "STRAIGHT_RESULT " in l), None)
        data = json.loads(result) if result else {"ok": False, "reason": "no result line"}
        data["rc"] = proc.returncode
        if verdict["reason"] and verdict["reason"].startswith("fail:"):
            data["ok"] = False
            data["reason"] = verdict["reason"][5:]
        return bool(data.get("ok")), data

    def load_map(self, yaml_path):
        data = self._run_helper("ros_map_switch.py", [f"--map-yaml '{yaml_path}'"], "MAP_SWITCH_RESULT ")
        return bool(data.get("ok")), data

    def clear_costmaps(self):
        data = self._run_helper("ros_map_switch.py", ["--clear-only"], "MAP_SWITCH_RESULT ")
        return bool(data.get("ok")), data

    def teleop(self):
        """키보드 텔레옵을 이 터미널에서 직접 연다. Ctrl+C 는 텔레옵 종료로만 쓰인다.

        scripts/teleop.sh 는 Nav2(controller_server)가 있으면 거부한다. 진행기는 Nav2 를
        켜 둔 채 goal 이 없을 때만(cancel 후) 텔레옵을 열고, 닫힌 뒤에만 다음 goal 을 보낸다.
        keyboard_teleop 은 대기 중 0 을 계속 보내지 않으므로(should_publish) Nav2 와 섞이지 않는다.
        """
        if "/packagu_keyboard_teleop" in self.node_list():
            raise BackendError("another keyboard_teleop is already running; close it first")
        argv = ["docker", "exec", "-it", "-w", "/ros2_ws", self.container, "bash", "-lc",
                "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run drive_pkg keyboard_teleop"]
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            rc = subprocess.call(argv, env=self.env)
        finally:
            signal.signal(signal.SIGINT, previous)
        for _ in range(15):
            if "/packagu_keyboard_teleop" not in self.node_list():
                return rc
            time.sleep(0.3)
        raise BackendError("keyboard_teleop is still alive after exit; stop it before the next goal")

    # -- stack start (only when missing; never stopped automatically so retakes stay fast)
    def start_background(self, key, args, ready_pattern, timeout, log_dir):
        log_path = Path(log_dir) / f"{key}.log"
        handle = open(log_path, "w", encoding="utf-8")
        proc = subprocess.Popen([self.fieldctl, *args], stdout=handle, stderr=subprocess.STDOUT,
                                env=self.env, cwd=str(self.root), start_new_session=True)
        self.background[key] = proc
        pattern = re.compile(ready_pattern)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise BackendError(f"{key} exited early (rc={proc.returncode}); see {log_path}")
            text = log_path.read_text(encoding="utf-8", errors="replace")
            if pattern.search(text):
                return str(log_path)
            time.sleep(1.0)
        raise BackendError(f"{key} not ready within {timeout} s; see {log_path}")

    def stop_navigation(self):
        rc, _ = self.ctl("nav", "stop")
        if rc != 0:
            raise BackendError("nav stop failed")


# ---------------------------------------------------------------- button website
class ButtonWeb:
    """버튼 웹사이트 HTTP API. 인식·누름 자세 계산·팔 구동은 전부 웹사이트(=Jetson 코드)가 한다."""

    def __init__(self, base_url, request_timeout=5.0):
        self.base = base_url.rstrip("/")
        self.request_timeout = request_timeout

    def _request(self, path, data=None):
        body = None if data is None else json.dumps(data).encode()
        request = urllib.request.Request(self.base + path, data=body, method="GET" if data is None else "POST")
        if body is not None:
            request.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(request, timeout=self.request_timeout) as response:
                return json.loads(response.read().decode())
        except urllib.error.HTTPError as exc:
            try:
                message = json.loads(exc.read().decode()).get("error", str(exc))
            except ValueError:
                message = str(exc)
            raise BackendError(f"button web {path}: {message}")
        except (urllib.error.URLError, OSError, ValueError) as exc:
            raise BackendError(f"button web {self.base} unreachable: {exc}")

    def state(self):
        return self._request("/api/state")

    def set_mission(self, stage, direction):
        self._request("/api/mission", {"stage": stage, "direction": direction})

    def press(self, token):
        return self._request("/api/press", {"token": token})

    def stop(self):
        try:
            self._request("/api/stop", {})
        except BackendError:
            pass


# ---------------------------------------------------------------- lift Uno
# 지게팔 펌웨어 firmware/lift_uno/lift_uno.ino 의 시리얼 약속. 문구는 펌웨어와 글자까지 같아야 한다
# (scripts/test_lift_uno_contract.py 가 소스와 대조한다).
LIFT_FIRMWARE_ID = "u: Up (ultrasonic stop)"  # 부팅 안내 줄. 이 펌웨어만 낸다
LIFT_STOP_DISTANCE_RE = re.compile(r"^Up stop distance:\s*(\d+(?:\.\d+)?)\s*cm\b")
LIFT_ECHO_RE = re.compile(r"^Echo:\s*\d+\s*us\s*\|\s*Distance:\s*(-?\d+(?:\.\d+)?)\s*cm\b")
LIFT_LINES = {
    "up": {"start": "Up start", "complete": "Up complete",
           "height_limit": "Up stopped: distance below stop threshold"},
    "down": {"start": "Down start", "complete": "Down complete"},
}
LIFT_END_LINES = ("Up complete", "Up stopped: distance below stop threshold", "Down complete")
LIFT_PULSES_PER_COMMAND = 12800
LIFT_ACK_TIMEOUT_SEC = 2.0
LIFT_BANNER_TIMEOUT_SEC = 6.0
LIFT_NO_STOP_NOTE = ("지게팔 펌웨어에는 정지 명령이 없어 이동은 펌웨어가 끝낼 때까지(최대 약 6 초) 계속된다 "
                     "— 위험하면 물리 비상정지")


class LiftUno:
    """지게팔 보드(lift_uno 펌웨어). 포트를 여는 순간 보드가 리셋되므로 진행 내내 연다.

    펌웨어는 `u`·`d` 한 번에 12800 펄스를 보내고, 상승 중 초음파가 기준(10 cm) 미만이면 멈춘다(박스 높이 제한).
    정지·상태 명령과 위치 기억이 없다. 그래서 끝 줄을 볼 때까지 다음 명령을 보내지 않고, 끝 줄을 못 보면
    moving 을 유지해 이후 지게팔 동작을 막는다.
    """

    def __init__(self, port, output=None, line_hook=None):
        self.port = port
        self.output = output or (lambda line: print(line, flush=True))
        self.line_hook = line_hook  # 같은 보드에 붙은 초음파 줄을 가져간다
        self.conn = None
        self.lines = []
        self.lock = threading.Lock()
        self.moving = False  # 명령을 보냈지만 끝 줄을 아직 못 봄 → 다음 지게팔 명령 금지
        self.command_at = None
        self.stop_cm = None  # 부팅 안내 줄의 높이 제한 거리
        self.last_cm = None  # 이번 상승 중 마지막 측정 거리
        self._stop = threading.Event()

    def open(self):
        import serial  # pyserial (Jetson 호스트에 이미 있음: lift_uno_position_terminal.py 와 같음)

        import fcntl

        if not os.path.exists(self.port):
            raise BackendError(f"lift port not found: {self.port}")
        # lift_uno_adapter.py 와 같은 잠금 파일 + 포트 독점: 지게팔 터미널·진행기 중복 실행 거부
        self._lock = open("/tmp/packagu_lift_uno.lock", "w", encoding="ascii")
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._lock.close()
            raise BackendError("another lift serial owner is active (lift terminal or second rehearsal)")
        try:
            self.conn = serial.Serial(self.port, 115200, timeout=0.2, exclusive=True)
        except Exception as exc:  # noqa: BLE001 — busy port or permission
            self._lock.close()
            raise BackendError(f"cannot open lift port {self.port}: {exc}")
        opened = time.monotonic()
        threading.Thread(target=self._reader, daemon=True).start()
        try:
            self.check_banner(opened)
        except BackendError:
            self.close()
            raise

    def check_banner(self, since, timeout=LIFT_BANNER_TIMEOUT_SEC):
        """포트를 연 뒤(보드 리셋) 부팅 안내 줄로 펌웨어를 확인하고 높이 제한 거리를 읽는다."""
        line = self._wait_for(since, lambda l: LIFT_STOP_DISTANCE_RE.match(l) is not None, timeout)
        with self.lock:
            seen = [l for stamp, l in self.lines if stamp >= since]
        if line is None or LIFT_FIRMWARE_ID not in seen:
            raise BackendError("lift firmware banner not seen (wrong board or firmware?) — "
                               "firmware/lift_uno 를 올렸는지 확인")
        self.stop_cm = float(LIFT_STOP_DISTANCE_RE.match(line).group(1))
        return self.stop_cm

    def _reader(self):
        while not self._stop.is_set():
            try:
                raw = self.conn.readline()
            except Exception as exc:  # noqa: BLE001 — serial errors end the reader
                self._append(f"SERIAL_ERROR {exc}")
                return
            if raw:
                self._append(raw.decode("utf-8", errors="replace").strip())

    def _append(self, line):
        if self.line_hook:
            self.line_hook(line)
        echo = LIFT_ECHO_RE.match(line)
        if echo:
            self.last_cm = float(echo.group(1))
            return  # 상승 중 약 10 Hz 거리 줄은 화면에 찍지 않는다. 진행 표시에 마지막 값만 보인다
        if self.line_hook and is_ultrasonic_only(line):
            return  # 순수 초음파 줄만 숨긴다. 시작·완료·정지 줄은 남긴다
        with self.lock:
            self.lines.append((time.monotonic(), line))
        self.output("  │ UNO> " + line)

    def _send(self, command):
        if self.conn is None:
            raise BackendError("lift port is not open")
        try:
            self.conn.write(command.encode("ascii"))  # 펌웨어는 한 글자씩 읽는다. 줄바꿈은 보내지 않는다
            self.conn.flush()
        except Exception as exc:  # noqa: BLE001 — serial errors become operator-visible failures
            raise BackendError(f"lift serial write failed: {exc}")

    def _wait_for(self, since, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                for stamp, line in self.lines:
                    if stamp >= since and predicate(line):
                        return line
            time.sleep(0.05)
        return None

    def move(self, action, timeout, progress=None):
        """`u` 또는 `d` 한 번(12800 펄스). 반환: {"result": "complete" | "height_limit", "seconds", "last_cm"}.

        끝 줄을 못 보면 BackendError. 이때 moving 은 True 로 남는다(펌웨어가 아직 움직일 수 있음).
        """
        if self.moving:
            raise BackendError("lift end of the previous command was not seen — not sending another command")
        lines = LIFT_LINES[action]
        ends = {text: result for result, text in lines.items() if result != "start"}
        command = "u" if action == "up" else "d"
        self.last_cm = None
        since = time.monotonic()
        self.moving, self.command_at = True, since
        try:
            self._send(command)
            ack = self._wait_for(since, lambda l: l == lines["start"] or l.startswith("SERIAL_ERROR"),
                                 LIFT_ACK_TIMEOUT_SEC)
            if ack is None or ack.startswith("SERIAL_ERROR"):
                raise BackendError(f"lift did not acknowledge '{command}' ({ack or repr(lines['start']) + ' not seen'})")
            while True:
                elapsed = time.monotonic() - since
                if elapsed >= timeout:
                    raise BackendError(f"lift {action} end line not seen within {timeout:.0f} s")
                line = self._wait_for(since, lambda l: l in ends or l.startswith("SERIAL_ERROR"),
                                      min(1.0, timeout - elapsed))
                if line is not None:
                    if line.startswith("SERIAL_ERROR"):
                        raise BackendError(f"lift {action}: {line}")
                    break
                if progress:
                    progress(time.monotonic() - since, self.last_cm)
        except BackendError as exc:
            # moving 은 그대로 둔다. 정지 명령이 없으니 wait_until_stopped 가 끝 줄로 확인할 때까지 지게팔 동작 금지
            raise BackendError(f"{exc} — {LIFT_NO_STOP_NOTE}") from None
        self.moving = False
        return {"result": ends[line], "seconds": time.monotonic() - since, "last_cm": self.last_cm}

    def wait_until_stopped(self, timeout):
        """마지막 명령의 끝 줄을 timeout 초까지 기다린다. 봤으면(또는 움직이지 않았으면) True.

        펌웨어에 정지 명령이 없으므로 멈추게 하지는 못하고, 스스로 끝냈는지만 확인한다.
        """
        if not self.moving:
            return True
        if self._wait_for(self.command_at, lambda l: l in LIFT_END_LINES, timeout) is not None:
            self.moving = False
        return not self.moving

    def close(self):
        self._stop.set()
        if self.conn is not None:
            self.conn.close()
        if getattr(self, "_lock", None) is not None:
            self._lock.close()


# ---------------------------------------------------------------- ultrasonic
US_RE = re.compile(r"(?:\bUS\b|ultra\w*|sonar|dist(?:ance)?(?:_cm)?)\s*[=:]?\s*(-?\d+(?:\.\d+)?)", re.I)


ULTRASONIC_ONLY_RE = re.compile(r"^(?:US|DIST(?:ANCE)?|ULTRA\w*|SONAR)\b", re.I)


def is_ultrasonic_only(line):
    """초음파 전용 줄(US/DIST/ULTRASONIC/SONAR 로 시작)인지. 지게팔 응답 줄은 False."""
    return bool(ULTRASONIC_ONLY_RE.match(line.strip()))


def parse_ultrasonic(line):
    """아두이노 거리 줄에서 cm 값을 꺼낸다. 예: 'US distance_cm=23.4', 'DIST:23', 'ultrasonic 23.4'.

    지금 지게팔 펌웨어(lift_uno)는 상승 중에만 'Echo: <µs> us | Distance: <cm> cm' 를 낸다. 대기 중 상시 출력이
    없어서 S03 접근은 이 펌웨어로는 거리값을 받지 못한다(S03 방식 미정, 2026-10-29).
    부팅 안내 줄 'Up stop distance: 10.0 cm' 는 설정값이라 거리로 읽지 않는다.
    STATUS 줄 안의 키(position_pulses 등)는 거리로 오인하지 않는다.
    """
    if LIFT_STOP_DISTANCE_RE.match(line):
        return None
    echo = LIFT_ECHO_RE.match(line)
    if echo:
        match = echo
    elif line.startswith("STATUS"):
        match = re.search(r"\b(?:us_cm|distance_cm|ultrasonic_cm)=(-?\d+(?:\.\d+)?)", line)
    else:
        match = US_RE.search(line)
    if not match:
        return None
    value = float(match.group(1))
    return value if 0.0 < value < 1000.0 else None


class Ultrasonic:
    """최신 거리값과 나이(초)를 기억한다. 값이 오래되면 쓰지 않는다(fail-closed)."""

    def __init__(self, port="", stale_sec=0.5, output=None, clock=time.monotonic):
        self.port = port  # 비어 있으면 지게팔 Uno 와 같은 포트(line_hook 으로 받음)
        self.stale_sec = float(stale_sec)
        self.output = output or (lambda line: print(line, flush=True))
        self.clock = clock
        self.value, self.at = None, 0.0
        self.conn = None
        self._stop = threading.Event()

    def feed(self, line):
        value = parse_ultrasonic(line)
        if value is None:
            return False
        self.value, self.at = value, self.clock()
        return True

    def reading(self):
        """(cm 또는 None, 나이 초)."""
        if self.value is None:
            return None, float("inf")
        return self.value, self.clock() - self.at

    def fresh(self):
        value, age = self.reading()
        return value is not None and age <= self.stale_sec

    def open(self):
        if not self.port:
            return  # shared with the lift Uno
        import serial

        if not os.path.exists(self.port):
            raise BackendError(f"ultrasonic port not found: {self.port}")
        self.conn = serial.Serial(self.port, 115200, timeout=0.2)
        time.sleep(2.0)
        self.conn.reset_input_buffer()

        def reader():
            while not self._stop.is_set():
                try:
                    raw = self.conn.readline()
                except Exception as exc:  # noqa: BLE001
                    self.output(f"  │ US serial error {exc}")
                    return
                if raw:
                    self.feed(raw.decode("utf-8", errors="replace").strip())

        threading.Thread(target=reader, daemon=True).start()

    def close(self):
        self._stop.set()
        if self.conn is not None:
            self.conn.close()


# ---------------------------------------------------------------- fakes
class _FailPlan:
    """'goal:f1_locker': 1 처럼 키별로 앞에서 N번 실패시킨다."""

    def __init__(self, failures=None):
        self.failures = dict(failures or {})

    def hit(self, key):
        left = self.failures.get(key, 0)
        if left > 0:
            self.failures[key] = left - 1
            return True
        return False


class FakeField:
    def __init__(self, failures=None, base_running=True, nav_running=True, loaded="F1", teleop_rc=0,
                 arm_node=False):
        self.calls = []
        self.arm_node = arm_node
        self.teleop_rc = teleop_rc
        self.fail = _FailPlan(failures)
        self.base_running, self.nav_running, self.loaded = base_running, nav_running, loaded
        self.stopped = True
        self.background = {}
        self.waypoints = {
            "f1_initial_test": ("F1", -4.518, 2.179, -1.701), "f1_locker": ("F1", -5.125, -8.625, -1.69),
            "f1_elevator_entry": ("F1", -8.725, -0.175, -2.222665),
            "f1_elevator_exit": ("F1", -8.725, -0.175, 0.918927),
            "f2_elevator_exit": ("F2", -11.375, -2.525, -0.828849),
            "f2_delivery_left_room4": ("F2", -24.725, -17.975, -2.3963),
            "f2_elevator_staging_v1": ("F2", -11.175, -3.075, 2.312744),
        }

    def _log(self, *call):
        self.calls.append(call)

    def container_running(self):
        return True

    def arm_node_running(self):
        return self.arm_node

    def waypoint(self, name):
        if name not in self.waypoints:
            raise BackendError(f"waypoint '{name}' is not registered in waypoints.json")
        floor, x, y, yaw = self.waypoints[name]
        return {"floor": floor, "x": x, "y": y, "yaw": yaw}

    def pinned_map(self, floor):
        return f"/ros2_ws/maps/field/{floor.lower()}/pinned.yaml"

    def status(self):
        return {"base": "running" if self.base_running else "stopped", "mapping": "stopped",
                "navigation": "running" if self.nav_running else "stopped"}

    def stop(self):
        self._log("stop")
        self.stopped = True

    def resume(self):
        self._log("resume")
        self.stopped = False

    def cancel_all(self):
        self._log("cancel")

    def ensure_idle(self):
        self._log("idle")
        if self.fail.hit("idle"):
            raise BackendError("not safe to drive directly: 1 Nav2 goal(s) still active after cancel")
        return {"ok": True}

    def goal(self, name, floor):
        self._log("goal", name, floor)
        if self.stopped:
            return False, "gate stopped"
        if self.waypoint(name)["floor"] != self.loaded:
            return False, "wrong map loaded"
        if self.fail.hit("goal:" + name):
            return False, "ABORTED"
        return True, "SUCCEEDED"

    def pose_set(self, x, y, yaw):
        self._log("pose", round(x, 3), round(y, 3), round(yaw, 3))

    def map_guard(self, floor):
        self._log("guard", floor)
        return floor == self.loaded

    def record_start(self, name):
        self._log("record_start", name)
        return True

    def record_stop(self):
        self._log("record_stop")

    def straight(self, distance, speed):
        self._log("straight", distance)
        if self.stopped:
            return False, {"reason": "gate stopped"}
        if self.fail.hit("straight"):
            return False, {"reason": "timeout"}
        return True, {"traveled_m": abs(distance)}

    def straight_until(self, max_distance, speed, check, poll=0.0):
        self._log("straight_until", max_distance)
        if self.stopped:
            return False, {"reason": "gate stopped"}
        for step in range(200):
            result = check()
            if result == "stop":
                return True, {"reason": "stopped by sensor", "traveled_m": round(step * 0.01, 2)}
            if result:
                return False, {"reason": result[5:]}
            if step * 0.01 >= max_distance:
                break
        return False, {"reason": "max distance reached before sensor stop"}

    def load_map(self, yaml_path):
        self._log("load_map", yaml_path)
        if self.fail.hit("load_map"):
            return False, {"reason": "load_map failed"}
        self.loaded = yaml_path.split("/")[-2].upper()
        return True, {}

    def clear_costmaps(self):
        self._log("clear")
        if self.fail.hit("clear"):
            return False, {"reason": "clear service unavailable"}
        return True, {}

    def teleop(self):
        self._log("teleop", "stopped" if self.stopped else "released")
        return self.teleop_rc

    def start_background(self, key, args, ready_pattern, timeout, log_dir):
        self._log("start", key, *args)
        if key == "base":
            self.base_running = True
        else:
            self.nav_running, self.loaded = True, args[-1]
        return "fake.log"

    def stop_navigation(self):
        self._log("nav_stop")
        self.nav_running = False


class FakeWeb:
    MISSIONS = {("hall", "up"): "UP", ("hall", "down"): "DOWN", ("car", "up"): "4", ("car", "down"): "1"}

    def __init__(self, failures=None, physical=True, ros_arm=False):
        self.calls = []
        self.ros_arm = ros_arm
        self.fail = _FailPlan(failures)
        self.target = "UP"
        self.physical = physical
        self.busy_polls = 0
        self.last_error = None

    def state(self):
        ready = None
        token = None
        if not self.fail.hit("ready:" + self.target):
            ready = {"label": self.target, "press": {"001": 1500}, "reason": None}
            token = "tok-" + self.target
        busy = self.busy_polls > 0
        if busy:
            self.busy_polls -= 1
        message = self.last_error or ("버튼 %s 누르기 완료" % self.target)
        return {"error": None, "frames": 100, "execute": self.physical, "mode": "fake", "ros_arm": self.ros_arm,
                "target": self.target, "ready": None if busy else ready, "token": None if busy else token,
                "busy": busy, "message": message, "floor": {"floor": "4", "score": 0.9}}

    def set_mission(self, stage, direction):
        self.calls.append(("mission", stage, direction))
        self.target = self.MISSIONS[stage, direction]

    def press(self, token):
        self.calls.append(("press", token))
        if token != "tok-" + self.target:
            raise BackendError("button web /api/press: 인식이 변경됐거나 준비되지 않았습니다")
        self.last_error = "중지 / 오류: fake" if self.fail.hit("press:" + self.target) else None
        self.busy_polls = 2
        return {"started": True}

    def stop(self):
        self.calls.append(("stop",))


class FakeLift:
    """lift_uno 펌웨어 흉내. 실패 주입: move:up / move:down = 끝 줄 못 봄, settle = 끝 줄을 끝내 못 봄,
    height = 상승이 높이 제한으로 멈춤."""

    def __init__(self, failures=None, stop_cm=10.0):
        self.calls = []
        self.fail = _FailPlan(failures)
        self.stop_cm = stop_cm
        self.moving = False

    def open(self):
        self.calls.append(("open",))

    def move(self, action, timeout, progress=None):
        self.calls.append(("move", action))
        if self.moving:
            raise BackendError("lift end of the previous command was not seen — not sending another command")
        if self.fail.hit("move:" + action):
            self.moving = True
            raise BackendError(f"lift {action} end line not seen: fake — {LIFT_NO_STOP_NOTE}")
        if progress:
            progress(1.0, 30.0 if action == "up" else None)
        if action == "up" and self.fail.hit("height"):
            return {"result": "height_limit", "seconds": 1.2, "last_cm": 9.5}
        return {"result": "complete", "seconds": 4.1 if action == "up" else 4.0,
                "last_cm": 30.0 if action == "up" else None}

    def wait_until_stopped(self, timeout):
        self.calls.append(("settle",))
        if self.moving and not self.fail.hit("settle"):
            self.moving = False
        return not self.moving

    def close(self):
        self.calls.append(("close",))


class FakeUltrasonic:
    """읽을 때마다 거리가 줄어드는 가짜 센서. values 를 다 쓰면 마지막 값 유지."""

    def __init__(self, values=None, stale=False, stale_sec=0.5):
        self.values = list(values if values is not None else [40, 30, 20, 12, 8, 6])
        self.stale = stale
        self.stale_sec = stale_sec
        self.port = ""

    def reading(self):
        if self.stale or not self.values:
            return None, float("inf")
        value = self.values.pop(0) if len(self.values) > 1 else self.values[0]
        return float(value), 0.05

    def fresh(self):
        return not self.stale

    def open(self):
        pass

    def close(self):
        pass
