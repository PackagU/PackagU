#!/usr/bin/env python3
"""직진 전용 택배 집기 — 사람이 "곧게 가면 포크가 들어가는" 위치에 놓은 뒤 Jetson 이 한 명령으로 판단·실행한다.

    ~/pickup_venv/bin/python apps/pickup_real/straight_pickup.py              # 택배가 올라간 팔레트(지금)
    ~/pickup_venv/bin/python apps/pickup_real/straight_pickup.py --target-label 402   # 송장 숫자(시연)
    ~/pickup_venv/bin/python apps/pickup_real/straight_pickup.py --plan-only  # 움직임 없이 점검·기준 기록·계획

순서 (단계마다 사람 승인 없음. 허용 범위를 벗어나면 그 자리에서 멈추고 이유를 출력한다)
  1 PREFLIGHT  카메라 새 영상·fps, 라이다/odom 주기·나이, 구동 준비, /cmd_vel 다른 발행자 없음, 후방 여유, 지게팔 포트.
               이전 inserted 상태·목표 좌표는 무효로 기록하고 쓰지 않는다
  2 REFERENCE  팔 보기 자세 B → 목표 팔레트(택배 있는 쪽 / 송장 숫자) 선택 → 지금 정면 배치를 카메라 기준으로 기록.
               이때부터 끝날 때까지 카메라 원본 화면을 run_dir/frames 에 계속 저장(straight.record_hz)
               사람이 맞춘 배치 = 앞면 기울기 0 으로 보고 카메라 roll 을 역산(카메라는 포크 중심보다 오른쪽 약 11 cm 이라
               팔레트가 화면 왼쪽에 보이는 것이 정상. 화면 가운데로 돌리지 않는다). teach_tape.json 은 바꾸지 않는다
  3 APPROACH   직진만(각속도 0). 두 표식이 보이는 동안 구간마다 다시 보고 기준 대비 좌우·기울기·거리 변화를 확인,
               영상 거리 / 실제 odom 비로 남은 구간을 보정. 보이지 않는 마지막 구간은 실제 odom 으로만 간다
               (회전·후진·재정렬 없음 — 벗어나면 정지)
  4 FRONT VIEW 팔을 정면 자세(camera_views.json front_view, 이후 버튼 누르기용)로
  5 LIFT       지게팔 부팅 줄 확인 → `u` 를 초음파 정지까지 반복(상한 있음). 상승 판정은 보드 완료 응답이 아니라
               초음파 거리: 시작 때 10 cm 이상(또는 무응답) → 상승 중 10 cm 미만 정지. 상승 시간·펄스를 기록해
               "운반 높이 - 진입 높이"를 저장(다음 집기 때 그만큼 내림)
  6 RETREAT    후방 라이다 여유 확인 → 실제 odom 으로 곧게 후진
모든 이동 명령은 /cmd_vel → nav_safety_gate → OpenCR 경로(센서 끊김 차단 유지). 끝나면 gate 소프트웨어 정지.
로봇을 움직이는 실행은 사용자가 현장에서 지시할 때만 한다(AGENTS.md §0.4).
"""
import argparse
import json
import math
import subprocess
import sys
import time
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from geometry import plan_moves, robot_from_camera, simulate_moves, wrap  # noqa: E402
from straight_geometry import markers_in_robot, roll_for_square_face, visible_after  # noqa: E402
from parcel_target import pixel_bearing_deg  # noqa: E402

LOG_ROOT = ROOT / "logs" / "pickup_real"
STATE_FILE = LOG_ROOT / "state.json"
HEIGHT_FILE = LOG_ROOT / "lift_height_reference.json"
REFERENCE_LATEST = LOG_ROOT / "straight_reference_latest.json"
LIFT_PULSES = 12800
CHECK_EVERY = 320


class Stop(RuntimeError):
    """안전 정지. 이후 단계(상승·후진)는 하지 않는다."""


# ===================================================================== real devices
NAV2_CMD_VEL_NODES = ("behavior_server", "velocity_smoother", "controller_server")


def other_cmd_vel_publishers(names, cfg):
    """/cmd_vel 발행자 중 허용 안 된 것. base.allow_idle_nav2 면 Nav2 노드는 빼고 본다
    (대신 점검 동안 /cmd_vel 메시지가 0 이어야 한다 — Nav2 가 목표 없이 쉬는 중)."""
    allow = NAV2_CMD_VEL_NODES if cfg.get("base", {}).get("allow_idle_nav2") else ()
    return [n for n in names if n not in allow]


class RealStraightBase:
    """기존 RealBase(노드 확인·/cmd_vel 발행자 확인·gate resume/stop) + 직진 전용 helper(ros_straight_odom.py)."""

    def __init__(self, cfg, log):
        from pickup_real import RealBase
        self.inner = RealBase(cfg, log)
        self.stack = self.inner.stack
        self.log = log
        self.ready = False

    def _helper(self, args):
        target = "/tmp/packagu_pickup_straight_odom.py"
        done = subprocess.run(["docker", "cp", str(HERE / "ros_straight_odom.py"), "%s:%s" % (self.stack.container, target)],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        if done.returncode != 0:
            raise Stop("docker cp ros_straight_odom.py failed: " + done.stdout.strip())
        argv = ["docker", "exec", "-i", "-w", "/ros2_ws", self.stack.container, "bash", "-lc",
                "source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 " + target + " " + args]
        rc, lines = self.stack._stream(argv)
        line = next((l.split("STRAIGHT_ODOM_RESULT ", 1)[1] for l in reversed(lines) if "STRAIGHT_ODOM_RESULT " in l), None)
        data = json.loads(line) if line else {"ok": False, "reason": "no result line"}
        data["rc"] = rc
        return data

    def probe(self):
        if not self.stack.container_running():
            raise Stop("container %s not running" % self.stack.container)
        return self._helper("--probe --probe-seconds 4")

    def prepare(self):
        """주행 준비 한 번에(헬퍼 --prepare: 노드·/cmd_vel 확인 + gate 해제). 기존 RealBase.prepare 는 ros2 CLI 세 번으로
        Jetson 부하 속 35 초 걸렸다(192호 2026-11-01)."""
        allow = " --allow-idle-nav2" if self.inner.cfg.get("allow_idle_nav2") else ""
        data = self._helper("--prepare" + allow)
        self.log("base", action="prepare", **{k: data.get(k) for k in ("ok", "reason", "total_s", "released_s")})
        if not data.get("ok"):
            raise Stop("drive prepare: %s" % data.get("reason"))
        self.inner.ready = True
        self.ready = True

    def straight(self, distance, speed, max_yaw_deg, max_lateral_m, hold_heading=False):
        return self._helper("--distance %.4f --speed %.3f --max-yaw-deg %.2f --max-lateral-m %.4f%s"
                            % (distance, speed, max_yaw_deg, max_lateral_m, " --hold-heading" if hold_heading else ""))

    def turn(self, rad):
        from pickup_real import Stop as PickupStop
        try:
            return self.inner.turn(rad)
        except PickupStop as exc:
            raise Stop(str(exc))

    def stop(self):
        self.inner.stop()


class RealCamera:
    def __init__(self, cfg):
        from pickup_real import HttpCamera
        self.http = HttpCamera(int(cfg["camera"]["http_port"]))
        self.port = int(cfg["camera"]["http_port"])

    def look(self, tag, target_hint=None):
        return self.http.look(tag, target_hint=target_hint)

    def state(self):
        return self.http.state()

    def labels(self, n=8, timeout=6.0, want=None, tag="labels"):
        q = {"n": n, "timeout": timeout, "tag": tag}
        if want:
            q["want"] = want
        try:
            return self.http._get("/api/labels?" + urllib.parse.urlencode(q), timeout + 6)
        except OSError as exc:
            return {"ok": False, "reason": "camera server unreachable: %s" % exc}

    def status(self, text):
        self.http.status(text)

    def frame(self):
        """최신 실시간 원본 한 장(BGR). 상승 전후 비교용."""
        import cv2
        with urllib.request.urlopen("http://127.0.0.1:%d/frame.jpg?view=live" % self.port, timeout=4) as r:
            buf = np.frombuffer(r.read(), np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is None:
            raise Stop("camera live frame decode failed")
        return img


class RealArmViews:
    """기존 RealArm(보기 자세 B·수납) + 정면 자세(camera_views.json 의 front_view, 현장 값 그대로)."""

    def __init__(self, cfg, log):
        from pickup_real import RealArm, Stop as PickupStop
        self.PickupStop = PickupStop
        self.arm = RealArm(cfg, log)
        views = json.loads((ROOT / cfg["arm"]["views_file"]).read_text())
        name = cfg["lift_carry"]["front_view"]
        if name not in views:
            raise Stop("arm front view %s not in %s" % (name, cfg["arm"]["views_file"]))
        pose = {k: int(v) for k, v in views[name].items()}
        for sid, pwm in pose.items():
            self.arm.proto.check_pwm(sid, pwm)
        self.arm.poses["front"] = pose

    def _do(self, fn):
        try:
            fn()
        except self.PickupStop as exc:
            raise Stop(str(exc))

    def view(self):
        self._do(lambda: self.arm._go("view"))

    def front(self):
        self._do(lambda: self.arm._go("front"))

    def tilt(self, frac):
        """보기 자세 B(0) 와 정면 자세(1) 사이 관절값 직선 보간. 음수는 B 보다 더 아래(작은 범위만 허용)."""
        if not -0.35 <= float(frac) <= 1.0:
            raise Stop("arm tilt %.2f outside -0.35..1.0" % frac)
        v, f = self.arm.poses["view"], self.arm.poses["front"]
        pose = {k: int(round(v[k] + float(frac) * (f[k] - v[k]))) for k in v}
        for sid, pwm in pose.items():
            self.arm.proto.check_pwm(sid, pwm)
        # 기울기마다 이름을 따로 둔다. 하나의 이름("tilt")을 쓰면 RealArm._go 가 "이미 그 자세"로 보고
        # 연속 기울기를 건너뛴다(2026-11-03 강의실 1c: 0.35·0.2 에서 팔이 0.5 그대로).
        name = "tilt%+.2f" % float(frac)
        self.arm.poses[name] = pose
        self._do(lambda: self.arm._go(name))

    def close(self):
        self.arm.close()


class BoardReboot(Exception):
    """지게팔 보드가 상승 도중 재부팅(부팅 안내 줄)."""


def rebooted(line):
    """부팅 안내 줄인가. 리셋이 출력 도중이면 앞 줄 조각 뒤에 붙어 온다(예: "Echo: 1      Motor ready")."""
    from lift_carry_protocol import is_reboot
    return is_reboot(line)


class FrameRecorder:
    """기준 관측부터 끝날 때까지 카메라 원본 화면을 주기적으로 저장(정면 직진 화면 기록, 사용자 요청 10/31).
    frames/NNNNN.jpg + index.csv(시각·단계·그때까지 실제 odom 전진·이동 번호). 저장 실패는 판단에 쓰지 않는다."""

    def __init__(self, camera, out_dir, hz, status):
        import threading
        self.camera, self.dir, self.period, self.status = camera, Path(out_dir), 1.0 / max(0.5, float(hz)), status
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.count, self.errors, self.t0 = 0, 0, time.monotonic()

    def start(self):
        self.dir.mkdir(parents=True, exist_ok=True)
        self.thread.start()
        return self

    def _run(self):
        import cv2
        with (self.dir / "index.csv").open("w", encoding="utf-8") as idx:
            idx.write("n,t_s,stage,odom_forward_m,moves\n")
            while not self.stop_event.is_set():
                t = time.monotonic()
                try:
                    img = self.camera.frame()
                    cv2.imwrite(str(self.dir / ("%05d.jpg" % self.count)), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    stage, fwd, moves = self.status()
                    idx.write("%d,%.3f,%s,%.4f,%d\n" % (self.count, t - self.t0, stage, fwd, moves))
                    idx.flush()
                    self.count += 1
                except Exception:  # noqa: BLE001 — 기록 실패는 무시(판단과 무관)
                    self.errors += 1
                self.stop_event.wait(max(0.0, self.period - (time.monotonic() - t)))

    def stop(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=3)
        return {"frames": self.count, "errors": self.errors, "dir": str(self.dir)}


class LiftCarry:
    """지게팔 보드(backends.LiftUno) 위에서 줄마다 시각을 남겨 초음파 거리·끝 줄·재부팅을 판단한다."""

    def __init__(self, cfg, log):
        sys.path.insert(0, str(ROOT / "scripts"))
        from field_video.backends import BackendError, LiftUno
        from lift_carry_protocol import banner_line
        self.BackendError = BackendError
        self.log = log
        self.lines = []   # (monotonic, line)
        self.cfg=cfg
        self.nominal=float(cfg["lift_carry"]["nominal_pulse_s"])
        class CarryUno(LiftUno):
            def check_banner(uno,since,timeout=6):
                # 상위 공유 파일 대신 Carry 인스턴스에만 적용한다.
                import re
                end=time.monotonic()+timeout
                while time.monotonic()<end:
                    with uno.lock:
                        seen=[l for t,l in uno.lines if t>=since]
                    identity=any(banner_line(l,"u: Up (ultrasonic stop)") for l in seen)
                    stop=next((re.search(r"stop distance:\s*(\d+(?:\.\d+)?)\s*cm",l) for l in seen if re.search(r"stop distance:\s*(\d+(?:\.\d+)?)\s*cm",l)),None)
                    if identity and stop:
                        uno.counted = "Counted lift v1" in seen
                        uno.stop_cm=float(stop.group(1))
                        return uno.stop_cm
                    time.sleep(.05)
                raise BackendError("Carry firmware identity/stop distance missing")
        self.uno = CarryUno(cfg["lift"]["port"], output=lambda line: log("lift_out", line=line),
                           line_hook=lambda line: self.lines.append((time.monotonic(), line)))

    def open(self):
        from lift_carry_protocol import banner_line
        t0 = time.monotonic()
        try:
            self.uno.open()
        except self.BackendError as exc:
            raise Stop("lift open/banner: %s" % exc)
        seen=[l for t,l in self.lines if t>=t0]
        return [expected for expected in self.cfg["lift_carry"]["expected_banner"]
                if any(banner_line(line,expected) for line in seen)]

    def up_once(self, timeout):
        from lift_carry_protocol import parse_record
        mark = len(self.lines)
        sent = time.monotonic()
        if self.uno.moving:
            return {"sent":sent,"result":"error","error":"previous lift movement not resolved","lines":[]}
        self.uno.moving,self.uno.command_at=True,sent
        try:
            self.uno._send("u")
            while time.monotonic()-sent<timeout:
                now=time.monotonic()
                rec={"sent":sent,"now":now,"result":None,"error":None,"lines":list(self.lines[mark:])}
                evidence=parse_record(rec,float(self.cfg["lift_carry"]["stop_distance_cm"]),self.nominal)
                if evidence["reboot"] or evidence["serial_error"]:
                    rec["error"]="board rebooted during up" if evidence["reboot"] else "serial error"
                    return rec
                if evidence["end"] and (not self.uno.counted or any(
                    line.startswith("Up pulses: ") for _, line in rec["lines"]
                )):
                    time.sleep(.15)
                    rec.update(now=time.monotonic(),lines=list(self.lines[mark:]))
                    final=parse_record(rec,float(self.cfg["lift_carry"]["stop_distance_cm"]),self.nominal)
                    if final["reboot"] or final["serial_error"]:
                        rec["error"]="board rebooted/serial failed after completion"
                        return rec
                    self.uno.moving=False
                    rec["result"]=evidence["end"]
                    if evidence["end"]=="complete" and evidence.get("seconds"):
                        self.nominal=max(1e-5,(evidence["seconds"]-.1)/LIFT_PULSES)
                    self.log("lift_completion",source=evidence.get("end_source"),seconds=evidence.get("seconds"))
                    return rec
                time.sleep(.05)
            return {"sent":sent,"now":time.monotonic(),"result":"error","error":"lift timeout: completion evidence missing","lines":list(self.lines[mark:])}
        except self.BackendError as exc:
            return {"sent":sent,"result":"error","error":str(exc),"lines":list(self.lines[mark:])}

    def close(self):
        try:
            self.uno.wait_until_stopped(8)
        finally:
            self.uno.close()


# ===================================================================== lift evidence
def parse_up(rec, stop_cm, nominal_pulse_s=None):
    from lift_carry_protocol import parse_record
    return parse_record(rec,stop_cm,nominal_pulse_s)


def image_change(a, b):
    """두 BGR 영상 평균 절대 밝기 차(0~255, 160x120 축소)."""
    import cv2
    ga = cv2.resize(cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), (160, 120), interpolation=cv2.INTER_AREA).astype(np.float32)
    gb = cv2.resize(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY), (160, 120), interpolation=cv2.INTER_AREA).astype(np.float32)
    return float(np.mean(np.abs(ga - gb)))


def lower_plan(pulses, tolerance_pulses=CHECK_EVERY):
    """저장된 운반 높이 → 진입 높이로 내리는 계획. 현재 펌웨어 `d` 는 12800 펄스 단위뿐이다."""
    full, rest = divmod(int(pulses), LIFT_PULSES)
    if rest <= tolerance_pulses:
        return {"d_commands": full, "residual_pulses": rest, "ok": True}
    if LIFT_PULSES - rest <= tolerance_pulses:
        return {"d_commands": full + 1, "residual_pulses": rest - LIFT_PULSES, "ok": True}
    return {"d_commands": full, "residual_pulses": rest, "ok": False,
            "why": "firmware d moves 12800 pulses only; %d-pulse remainder needs a step-count down command" % rest}


# ===================================================================== runner
class StraightPickup:
    def __init__(self, cfg, args, est, teach, camera, arm, base, lift, run_dir):
        self.cfg, self.args, self.est, self.teach = cfg, args, est, teach
        self.cam, self.arm, self.base, self.lift = camera, arm, base, lift
        self.s = cfg["straight"]
        self.lc = cfg["lift_carry"]
        self.mount = {k: float(teach["mount"][k]) for k in ("forward", "left", "yaw")}
        self.fork_tip = float(cfg["robot"]["fork_tip_forward_m"])
        self.fork_left = float(cfg["robot"].get("fork_center_left_m", 0.0))
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.events = (self.run_dir / "events.jsonl").open("a", encoding="utf-8")
        self.t0 = time.monotonic()
        self.summary = {"tool": "straight_pickup", "target": self.target_desc(), "ok": False, "stage": None,
                        "moves": [], "looks": []}
        self.odom = {"forward": 0.0, "lateral": 0.0, "yaw": 0.0}   # 기준 자세부터 실제 odom 누계
        self.state_dir = LOG_ROOT
        self.recorder = None
        self.bearing = None      # 숫자 모드: 정면 자세에서 읽은 송장 숫자 방향(도, 오른쪽 +)
        self.fixed_roll = None   # 저장된 정면 기준 roll(rad). None 이면 지금 배치를 정면으로 보고 역산

    # ------------------------------------------------------------------ utils
    def target_desc(self):
        return ("label:%s" % self.args.target_label) if self.args.target_label else "parcel"

    def log(self, kind, **fields):
        rec = {"t": round(time.monotonic() - self.t0, 3), "kind": kind}
        rec.update(fields)
        self.events.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
        self.events.flush()
        if kind not in ("base_out", "lift_out"):
            print("[%6.1f] %-12s %s" % (rec["t"], kind, json.dumps(fields, ensure_ascii=False, default=str)[:260]),
                  flush=True)

    def stage(self, name):
        self.summary["stage"] = name
        self.log("stage", name=name)
        try:
            self.cam.status("straight: " + name)
        except Exception:  # noqa: BLE001 — 화면 문구 실패는 무시
            pass

    def write_state(self, stage, **extra):
        if getattr(self.args, "fake", False):
            self.summary.setdefault("states", []).append(stage)
            return
        data = {"stage": stage, "at": time.time(), "run_dir": str(self.run_dir), "tool": "straight_pickup"}
        data.update(extra)
        STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = STATE_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, default=str))
        tmp.replace(STATE_FILE)

    def hint(self, expected=None):
        h = {}
        if expected is not None:
            h["expected_robot"] = [float(v) for v in expected]
        elif self.bearing is not None:
            # 숫자 모드: 정면 자세에서 숫자가 보인 방향(또는 정렬 뒤 예상 방향)의 팔레트
            h["bearing_deg"], h["bearing_tol_deg"] = float(self.bearing), float(self.s.get("bearing_tol_deg", 8.0))
            return h
        elif not self.args.target_label:
            h["parcel"] = True
        if self.args.target_label:
            h["label"] = str(self.args.target_label)
        return h

    def look(self, tag, expected=None):
        r = self.cam.look(tag, target_hint=self.hint(expected))
        if not r.get("ok") and str(r.get("reason", "")).startswith("unstable"):
            self.log("look_retry", tag=tag, reason=r.get("reason"))
            time.sleep(0.5)
            r = self.cam.look(tag + "_retry", target_hint=self.hint(expected))
        keep = {k: r.get(k) for k in ("ok", "reason", "tape_centres_px", "cam", "spread", "good", "frames_seen",
                                      "image", "target_evidence", "target_summary", "pitch_down_deg")}
        self.summary["looks"].append(dict(keep, tag=tag))
        self.log("look", tag=tag, **keep)
        if not r.get("ok") or not r.get("tape_centres_px"):
            raise Stop("vision (%s): %s" % (tag, r.get("reason") or "no marker centres"))
        return r

    def server_pose(self, r):
        """카메라 서버 모델(설정 roll)의 로봇 자세 — 서버의 목표 연속성 비교용(같은 모델끼리만 비교)."""
        k = float(self.teach.get("position_scale", 1.0))
        cam = dict(r["cam"], x=r["cam"]["x"] * k, y=r["cam"]["y"] * k)
        return robot_from_camera(cam, self.mount)

    def measure(self, r, roll):
        m, why = markers_in_robot(self.est, r["tape_centres_px"], roll, self.mount)
        if m is None:
            raise Stop("marker geometry: %s" % why)
        return m

    def move(self, distance, why):
        s = self.s
        self.summary["moves"].append(["straight", round(distance, 4), why])
        self.log("move", what="straight", commanded_m=round(distance, 4), why=why)
        speed = float(s["speed_m_s"]) if distance > 0 else float(s["retreat_speed_m_s"])
        data = self.base.straight(distance, speed, float(s["move_max_yaw_deg"]), float(s["move_max_lateral_m"]))
        self.log("moved", **{k: data.get(k) for k in ("ok", "reason", "forward_m", "lateral_m", "yaw_change_deg",
                                                       "peak_lateral_m", "peak_yaw_deg", "duration_s")})
        f = data.get("forward_m")
        if f is not None:
            # 누계는 명령값이 아니라 실제 odom(시작 heading 기준 전진·좌우·yaw)
            yaw0 = self.odom["yaw"]
            self.odom["forward"] += f * math.cos(yaw0) - float(data.get("lateral_m") or 0) * math.sin(yaw0)
            self.odom["lateral"] += f * math.sin(yaw0) + float(data.get("lateral_m") or 0) * math.cos(yaw0)
            self.odom["yaw"] += math.radians(float(data.get("yaw_change_deg") or 0.0))
        self.log("odom_total", forward_m=round(self.odom["forward"], 4), lateral_m=round(self.odom["lateral"], 4),
                 yaw_deg=round(math.degrees(self.odom["yaw"]), 3))
        if not data.get("ok"):
            raise Stop("straight move failed: %s" % data.get("reason"))
        if abs(math.degrees(self.odom["yaw"])) > float(s["total_max_yaw_deg"]):
            raise Stop("odom yaw since reference %.2f deg > %.2f" % (math.degrees(self.odom["yaw"]), float(s["total_max_yaw_deg"])))
        if abs(self.odom["lateral"]) > float(s["total_max_lateral_m"]):
            raise Stop("odom lateral drift since reference %.3f m > %.3f" % (self.odom["lateral"], float(s["total_max_lateral_m"])))
        return data

    # ------------------------------------------------------------------ stages
    def preflight(self):
        self.stage("PREFLIGHT")
        s = self.s
        old = None
        if not getattr(self.args, "fake", False):
            try:
                old = json.loads(STATE_FILE.read_text())
            except (OSError, ValueError):
                old = None
        self.write_state("invalidated_manual_reposition", prior=old,
                         note="operator moved robot and pallet by hand; previous inserted state/targets are not used")
        if hasattr(self.cam, "http"):
            # 2026-11-03 강의실 1회차 P1: 카메라 서버는 요청이 없으면 2 Hz로 쉰다(camera_server IDLE_HZ).
            # 쉬는 속도로 재면 항상 5 fps 미만이라, look 한 번으로 깨운 뒤 실제 인식 속도·나이를 잰다. 기준값은 그대로.
            woke = time.monotonic()
            self.cam.http.look("preflight_wake", n=3, timeout=3.0)
            time.sleep(max(0.6, 1.2 - (time.monotonic() - woke)))
        st = self.cam.state()
        self.log("camera_state", **{k: st.get(k) for k in ("fps", "age_s", "camera_error", "capture_fps",
                                                         "capture_age_s", "capture_error", "mode", "teach_error")})
        if st.get("error") or st.get("camera_error") or st.get("capture_error"):
            raise Stop("camera: %s" % (st.get("error") or st.get("camera_error") or st.get("capture_error")))
        if float(st.get("fps") or 0) < float(s["min_camera_fps"]):
            raise Stop("camera recognition fps %.1f < %.1f" % (float(st.get("fps") or 0), float(s["min_camera_fps"])))
        if st.get("age_s") is None or float(st["age_s"]) > float(self.cfg["camera"]["max_age_s"]):
            raise Stop("camera frame age %s s" % st.get("age_s"))
        p = self.base.probe()
        self.log("sensor_probe", **p)
        self.summary["preflight"] = p
        if not p.get("ok"):
            raise Stop("sensor probe failed: %s" % p.get("reason"))
        if float(p.get("scan_hz") or 0) < float(s["min_scan_hz"]) or (p.get("scan_age_s") is None or p["scan_age_s"] > 0.5):
            raise Stop("lidar scan not fresh (%.2f Hz, age %s s)" % (float(p.get("scan_hz") or 0), p.get("scan_age_s")))
        if float(p.get("odom_hz") or 0) < float(s["min_odom_hz"]) or (p.get("odom_age_s") is None or p["odom_age_s"] > 0.3):
            raise Stop("odom not fresh (%.2f Hz, age %s s)" % (float(p.get("odom_hz") or 0), p.get("odom_age_s")))
        if p.get("drive_ready") is not True:
            raise Stop("OpenCR drive not ready (/drive/ready=%s)" % p.get("drive_ready"))
        others = other_cmd_vel_publishers(p.get("cmd_vel_publishers") or [], self.cfg)
        if others:
            raise Stop("/cmd_vel already has publishers %s — teleop/web off" % others)
        if p.get("cmd_vel_msgs"):
            raise Stop("/cmd_vel received %d message(s) during the probe — Nav2 goal or teleop active" % p["cmd_vel_msgs"])
        self.check_rear(p, float(self.cfg["retreat"]["distance_m"]))
        if not self.args.no_lift:
            port = self.cfg["lift"]["port"]
            if not getattr(self.args, "fake", False) and not Path(port).exists():
                raise Stop("lift port missing: %s" % port)
            try:
                ref = json.loads(HEIGHT_FILE.read_text()) if not getattr(self.args, "fake", False) else {}
            except (OSError, ValueError):
                ref = {}
            self.log("lift_height_reference", name=ref.get("name"), absolute_height_m=ref.get("absolute_height_m"),
                     idle_policy=ref.get("idle_policy"))

    def stored_roll(self):
        """지난 사람 정렬 기준(straight_reference_latest.json)의 roll. 정렬 모드는 이 값이 있어야 한다."""
        if getattr(self.args, "roll_deg", None) is not None:
            return math.radians(float(self.args.roll_deg))
        try:
            ref = json.loads(REFERENCE_LATEST.read_text())
            roll = float(ref["camera_roll_deg_implied"])
        except (OSError, ValueError, KeyError) as exc:
            raise Stop("stored front reference roll unavailable (%s) — 먼저 사람이 맞춘 배치에서 straight_pickup 실행" % exc)
        if abs(roll) > float(self.s["max_reference_roll_deg"]):
            raise Stop("stored roll %.1f deg out of range" % roll)
        self.log("stored_roll", roll_deg=roll, source=str(REFERENCE_LATEST))
        return math.radians(roll)

    def check_rear(self, probe, retreat):
        rear = probe.get("rear") or {}
        need = retreat + float(self.s["rear_margin_m"])
        nearest = rear.get("nearest_m")
        if nearest is None:
            if rear.get("reason") != "no rear points (open or out of range)":
                raise Stop("rear clearance not measured: %s" % rear.get("reason"))
            self.log("rear_clear", required_m=round(need, 3), nearest_m=None, note="no lidar points behind within range")
            return
        if nearest < need:
            raise Stop("rear clearance %.2f m < required %.2f m for retreat" % (nearest, need))
        self.log("rear_clear", required_m=round(need, 3), nearest_m=nearest)

    def reference(self):
        self.stage("REFERENCE")
        s = self.s
        self.arm.view()
        if self.recorder is None and float(s.get("record_hz", 0) or 0) > 0:
            self.recorder = FrameRecorder(self.cam, self.run_dir / "frames", float(s["record_hz"]),
                                          lambda: (self.summary.get("stage"), self.odom["forward"],
                                                   len(self.summary["moves"]))).start()
        r = self.look("straight_reference")
        m0 = self.measure(r, 0.0)
        if self.fixed_roll is not None:
            roll = self.fixed_roll
            m = self.measure(r, roll)
            if getattr(self.args, "align", False) and not self.args.plan_only:
                r, m = self.align(r, m, roll)
            if abs(math.degrees(m["face_yaw"])) > float(s["max_fixed_roll_face_yaw_deg"]):
                raise Stop("reference: pallet face %.2f deg off square with stored roll (> %.1f) — 정렬 필요"
                           % (math.degrees(m["face_yaw"]), float(s["max_fixed_roll_face_yaw_deg"])))
        else:
            roll, why = roll_for_square_face(self.est, r["tape_centres_px"], self.mount)
            if roll is None:
                raise Stop("reference: %s — 배치가 정면이 아니거나 카메라 자세 이상" % why)
            if abs(math.degrees(roll)) > float(s["max_reference_roll_deg"]):
                raise Stop("reference: implied camera roll %.1f deg > %.1f — 배치/카메라 확인"
                           % (math.degrees(roll), float(s["max_reference_roll_deg"])))
            m = self.measure(r, roll)
        lateral = m["lateral"] - self.fork_left
        tip_to_face = m["forward"] - self.fork_tip
        total = tip_to_face + float(s["insert_depth_m"])
        ref = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "target": self.target_desc(),
               "tape_centres_px": r["tape_centres_px"], "image": r.get("image"),
               "target_evidence": r.get("target_evidence"), "target_summary": r.get("target_summary"),
               "camera_roll_deg_implied": round(math.degrees(roll), 3), "pitch_deg": round(math.degrees(m["pitch"]), 3),
               "roll_source": "stored reference" if self.fixed_roll is not None else "this placement taken as square",
               "face_yaw_deg": round(math.degrees(m["face_yaw"]), 3), "label_bearing_deg": self.bearing,
               "markers_robot_m": {"left": [round(v, 4) for v in m["L"]], "right": [round(v, 4) for v in m["R"]]},
               "pallet_centre_robot_m": [round(m["forward"], 4), round(m["lateral"], 4)],
               "fork_lateral_offset_m": round(lateral, 4), "fork_tip_to_face_m": round(tip_to_face, 4),
               "planned_insert_travel_m": round(total, 4), "insert_depth_m": float(s["insert_depth_m"]),
               "with_config_roll0": {"face_yaw_deg": round(math.degrees(m0["face_yaw"]), 3),
                                     "lateral_m": round(m0["lateral"], 4), "forward_m": round(m0["forward"], 4)},
               "server_model_pose": [round(v, 4) for v in self.server_pose(r)],
               "mount": self.mount, "fork_tip_forward_m": self.fork_tip, "focal_scale": self.teach.get("focal_scale"),
               "note": "사람이 직진 삽입 가능하게 맞춘 배치를 앞면 기울기 0 으로 정의. roll 은 그 정의에서 역산한 값"
                       "(teach_tape.json 미변경). 카메라는 포크 중심 오른쪽 약 11 cm — 화면 중앙 정렬을 하지 않는다"}
        (self.run_dir / "straight_reference.json").write_text(json.dumps(ref, indent=1, ensure_ascii=False))
        if not getattr(self.args, "fake", False) and self.fixed_roll is None:
            REFERENCE_LATEST.write_text(json.dumps(dict(ref, run_dir=str(self.run_dir)), indent=1, ensure_ascii=False))
        self.summary["reference"] = ref
        self.log("reference", roll_deg=ref["camera_roll_deg_implied"], fork_lateral_cm=round(lateral * 100, 2),
                 tip_to_face_m=ref["fork_tip_to_face_m"], insert_travel_m=ref["planned_insert_travel_m"],
                 roll0_face_yaw_deg=ref["with_config_roll0"]["face_yaw_deg"])
        if abs(lateral) > float(s["max_reference_lateral_m"]):
            raise Stop("reference: pallet centre %.1f cm from the fork centre line (> %.1f cm) — 수동 재정렬 필요"
                       % (lateral * 100, float(s["max_reference_lateral_m"]) * 100))
        if not float(s["min_total_m"]) <= total <= float(s["max_total_m"]):
            raise Stop("reference: insert travel %.3f m outside %.2f..%.2f m" % (total, float(s["min_total_m"]), float(s["max_total_m"])))
        self.write_state("straight_reference", reference=str(self.run_dir / "straight_reference.json"))
        return r, m, roll

    def scan_label(self):
        """숫자 모드: 팔을 정면 자세에서 보기 자세 B 쪽으로 차례로 기울이며 송장 숫자를 읽고, 읽힌 자세에서 그 숫자의
        방향(오른쪽 +)을 잰다. 정면 자세만으로는 팔레트 위 상자 라벨이 화면 아래로 빠질 수 있다(192호 2026-11-01)."""
        self.stage("FIND_LABEL")
        want = str(self.args.target_label)
        need = int(self.s.get("label_min_frames", 3))
        tried, res, hits = [], {}, []
        for frac in [float(f) for f in self.s.get("label_tilts", [1.0])]:
            if frac >= 1.0:
                self.arm.front()
            else:
                self.arm.tilt(frac)
            time.sleep(float(self.lc.get("settle_s", 1.0)))
            res = self.cam.labels(n=int(self.s.get("label_frames", 8)), timeout=float(self.s.get("label_timeout_s", 8.0)),
                                  want=want, tag="find_label_%s_%+.2f" % (want, frac))
            self.log("labels", tilt=frac, **{k: res.get(k) for k in ("ok", "reason", "frames_seen", "labels", "unread")})
            if not res.get("ok"):
                tried.append("%+.2f: %s" % (frac, res.get("reason")))
                continue
            hits = [lb for lb in res.get("labels", []) if lb["text"] == want]
            seen = ", ".join("%s(%d)" % (lb["text"], lb["frames"]) for lb in res.get("labels", [])) or "none"
            tried.append("%+.2f: %s" % (frac, seen))
            if hits and hits[0]["frames"] >= need:
                break
            hits = []
        self.summary["label_scan"] = res
        if not hits:
            raise Stop("label %s not read in %d+ frames at tilts (%s)" % (want, need, "; ".join(tried)))
        others = [lb for lb in res.get("labels", []) if lb["text"] != want and lb["frames"] >= need]
        K = self.est.K
        # 기울인 자세의 아래 보기 각은 B(약 35°)와 정면(약 0°) 사이 직선 보간으로 근사(방향 오차 1° 미만)
        pitch = math.radians(float(self.s.get("view_pitch_deg", 35.0))) * (1.0 - min(1.0, frac))
        self.bearing = round(float(pixel_bearing_deg(K, hits[0]["u"], hits[0].get("v"), pitch)), 2)
        self.log("label_found", text=want, tilt=frac, frames=hits[0]["frames"], u=round(hits[0]["u"], 1),
                 bearing_deg=self.bearing, other_labels=[lb["text"] for lb in others])
        return self.bearing

    def search_parcel(self):
        """택배 모드: 보기 자세 B 에서 택배가 올라간 팔레트가 안 보이면 카메라를 위아래로 움직여 직접 찾는다.
        찾은 자세에서 그 팔레트(표식 쌍 가운데)의 방향을 기억하고 보기 자세 B 로 돌아온다. 방향은 표식 두 개로
        pitch 를 풀어 계산하므로 팔을 기울여도 유지된다(카메라 높이 오차는 앞·옆 거리를 같은 비율로 바꿔 방향은 그대로)."""
        self.stage("SEARCH")
        tried = []
        for frac in [float(f) for f in self.s.get("search_tilts", [0.0])]:
            if frac == 0.0:
                self.arm.view()
            else:
                self.arm.tilt(frac)
            tag = "search_%+.2f" % frac
            r = self.cam.look(tag, target_hint={"parcel": True})
            keep = {k: r.get(k) for k in ("ok", "reason", "tape_centres_px", "cam", "image", "target_evidence",
                                          "target_summary")}
            self.summary["looks"].append(dict(keep, tag=tag))
            self.log("look", tag=tag, tilt=frac, **keep)
            tried.append("%+.2f: %s" % (frac, "ok" if r.get("ok") else str(r.get("reason"))[:60]))
            if not (r.get("ok") and r.get("tape_centres_px") and r.get("cam")):
                continue
            if frac == 0.0:
                self.log("parcel_found", tilt=0.0, bearing_deg=None, note="visible in view B")
                return None
            t = r["tape_centres_px"]
            u, v = 0.5 * (t["left"][0] + t["right"][0]), 0.5 * (t["left"][1] + t["right"][1])
            self.bearing = round(float(pixel_bearing_deg(self.est.K, u, v, float(r["cam"]["pitch_down"]))), 2)
            self.summary["parcel_search"] = {"tilt": frac, "bearing_deg": self.bearing}
            self.log("parcel_found", tilt=frac, bearing_deg=self.bearing)
            self.arm.view()
            return self.bearing
        self.arm.view()
        raise Stop("search: parcel pallet not found (" + "; ".join(tried) + ")")

    def align(self, r, m, roll):
        """팔레트 정면 축으로 정렬(숫자 모드에서 고른 팔레트가 정면이 아닐 때). 앞면에서 1 m 이상 떨어진 곳에서만,
        회전 + 직진. 삽입 구간(APPROACH)은 그대로 직진 전용. 끝나면 좌우·기울기 허용치 안이어야 한다."""
        self.stage("ALIGN")
        s = self.s
        tol = {"lateral": float(s["align_lateral_m"]), "yaw": math.radians(float(s["align_yaw_deg"])),
               "distance": float(s["align_distance_tol_m"])}
        target_x = -float(s["align_robot_to_face_m"])
        self.base.prepare()
        for it in range(int(s["align_max_iterations"]) + 1):
            psi = m["face_yaw"]
            c, sn = math.cos(-psi), math.sin(-psi)
            cx, cy = m["forward"], m["lateral"] - self.fork_left
            pose = (-(c * cx - sn * cy), -(sn * cx + c * cy), -psi)    # 로봇 자세(팔레트 좌표, 포크 중심 기준)
            moves, note = plan_moves(pose, (target_x, 0.0, 0.0), tol, math.radians(float(s["align_max_path_deg"])),
                                     float(s["align_max_back_m"]))
            self.log("align_plan", iteration=it, pose=[round(v, 4) for v in pose], moves=moves, note=note)
            if moves is None:
                raise Stop("align: " + note)
            if not moves:
                self.summary["aligned"] = {"iterations": it, "lateral_cm": round(pose[1] * 100, 2),
                                           "yaw_deg": round(math.degrees(pose[2]), 2)}
                return r, m
            if it == int(s["align_max_iterations"]):
                raise Stop("align: not within %.1f cm / %.1f deg after %d iterations"
                           % (tol["lateral"] * 100, math.degrees(tol["yaw"]), it))
            if -pose[0] - self.fork_tip < float(s["align_min_tip_to_face_m"]):
                raise Stop("align: forks too close to the pallet to turn (%.2f m)" % (-pose[0] - self.fork_tip))
            actual = []
            for kind, value in moves:
                self.summary["moves"].append([kind, round(math.degrees(value), 2) if kind == "turn" else round(value, 4), "align"])
                if kind == "turn":
                    if abs(math.degrees(value)) > float(s["align_max_turn_deg"]):
                        raise Stop("align: turn %.1f deg > %.1f" % (math.degrees(value), float(s["align_max_turn_deg"])))
                    self.log("move", what="turn", commanded_deg=round(math.degrees(value), 2), why="align")
                    data = self.base.turn(value) or {}
                    got = math.radians(float(data["turned_deg"])) if data.get("turned_deg") is not None else value
                    self.log("turned", turned_deg=round(math.degrees(got), 2))
                    actual.append(("turn", got))
                else:
                    self.log("move", what="straight", commanded_m=round(value, 4), why="align")
                    data = self.base.straight(value, float(s["align_speed_m_s"]), float(s["align_move_max_yaw_deg"]),
                                              float(s["move_max_lateral_m"]))
                    if not data.get("ok"):
                        raise Stop("align straight failed: %s" % data.get("reason"))
                    actual.append(("straight", float(data.get("forward_m", value))))
            # 이동 뒤 팔레트 중심이 보일 방향(카메라 기준)을 예측해 같은 팔레트만 다시 고른다
            nx, ny, nth = simulate_moves(pose, actual)
            c2, s2 = math.cos(nth), math.sin(nth)
            fwd = c2 * (-nx) + s2 * (-ny)                  # 팔레트 중심의 로봇 기준 앞
            lat = -s2 * (-nx) + c2 * (-ny) + self.fork_left
            self.bearing = round(math.degrees(math.atan2(-(lat - self.mount["left"]), fwd - self.mount["forward"])), 2)
            self.arm.view()
            r = self.look("align_%d" % it)
            m = self.measure(r, roll)
        raise Stop("align: loop ended")

    def plan(self, m, roll):
        """보이는 동안 다시 볼 구간들과 마지막 블라인드 구간(계획, 실제 이동 때 다시 계산)."""
        s = self.s
        total = m["forward"] - self.fork_tip + float(s["insert_depth_m"])
        looks, travel = [], 0.0
        while True:
            seg = min(float(s["segment_m"]), total - travel)
            if seg <= 0.02:
                break
            ok, uv = visible_after(self.est, m, travel + seg, roll, self.mount,
                                   margin_side=int(s["view_margin_px"]), margin_bottom=int(s["view_margin_px"]))
            if not ok or total - travel - seg < 0.02:
                break
            travel += seg
            looks.append({"at_m": round(travel, 4), "uv": [[round(a, 1), round(b, 1)] for a, b in uv]})
        return {"total_m": round(total, 4), "looks": looks, "blind_m": round(total - travel, 4)}

    def approach(self, r_ref, m_ref, roll):
        self.stage("APPROACH")
        s = self.s
        plan = self.plan(m_ref, roll)
        self.summary["plan"] = plan
        self.log("plan", **plan)
        if plan["blind_m"] > float(s["max_blind_m"]):
            raise Stop("blind final %.3f m > %.2f m (markers leave view too early)" % (plan["blind_m"], float(s["max_blind_m"])))
        if self.args.plan_only:
            return None
        self.base.prepare()
        m_last, r_last, scale = m_ref, r_ref, 1.0
        while True:
            total_true = m_last["forward"] - self.fork_tip + float(s["insert_depth_m"])
            seg = min(float(s["segment_m"]), total_true)
            ok, _ = visible_after(self.est, m_last, seg, roll, self.mount, margin_side=int(s["view_margin_px"]),
                                  margin_bottom=int(s["view_margin_px"]))
            if not ok or total_true - seg < 0.02:
                break
            before = dict(self.odom)
            pose_server = self.server_pose(r_last)
            self.move(seg, "vision segment")
            df = self.odom["forward"] - before["forward"]
            dl = self.odom["lateral"] - before["lateral"]
            dy = self.odom["yaw"] - before["yaw"]
            h = pose_server[2]
            expected = (pose_server[0] + df * math.cos(h) - dl * math.sin(h),
                        pose_server[1] + df * math.sin(h) + dl * math.cos(h), wrap(h + dy))
            r = self.look("straight_%03dcm" % round(self.odom["forward"] * 100), expected=expected)
            m = self.measure(r, roll)
            scale = self.check_consistency(m_ref, m)
            m_last, r_last = m, r
        # 마지막 구간: 카메라로 다시 못 본다. 마지막 영상 거리 → 실제 odom 명령(영상/odom 비로 보정)
        total_true = m_last["forward"] - self.fork_tip + float(s["insert_depth_m"])
        lo, hi = s["odom_vision_scale"]
        if not float(lo) <= scale <= float(hi):
            raise Stop("vision/odom distance ratio %.3f outside %.2f..%.2f" % (scale, float(lo), float(hi)))
        blind_cmd = total_true / scale
        if blind_cmd > float(s["max_blind_m"]):
            raise Stop("blind final %.3f m > %.2f m" % (blind_cmd, float(s["max_blind_m"])))
        self.log("final_segment", by_vision_m=round(total_true, 4), vision_over_odom=round(scale, 4),
                 odom_command_m=round(blind_cmd, 4), last_view_tip_to_face_m=round(m_last["forward"] - self.fork_tip, 4))
        before = self.odom["forward"]
        self.move(blind_cmd, "blind final (odom only)")
        got = self.odom["forward"] - before
        if abs(got - blind_cmd) > float(s["final_tolerance_m"]):
            raise Stop("final segment moved %.3f m by odom, commanded %.3f m" % (got, blind_cmd))
        depth_est = float(s["insert_depth_m"]) + (got * scale - total_true)
        ins = {"odom_forward_total_m": round(self.odom["forward"], 4), "odom_lateral_total_m": round(self.odom["lateral"], 4),
               "odom_yaw_total_deg": round(math.degrees(self.odom["yaw"]), 3),
               "vision_over_odom": round(scale, 4), "insert_depth_estimate_m": round(depth_est, 4),
               "last_view_tip_to_face_m": round(m_last["forward"] - self.fork_tip, 4)}
        self.summary["inserted"] = ins
        self.log("inserted", **ins)
        self.write_state("inserted_straight", **ins)
        return ins

    def check_consistency(self, m_ref, m):
        """기준 대비: 팔레트 중심 좌우(로봇 좌우 odom 이탈 반영)·앞면 기울기(odom yaw 반영)·거리(실제 odom 전진)."""
        s = self.s
        o = self.odom
        expect_lat = m_ref["lateral"] - o["lateral"]
        d_lat = m["lateral"] - expect_lat
        d_yaw = math.degrees(m["face_yaw"] + o["yaw"])
        seen = m_ref["forward"] - m["forward"]
        d_dist = seen - o["forward"]
        scale = seen / o["forward"] if o["forward"] > 0.05 else 1.0
        chk = {"lateral_change_cm": round(d_lat * 100, 2), "face_yaw_deg": round(d_yaw, 2),
               "vision_travel_m": round(seen, 4), "odom_travel_m": round(o["forward"], 4),
               "distance_disagree_m": round(d_dist, 4), "vision_over_odom": round(scale, 4),
               "pitch_deg": round(math.degrees(m["pitch"]), 2)}
        self.log("straight_check", **chk)
        self.summary.setdefault("checks", []).append(chk)
        if abs(d_lat) > float(s["lateral_change_m"]):
            raise Stop("pallet centre moved %.1f cm sideways vs reference (> %.1f cm)" % (d_lat * 100, float(s["lateral_change_m"]) * 100))
        if abs(m["lateral"] - self.fork_left) > float(s["max_reference_lateral_m"]):
            raise Stop("pallet centre %.1f cm from fork centre line" % ((m["lateral"] - self.fork_left) * 100))
        if abs(d_yaw) > float(s["face_yaw_deg"]):
            raise Stop("pallet face angle changed %.2f deg vs reference (> %.1f)" % (d_yaw, float(s["face_yaw_deg"])))
        if abs(d_dist) > float(s["distance_disagree_m"]) + 0.04 * o["forward"]:
            raise Stop("vision travel %.3f m vs odom %.3f m disagree (slip, blocked or pallet moved)" % (seen, o["forward"]))
        return scale

    def lift_and_carry(self):
        self.stage("FRONT_VIEW")
        lc = self.lc
        self.arm.front()
        time.sleep(float(lc.get("settle_s", 1.0)))
        before = self.cam.frame()
        self.save_frame(before, "lift_before")
        self.stage("LIFT")
        banner = self.lift.open()
        self.log("lift_banner", lines=banner)
        need = set(lc["expected_banner"])
        missing = sorted(need - set(banner))
        if missing:
            raise Stop("lift firmware banner differs from the shared candidate (missing %s) — 설치판 확인 전 상승 안 함" % missing)
        stop_cm = float(lc["stop_distance_cm"])
        nominal = float(lc["nominal_pulse_s"])
        records, total_pulses, total_seconds = [], 0, 0.0
        pre_first = None
        lifted = False
        for k in range(1, int(lc["max_up_commands"]) + 1):
            rec = parse_up(self.lift.up_once(float(lc["up_timeout_s"])), stop_cm, nominal)
            if getattr(getattr(self.lift, "uno", None), "counted", False) and rec.get("pulse_source") != "board count":
                raise Stop("lift up %d: board pulse count missing or inconsistent" % k)
            rec["k"] = k
            records.append(rec)
            self.log("lift_up", **{kk: v for kk, v in rec.items() if kk != "echo_cm"},
                     echo_n=len(rec["echo_cm"]), echo_first=rec["echo_cm"][:2], echo_last=rec["echo_cm"][-2:])
            if rec["reboot"]:
                raise Stop("lift board rebooted during up %d (brown-out or USB) — 추가 명령 중단" % k)
            if rec["serial_error"] or rec["error"]:
                raise Stop("lift up %d: %s" % (k, rec["error"] or "serial error"))
            if rec["end"] is None:
                raise Stop("lift up %d: no end line" % k)
            if rec["end"] == "complete" and rec.get("seconds"):
                nominal = max(1e-5, (rec["seconds"] - 0.10) / LIFT_PULSES)   # 실제 완료 시간으로 펄스 시간 갱신
            if k == 1:
                pre_first = rec.get("pre_cm")
                if pre_first is not None and pre_first < stop_cm:
                    raise Stop("ultrasonic already %.1f cm before lifting — 센서 앞이 막혀 상승 판단 불가" % pre_first)
            total_pulses += int(rec.get("pulses") or 0)
            total_seconds += float(rec.get("seconds") or 0.0)
            if rec["end"] == "height_limit":
                if total_pulses <= 0:
                    raise Stop("height limit before any movement — 상승 근거 없음")
                lifted = rec.get("stop_cm") is not None and rec["stop_cm"] < stop_cm
                break
        after = self.cam.frame()
        self.save_frame(after, "lift_after")
        change = image_change(before, after)
        evidence = {"ultrasonic_first_cm": pre_first, "ultrasonic_stop_cm": records[-1].get("stop_cm"),
                    "up_commands": len(records), "pulses_above_entry": total_pulses,
                    "seconds_moving": round(total_seconds, 3), "pulse_s": nominal,
                    "pulse_sources": [r.get("pulse_source") for r in records],
                    "camera_change": round(change, 2), "camera_change_min": float(lc["min_image_change"])}
        self.summary["lift"] = {"evidence": evidence, "records": records}
        self.log("lift_evidence", **evidence)
        if not lifted:
            raise Stop("box did not reach the ultrasonic after %d up commands — 상승 미확인, 후진 안 함" % len(records))
        if change < float(lc["min_image_change"]):
            self.log("lift_warning", note="front camera image barely changed; ultrasonic says the box rose into its beam")
        self.save_height(evidence)
        self.write_state("lifted_verified", **evidence)
        return evidence

    def save_frame(self, img, name):
        try:
            import cv2
            cv2.imwrite(str(self.run_dir / (name + ".jpg")), img)
        except Exception as exc:  # noqa: BLE001 — 기록 실패는 판단에 쓰지 않는다
            self.log("save_error", name=name, error=str(exc))

    def save_height(self, ev):
        """진입 높이 기준 파일에 운반 높이(펄스·시간)를 덧붙인다. 다음 집기 전에 이만큼 내린다."""
        rec = {"recorded_at": time.time(), "from": "pallet_entry_height", "pulses": ev["pulses_above_entry"],
               "seconds_moving": ev["seconds_moving"], "pulse_s": ev["pulse_s"], "up_commands": ev["up_commands"],
               "stop": "ultrasonic %.1f cm" % (ev["ultrasonic_stop_cm"] or -1),
               "lower_plan": lower_plan(ev["pulses_above_entry"]),  # legacy fixed-step d compatibility
               "run_dir": str(self.run_dir),
               "note": "초음파 정지까지의 펄스 합. 기존 보드의 마지막 상승은 추정이며 높이 cm는 미측정."}
        self.summary["carry_height"] = rec
        if getattr(self.args, "fake", False):
            return
        try:
            data = json.loads(HEIGHT_FILE.read_text())
        except (OSError, ValueError):
            data = {"name": "pallet_entry_height", "absolute_height_m": None}
        data["carry_height_above_entry"] = rec
        data.setdefault("history", []).append(rec)
        tmp = HEIGHT_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=1, ensure_ascii=False))
        tmp.replace(HEIGHT_FILE)

    def retreat(self):
        self.stage("RETREAT")
        dist = float(self.cfg["retreat"]["distance_m"])
        p = self.base.probe()
        self.log("sensor_probe", **{k: p.get(k) for k in ("scan_hz", "odom_hz", "drive_ready", "gate_ready", "rear")})
        self.check_rear(p, dist)
        before = self.odom["forward"]
        self.move(-dist, "retreat with pallet")
        got = before - self.odom["forward"]
        self.summary["retreat"] = {"odom_back_m": round(got, 4)}
        self.write_state("retreated_with_parcel", odom_back_m=round(got, 4), arm="front_view")

    def run(self):
        try:
            if getattr(self.args, "label_check", False):
                # 움직임 확인용: 팔만 기울이며 송장 숫자 찾기(바퀴·지게팔 명령 없음)
                if not self.args.target_label:
                    raise Stop("--label-check needs --target-label")
                bearing = self.scan_label()
                self.summary.update(ok=True, result="label %s found, bearing %.1f deg" % (self.args.target_label, bearing))
                return True
            self.preflight()
            if getattr(self.args, "roll_from_reference", False) or getattr(self.args, "align", False):
                self.fixed_roll = self.stored_roll()
            if self.args.target_label and getattr(self.args, "scan_label", False):
                self.scan_label()
            elif not self.args.target_label and self.s.get("search_tilts"):
                self.search_parcel()
            r, m, roll = self.reference()
            ins = self.approach(r, m, roll)
            if self.args.plan_only:
                self.summary.update(ok=True, result="plan only (no motion)")
                return True
            if self.args.no_lift:
                self.summary.update(ok=True, result="inserted (lift skipped by option)")
                return True
            self.lift_and_carry()
            self.retreat()
            self.summary.update(ok=True, result="lifted and retreated", inserted=ins)
            return True
        except Stop as exc:
            self.summary["reason"] = str(exc)
            self.log("STOP", reason=str(exc))
            return False
        except KeyboardInterrupt:
            self.summary["reason"] = "operator interrupt"
            self.log("STOP", reason="operator interrupt")
            return False
        finally:
            if getattr(self.base, "ready", False):
                self.base.stop()
            if self.recorder is not None:
                self.summary["recording"] = self.recorder.stop()
            if self.lift is not None:
                try:
                    self.lift.close()
                except Exception as exc:  # noqa: BLE001 — 정리 실패도 기록
                    self.log("lift_close_error", error=str(exc))
            try:
                self.cam.status("straight: %s" % ("OK" if self.summary["ok"] else "STOP " + self.summary.get("reason", "")[:40]))
            except Exception:  # noqa: BLE001
                pass
            self.summary["odom_total"] = {k: round(v, 4) for k, v in self.odom.items()}
            self.summary["duration_s"] = round(time.monotonic() - self.t0, 1)
            (self.run_dir / "summary.json").write_text(json.dumps(self.summary, indent=1, ensure_ascii=False, default=str))
            self.events.close()


def load(cfg_path):
    cfg = yaml.safe_load(Path(cfg_path).read_text())
    from tape_vision import TEACH_FILE, TapeEstimator, teach_problem
    teach = json.loads((HERE / TEACH_FILE).read_text())
    problem = teach_problem(teach, cfg)
    if problem:
        raise SystemExit("teach_tape.json rejected: %s" % problem)
    return cfg, teach, TapeEstimator(cfg, float(teach.get("focal_scale", 1.0)), ROOT)


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target-label", help="송장 숫자(예: 402). 없으면 택배가 올라간 팔레트")
    ap.add_argument("--plan-only", action="store_true", help="움직임 없이 점검·기준 기록·계획(팔은 보기 자세 B 로)")
    ap.add_argument("--no-lift", action="store_true", help="삽입까지만")
    ap.add_argument("--label-check", action="store_true", help="팔만 정면 자세로 돌려 송장 숫자 판독 결과만 출력(바퀴·지게팔 없음)")
    ap.add_argument("--scan-label", action="store_true", help="숫자 모드: 정면 자세에서 숫자를 읽어 그 방향 팔레트를 고른다")
    ap.add_argument("--align", action="store_true", help="고른 팔레트 정면 축으로 회전+직진 정렬 뒤 직진 삽입(저장된 roll 사용)")
    ap.add_argument("--roll-from-reference", action="store_true", help="저장된 정면 기준 roll 사용(지금 배치를 정면으로 보지 않음)")
    ap.add_argument("--roll-deg", type=float, help="카메라 roll 직접 지정(도)")
    ap.add_argument("--config", default=str(HERE / "config.yaml"))
    ap.add_argument("--name", default="")
    args = ap.parse_args(argv)
    args.fake = False
    return args


def main(argv=None):
    args = parse(argv)
    cfg, teach, est = load(args.config)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = LOG_ROOT / ("%s_straight%s%s" % (stamp, "_plan" if args.plan_only else "", ("_" + args.name) if args.name else ""))
    holder = {}

    def log(kind, **f):
        holder["p"].log(kind, **f)

    camera = RealCamera(cfg)
    arm = RealArmViews(cfg, log)
    base = RealStraightBase(cfg, log)
    lift = None if (args.plan_only or args.no_lift or args.label_check) else LiftCarry(cfg, log)
    p = StraightPickup(cfg, args, est, teach, camera, arm, base, lift, run_dir)
    holder["p"] = p
    try:
        ok = p.run()
    finally:
        arm.close()
    print("RESULT %s %s -> %s" % ("OK" if ok else "STOP", p.summary.get("reason", p.summary.get("result", "")), run_dir),
          flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
