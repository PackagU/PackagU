#!/usr/bin/env python3
"""택배 자동 집기 실기 실행기 — 팔 카메라로 받침을 보고 좌우·각도 정렬, odom 으로 삽입, 지게팔 상승, 후진.
인식은 설정 vision.mode: tape(받침 앞면 검정 표식 2개, 10/31 기본, 보정 teach_tape.json) / pallet(구멍 줄) / marker(ArUco).

전방 초음파가 아직 없어(2026-10-30) 접근 거리는 카메라(시트까지 거리)와 바퀴 odom 으로 정한다.
기존 지게팔 초음파(상승 중 10 cm 미만 정지)는 높이 제한 전용이고 여기서는 접근에 쓰지 않는다.

단계 (Jetson 호스트, 카메라 서버를 먼저 켠다: ~/pickup_venv/bin/python apps/pickup_real/camera_server.py)
  status   카메라·teach·장치 상태만 본다(움직임 없음)
  look     카메라 새 프레임 중앙값 한 번(움직임 없음) → 렌즈↔앞면 거리·좌우·각도. 줄자 실측과 대조용
  calibrate 표식 모드: 팔 보기 자세 → 받침 가운데로 제자리 회전 → 12 cm 후진·전진 → teach_tape.json(바퀴·팔 움직임)
  teach    사람이 정렬해 둔 predock 자세에서 기준을 기록(팔만 보기 자세로 움직임). --gap 은 줄자 값
  align    보기→이동→보기 반복으로 predock 에 정렬하고 멈춘다
  insert   predock 에서 구간마다 다시 보며 삽입 깊이까지 전진하고 멈춘다(상승 안 함)
  lift     insert 성공 직후에만: 지게팔 상승 → 후진 이탈
  run      align → insert → (확인) → lift 를 한 번에
공통: --dry-run 은 카메라만 실제로 보고 팔·바퀴·지게팔 명령은 기록만 한다. --fake 는 합성 카메라·가짜 장치(데스크톱 시험).

로봇을 움직이는 단계(teach 의 팔, align, insert, lift, run)는 사용자가 현장에서 지시할 때만 실행한다(AGENTS.md §0.4).
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from geometry import fork_errors, plan_moves, robot_from_camera, teach_mount, wrap  # noqa: E402

LOG_ROOT = ROOT / "logs" / "pickup_real"
STATE_FILE = LOG_ROOT / "state.json"


def scaled(cam, teach):
    """teach 의 position_scale(움직여서 잰 거리 배율)을 카메라 위치에 적용."""
    k = float((teach or {}).get("position_scale", 1.0))
    out = dict(cam)
    out["x"], out["y"] = cam["x"] * k, cam["y"] * k
    return out


class Stop(RuntimeError):
    """안전 정지: 이후 지게팔 상승을 하지 않는다."""


def default_teach_path(cfg):
    """모드별 보정 파일: tape → teach_tape.json(구멍 모드 teach.json 과 분리), 그 밖 → teach.json."""
    if cfg.get("vision", {}).get("mode") == "tape":
        from tape_vision import TEACH_FILE
        return HERE / TEACH_FILE
    return HERE / "teach.json"


def pocket_left(cfg):
    if cfg.get("vision", {}).get("mode") == "tape":
        return float(cfg["tape"].get("pocket_left_m", 0.0))
    return float(cfg["marker"]["pocket_left_m"])


# ===================================================================== devices
class HttpCamera:
    def __init__(self, port, timeout_pad=4.0):
        self.base = "http://127.0.0.1:%d" % port
        self.pad = timeout_pad

    def _get(self, path, timeout):
        with urllib.request.urlopen(self.base + path, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))

    def look(self, tag, n=None, timeout=None, target_hint=None):
        q = {"tag": tag}
        if n:
            q["n"] = n
        if timeout:
            q["timeout"] = timeout
        if target_hint:
            q["target_hint"] = json.dumps(target_hint)
        try:
            return self._get("/api/look?" + urllib.parse.urlencode(q), (timeout or 10) + self.pad)
        except OSError as exc:
            return {"ok": False, "reason": "camera server unreachable: %s" % exc}

    def status(self, text):
        try:
            self._get("/api/status?" + urllib.parse.urlencode({"text": text}), 2)
        except OSError:
            pass

    def state(self):
        try:
            return self._get("/api/state", 3)
        except OSError as exc:
            return {"error": str(exc)}


class DryLog:
    """--dry-run: 명령을 기록만 한다."""

    def __init__(self, log, name):
        self.log, self.name = log, name

    def __getattr__(self, item):
        def call(*args, **kwargs):
            self.log("dry_" + self.name, action=item, args=[repr(a) for a in args], kwargs=kwargs)
            if item in ("turn", "straight"):
                return {"ok": True, "dry": True}
            if item == "lift_up":
                return {"result": "dry", "seconds": 0}
            return {"ok": True, "dry": True}
        return call


class RealArm:
    """팔 서보(호스트 시리얼). 보기 자세 = Jetson camera_views.json, 수납 = button_arm_test 의 현장 HOME."""

    def __init__(self, cfg, log):
        a = cfg["arm"]
        self.cfg, self.log = a, log
        views = json.loads((ROOT / a["views_file"]).read_text())
        sys.path.insert(0, str(ROOT / "apps" / "button_arm_test"))
        import importlib.util
        spec = importlib.util.spec_from_file_location("pickup_servo_reference", str(ROOT / a["stow_file"]))
        proto = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(proto)
        self.proto = proto
        view = views.get(a["view"]) or a.get("view_pose")
        if not isinstance(view, dict):
            raise Stop("arm view pose %s not found" % a["view"])
        self.poses = {"view": {k: int(v) for k, v in view.items()}, "stow": dict(proto.HOME)}
        for name, pose in self.poses.items():
            for sid, pwm in pose.items():
                proto.check_pwm(sid, pwm)
        self.driver = None
        self.current = None

    def _open(self):
        if self.driver is None:
            self.driver = self.proto.SerialPoseDriver(self.cfg["port"], 115200, timeout=0.5)
            time.sleep(0.5)
        return self.driver

    def _go(self, name):
        if self.current == name:
            return
        d = self._open()
        d.send_pose(name, int(self.cfg["move_ms"]), self.poses)
        time.sleep(int(self.cfg["move_ms"]) / 1000.0)
        deadline = time.monotonic() + 2.0
        while True:
            pos = d.read_positions()
            if self.proto.positions_reached(pos, self.poses[name], int(self.cfg["tolerance_pwm"])):
                break
            if time.monotonic() > deadline:
                d.stop_all()
                raise Stop("arm did not reach %s: %s" % (name, pos))
            time.sleep(0.1)
        time.sleep(float(self.cfg["settle_s"]))
        self.current = name
        self.log("arm", pose=name, positions=pos)

    def view(self):
        self._go("view")

    def stow(self):
        if not self.cfg.get("keep_view_while_driving"):
            self._go("stow")

    def stow_for_lift(self):
        self._go("stow")

    def close(self):
        if self.driver is not None:
            self.driver.close()


class RealBase:
    """바퀴: 기존 진행기 FieldStack(fieldctl·직진 helper) + 이 폴더의 제자리 회전 helper."""

    def __init__(self, cfg, log):
        sys.path.insert(0, str(ROOT / "scripts"))
        from field_video.backends import FieldStack
        self.cfg, self.log = cfg["base"], log
        self.stack = FieldStack(self.cfg["container"], ROOT, output=lambda line: log("base_out", line=line))
        self.ready = False

    def prepare(self):
        if not self.stack.container_running():
            raise Stop("container %s not running" % self.stack.container)
        rc, out = self.stack.exec_capture("ros2 node list --no-daemon --spin-time 4", 15)
        if rc != 0 or "/nav_safety_gate" not in out or "/packagu_opencr_bridge" not in out:
            raise Stop("drive stack not running (gate/OpenCR node missing) — fieldctl base start --drive 먼저")
        # Nav2 를 켜지 않은 시험: goal 이 있을 수 없으니 /cmd_vel 다른 발행자만 확인한다
        rc, info = self.stack.exec_capture("ros2 topic info /cmd_vel --verbose --no-daemon --spin-time 4", 20)
        pubs, section = [], None
        for line in info.splitlines():
            if line.startswith("Publisher count"):
                section = "pub"
            elif line.startswith("Subscription count"):
                section = "sub"
            elif section == "pub" and line.strip().startswith("Node name:"):
                pubs.append(line.split(":", 1)[1].strip())
        allow = ("behavior_server", "velocity_smoother", "controller_server") if self.cfg.get("allow_idle_nav2") else ()
        others = [n for n in pubs if n not in allow]
        if rc != 0 or others:
            raise Stop("/cmd_vel already has publisher(s) %s — teleop/web UI 끄기" % (others or "?"))
        if pubs:
            # 쉬는 Nav2 만 허용: 2 초 동안 /cmd_vel 메시지가 없어야 한다(목표 진행 중이면 계속 나온다)
            rc2, echo = self.stack.exec_capture("timeout 2 ros2 topic echo --once /cmd_vel", 10)
            if "linear" in echo:
                raise Stop("/cmd_vel is active (Nav2 goal or teleop running) — 목표 취소 뒤 다시")
        try:
            self.stack.resume()
        except Exception as exc:  # noqa: BLE001 — BackendError
            raise Stop("gate resume failed: %s" % exc)
        self.ready = True
        self.log("base", action="ready (gate resumed)", gate=out.strip()[-60:])

    def _helper(self, args, tag):
        target = "/tmp/packagu_pickup_ros_turn.py"
        done = subprocess.run(["docker", "cp", str(HERE / "ros_turn.py"), "%s:%s" % (self.stack.container, target)],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        if done.returncode != 0:
            raise Stop("docker cp ros_turn.py failed: " + done.stdout.strip())
        argv = ["docker", "exec", "-i", "-w", "/ros2_ws", self.stack.container, "bash", "-lc",
                "source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 " + target + " " + args]
        rc, lines = self.stack._stream(argv)
        line = next((l.split(tag, 1)[1] for l in reversed(lines) if tag in l), None)
        data = json.loads(line) if line else {"ok": False, "reason": "no result line"}
        data["rc"] = rc
        return data

    def turn(self, rad):
        data = self._helper("--angle-deg %.2f --speed %.2f" % (math.degrees(rad), self.cfg["turn_speed_rad_s"]),
                            "TURN_RESULT ")
        if not data.get("ok"):
            raise Stop("turn failed: %s" % data.get("reason"))
        return data

    def straight(self, metres, speed):
        ok, data = self.stack.straight(metres, speed)
        if not ok:
            raise Stop("straight failed: %s" % data.get("reason"))
        return data

    def straight_until(self, max_metres, speed, check):
        """check() = None 계속 / 'stop' 정상 정지 / 'fail:<이유>' 실패(기존 진행기 도우미, 표준입력 STOP)."""
        ok, data = self.stack.straight_until(max_metres, speed, check)
        if not ok:
            raise Stop("straight_until failed: %s" % data.get("reason"))
        return data

    def stop(self):
        for attempt in (1, 2):   # 부하 속 첫 시도가 응답 확인을 못 할 때가 있다(192호 2026-11-01) → 한 번 더
            try:
                self.stack.stop()
                self.log("base", action="software stop", attempt=attempt)
                return
            except Exception as exc:  # noqa: BLE001 — report but keep cleaning up
                if attempt == 2:
                    self.log("base", action="software stop FAILED — 물리 비상정지", error=str(exc))


class RealLift:
    def __init__(self, cfg, log):
        sys.path.insert(0, str(ROOT / "scripts"))
        from field_video.backends import LiftUno
        self.cfg, self.log = cfg["lift"], log
        self.lift = LiftUno(self.cfg["port"], output=lambda line: log("lift_out", line=line))

    def lift_up(self):
        self.lift.open()
        try:
            res = self.lift.move("up", float(self.cfg["up_timeout_s"]))
        finally:
            self.lift.wait_until_stopped(8)
            self.lift.close()
        return res


# ===================================================================== fake world (desktop test)
class FakeWorld:
    """합성 카메라 + 오차 있는 구동. 제어기는 이 정답 자세를 읽지 않는다(카메라 영상만 받는다)."""

    def __init__(self, cfg, start, seed=1, mount=None, slip=None):
        from vision import intrinsics
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)
        self.pose = list(start)
        self.mount = mount or {"forward": 0.12, "left": 0.03, "yaw": math.radians(1.5), "height": 0.18,
                               "pitch": math.radians(15)}
        self.K_true, _, _ = intrinsics(dict(cfg["camera"], hfov_deg=cfg["camera"]["hfov_deg"] * 1.04))
        self.arm_view = False
        self.slip = slip or {"straight_scale": 0.97, "turn_scale": 1.03, "drift_rad_per_m": math.radians(1.0)}
        self.blocked_after = None  # 시험: 이 거리 이후 바퀴가 미끄러져 전진 안 됨(odom 은 감)
        self.inserted_depth = None
        self.lifted = False
        self.commands = []

    def camera_pose(self):
        x, y, th = self.pose
        m = self.mount
        cx = x + m["forward"] * math.cos(th) - m["left"] * math.sin(th)
        cy = y + m["forward"] * math.sin(th) + m["left"] * math.cos(th)
        return (cx, cy, wrap(th + m["yaw"]), m["height"], m["pitch"])

    def frame(self):
        from vision import render_sheet_view
        time.sleep(1 / 15.0)
        if not self.arm_view:
            return np.full((480, 640, 3), 90, np.uint8)  # 천장
        return render_sheet_view(self.cfg, self.K_true, self.camera_pose(), noise=2.0, rng=self.rng)

    def turn(self, rad):
        self.commands.append(("turn", rad))
        self.pose[2] = wrap(self.pose[2] + rad * self.slip["turn_scale"] + self.rng.normal(0, math.radians(0.3)))
        return {"ok": True}

    def straight(self, d, speed):
        self.commands.append(("straight", d))
        real = d * self.slip["straight_scale"]
        if self.blocked_after is not None:
            real = 0.0
        th = self.pose[2] + self.slip["drift_rad_per_m"] * abs(d) / 2
        self.pose[0] += real * math.cos(th)
        self.pose[1] += real * math.sin(th)
        self.pose[2] = wrap(self.pose[2] + self.slip["drift_rad_per_m"] * abs(d) * np.sign(self.rng.normal()))
        return {"ok": True, "traveled_m": abs(d)}


class FakeArm:
    def __init__(self, world, log):
        self.world, self.log = world, log

    def view(self):
        self.world.arm_view = True

    def stow(self):
        self.world.arm_view = False

    def stow_for_lift(self):
        self.world.arm_view = False

    def close(self):
        pass


class FakeBase:
    def __init__(self, world, log):
        self.world, self.log = world, log

    def prepare(self):
        pass

    def turn(self, rad):
        return self.world.turn(rad)

    def straight(self, d, speed):
        return self.world.straight(d, speed)

    def straight_until(self, max_d, speed, check):
        moved = 0.0
        while moved < max_d:
            result = check()
            if result == "stop":
                break
            if result:
                raise Stop("straight_until failed: %s" % result[5:])
            self.world.straight(0.005, speed)
            moved += 0.005
        return {"ok": True, "traveled_m": moved}

    def stop(self):
        self.log("base", action="software stop (fake)")


class FakeLift:
    def __init__(self, world):
        self.world = world

    def lift_up(self):
        self.world.lifted = True
        self.world.pose_at_lift = list(self.world.pose)  # 평가 전용(제어기는 읽지 않음)
        return {"result": "complete", "seconds": 0.0}


# ===================================================================== runner
class Pickup:
    def __init__(self, cfg, args, camera, arm, base, lift, run_dir):
        self.cfg, self.args = cfg, args
        self.mode = cfg.get("vision", {}).get("mode", "marker")
        if not getattr(args, "teach", None):
            args.teach = str(default_teach_path(cfg))
        self.cam, self.arm, self.base, self.lift = camera, arm, base, lift
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.events = (self.run_dir / "events.jsonl").open("a", encoding="utf-8")
        self.t0 = time.monotonic()
        self.summary = {"mode": args.mode, "dry_run": bool(args.dry_run), "fake": bool(args.fake), "ok": False,
                        "looks": 0, "realigns": 0, "moves": []}
        self.teach = None
        self.front = None   # 전방 초음파(reading() → (cm, 나이 s)). build() 가 front_range.enabled 일 때 붙인다
        self.predicted_pose = None  # 직전 목표 대비 실제 이동 피드백으로 예측, 다른 팔레트 재획득 방지
        self.last_camera_pose = None
        self.last_full_pitch = None
        self.last_full_at = 0.0
        self.distance_since_full = 0.0
        self.last_measurement_mode = None

    # -------------------------------------------------------------- utils
    def log(self, kind, **fields):
        rec = {"t": round(time.monotonic() - self.t0, 3), "kind": kind}
        rec.update(fields)
        self.events.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
        self.events.flush()
        if kind not in ("base_out",):
            short = {k: v for k, v in fields.items() if k not in ("positions",)}
            print("[%6.1f] %-10s %s" % (rec["t"], kind, json.dumps(short, ensure_ascii=False, default=str)[:220]),
                  flush=True)

    def stage(self, name):
        self.summary["stage"] = name
        self.log("stage", name=name)
        self.cam.status("stage: " + name)
        if time.monotonic() - self.t0 > float(self.cfg["limits"]["total_timeout_s"]):
            raise Stop("total timeout %.0f s" % float(self.cfg["limits"]["total_timeout_s"]))

    def load_teach(self):
        path = Path(self.args.teach)
        if not path.exists():
            raise Stop("teach 기록 없음(%s) — 먼저 calibrate(또는 teach) 단계" % path)
        self.teach = json.loads(path.read_text())
        if self.mode == "tape":
            # 옛 구멍용 teach(길이·극성) 나 설정이 바뀐 뒤의 보정은 쓰지 않는다
            from tape_vision import teach_problem
            problem = teach_problem(self.teach, self.cfg)
            if problem:
                raise Stop("teach rejected for tape mode: %s" % problem)
        self.mount = {"forward": self.teach["mount"]["forward"], "left": self.teach["mount"]["left"],
                      "yaw": self.teach["mount"]["yaw"]}
        self.target = (self.teach["predock_x"], self.teach["target_y"], 0.0)
        self.insert_total = float(self.teach["gap_m"]) + float(self.cfg["insert"]["depth_m"])
        # 정렬은 predock 보다 look_back_m 뒤(받침이 화면에 확실히 보이는 곳)에서 끝낸다. 남은 거리는 삽입이 맡는다
        back = float(self.cfg["align"].get("look_back_m", 0.0))
        self.look_target = (self.target[0] - back, self.target[1], 0.0)
        self.log("teach", file=str(path), mount=self.mount, target=self.target, look_target=self.look_target,
                 insert_total=self.insert_total)

    def look(self, tag):
        """카메라 한 번 보기. 흔들림(unstable)이면 0.5 s 쉬고 한 번 더 본다(멈춘 직후 흔들림·튀는 프레임)."""
        hint = None
        if self.mode == "tape" and self.cfg["align"].get("autonomous_view", False):
            if self.predicted_pose is not None:
                hint = {"expected_robot": list(self.predicted_pose)}
                if self.last_full_pitch is not None and self.distance_since_full <= 0.25 and time.monotonic() - self.last_full_at <= 120:
                    hint.update(allow_partial=True, pitch_down=self.last_full_pitch)
            elif getattr(self.args, "target_side", None):
                hint = {"side": self.args.target_side}
        r = self.cam.look(tag, target_hint=hint) if hint else self.cam.look(tag)
        if not r.get("ok") and str(r.get("reason", "")).startswith("unstable"):
            self.log("look_retry", tag=tag, reason=r.get("reason"), samples_y_mm=r.get("samples_y_mm"),
                     samples_heading_deg=r.get("samples_heading_deg"))
            time.sleep(0.5)
            r = self.cam.look(tag + "_retry", target_hint=hint) if hint else self.cam.look(tag + "_retry")
        return r

    def look_pose(self, tag, allow_lost=False, recover_view=False):
        """한 번 보고 (로봇 자세, 포크 오차). allow_lost=True 면 받침이 화면에 없을 때 Stop 대신 (None, None)."""
        self.arm.view()
        r = self.look(tag)
        if recover_view and self.mode == "tape" and self.cfg["align"].get("autonomous_view", False):
            budget = int(self.cfg["align"].get("max_view_recoveries", 3))
            while not r.get("ok") and not self.args.dry_run:
                hint = r.get("recovery") or {}
                used = int(self.summary.get("view_recoveries", 0))
                if hint.get("kind") != "edge" or hint.get("edge") not in ("left", "right") or used >= budget:
                    break
                angle = math.radians(float(self.cfg["align"].get("view_recovery_deg", 3)))
                if hint["edge"] == "right":
                    angle = -angle
                self.summary["view_recoveries"] = used + 1
                self.log("view_recovery", attempt=used + 1, edge=hint["edge"], turn_deg=math.degrees(angle))
                self.execute([("turn", angle)], float(self.cfg["base"]["align_speed_m_s"]))
                self.arm.view()
                r = self.look(tag + "_view%d" % (used + 1))
        self.summary["looks"] += 1
        self.log("look", tag=tag, **{k: r.get(k) for k in ("ok", "reason", "cam", "spread", "age_s", "good",
                                                          "frames_seen", "reprojection_px", "image", "inliers",
                                                          "samples_y_mm", "measurement_mode")})
        if not r.get("ok"):
            if allow_lost and r.get("good", 0) == 0 and r.get("frames_seen", 0) > 0:
                return None, None   # 새 프레임은 오는데 받침이 하나도 안 잡힘 = 너무 가까워 화면 밖
            raise Stop("vision (%s): %s" % (tag, r.get("reason")))
        pose = robot_from_camera(scaled(r["cam"], self.teach), self.mount)
        if self.mode == "tape" and self.cfg["align"].get("autonomous_view", False) and self.predicted_pose is not None:
            if (math.hypot(pose[0] - self.predicted_pose[0], pose[1] - self.predicted_pose[1]) > 0.12
                    or abs(wrap(pose[2] - self.predicted_pose[2])) > math.radians(8)):
                raise Stop("selected target disagrees with movement feedback")
        self.predicted_pose = pose
        self.last_camera_pose = dict(r["cam"], pitch_down=math.radians(float(r.get("pitch_down_deg", 34))))
        self.last_measurement_mode = r.get("measurement_mode", "two_markers")
        if self.last_measurement_mode != "single_endpoint+odometry":
            self.last_full_pitch = self.last_camera_pose["pitch_down"]
            self.last_full_at = time.monotonic()
            self.distance_since_full = 0.0
        else:
            self.log("partial_pose", source="one visible endpoint + known spacing + odometry heading",
                     spacing_m=self.cfg["tape"]["center_spacing_m"], distance_since_full_m=self.distance_since_full)
        err = fork_errors(pose, self.target)
        self.log("pose", tag=tag, robot=[round(v, 4) for v in pose],
                 lateral_cm=round(err["lateral"] * 100, 2), yaw_deg=round(math.degrees(err["yaw"]), 2),
                 along_cm=round(err["along"] * 100, 2))
        return pose, err

    def execute(self, moves, speed):
        """이동 실행. 반환: 직진마다 odom 이 잰 실제 이동 거리(m, 도우미가 안 알려 주면 명령값)."""
        self.arm.stow()
        traveled = []
        for kind, value in moves:
            self.summary["moves"].append([kind, round(math.degrees(value) if kind == "turn" else value, 4)])
            self.log("move", what=kind, value_deg=round(math.degrees(value), 2) if kind == "turn" else None,
                     value_m=round(value, 4) if kind == "straight" else None)
            if kind == "turn":
                data = self.base.turn(value) or {}
                actual = math.radians(float(data["turned_deg"])) if data.get("turned_deg") is not None else value
                if self.predicted_pose is not None:
                    x, y, h = self.predicted_pose
                    self.predicted_pose = (x, y, wrap(h + actual))
            else:
                data = self.base.straight(value, speed) or {}
                odom = data.get("traveled_m") if isinstance(data, dict) else None
                traveled.append(abs(float(odom)) if odom is not None and not data.get("dry") else abs(value))
                if self.predicted_pose is not None:
                    x, y, h = self.predicted_pose
                    actual = math.copysign(traveled[-1], value)
                    self.predicted_pose = (x + actual * math.cos(h), y + actual * math.sin(h), h)
                    self.distance_since_full += abs(actual)
                if odom is not None:
                    self.log("moved", odom_m=odom, commanded_m=round(value, 4))
        return traveled

    def tol(self, look=False):
        """look=True: look point 정렬용. 앞뒤 거리는 삽입이 카메라로 다시 재므로 느슨하게(look_distance_tol_m)."""
        a = dict(self.cfg["align"])
        if self.mode == "tape":
            a.update((self.cfg.get("tape") or {}).get("align") or {})   # 표식 모드 전용 허용치(더 정밀한 각도)
        dist = float(a.get("look_distance_tol_m", a["distance_tol_m"])) if look else float(a["distance_tol_m"])
        return {"lateral": float(a["lateral_tol_m"]), "yaw": math.radians(float(a["yaw_tol_deg"])), "distance": dist}

    # -------------------------------------------------------------- stages
    def do_teach(self):
        """사람이 정렬한 predock 자세(포크 중심 = 포켓 중심, 정면)를 기준으로 기록."""
        self.stage("TEACH")
        if self.args.gap is None:
            raise Stop("--gap (predock 에서 포크 끝 ↔ 포켓 입구, 줄자 m) 이 필요")
        self.arm.view()
        r = self.cam.look("teach", n=15, timeout=10)
        self.log("look", tag="teach", **{k: r.get(k) for k in ("ok", "reason", "cam", "spread", "age_s", "image")})
        if not r.get("ok"):
            raise Stop("teach vision: %s" % r.get("reason"))
        base_scale = float(r.get("focal_scale", 1.0))
        scale = base_scale
        if self.args.camera_distance:
            # 줄자로 잰 카메라 렌즈 ↔ 시트 수평 거리로 초점거리 배율 보정(가정 화각 오차 제거) → 새 배율로 다시 본다
            scale = base_scale * float(self.args.camera_distance) / (-r["cam"]["x"])
            self.log("teach_scale", focal_scale=round(scale, 4), measured=self.args.camera_distance,
                     estimated=round(-r["cam"]["x"], 4))
            self.write_teach(r, scale, provisional=True)
            r = self.cam.look("teach_rescaled", n=15, timeout=10)
            self.log("look", tag="teach_rescaled", **{k: r.get(k) for k in ("ok", "reason", "cam", "spread", "image",
                                                                              "focal_scale")})
            if not r.get("ok"):
                raise Stop("teach vision after rescale: %s" % r.get("reason"))
        teach = self.write_teach(r, scale)
        self.log("teach_saved", file=self.args.teach, **teach)
        self.summary.update(ok=True, teach=teach)

    def write_teach(self, r, scale, provisional=False):
        cam = dict(r["cam"])
        r_cfg = self.cfg["robot"]
        target_y = pocket_left(self.cfg) - float(r_cfg["fork_center_left_m"])
        mount = teach_mount(cam, float(r_cfg["camera_forward_m"]), target_y)
        teach = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "focal_scale": scale,
                 "mount": {k: mount[k] for k in ("forward", "left", "yaw")}, "predock_x": mount["predock_x"],
                 "target_y": target_y, "gap_m": float(self.args.gap), "camera_seen": cam,
                 "pitch_down_deg": r.get("pitch_down_deg"), "height_above_sheet": r.get("height_above_sheet"),
                 "provisional": provisional,
                 "note": "사람이 정렬한 predock 자세 기준. camera_forward_m 은 설정 가정값"}
        if self.mode == "tape":
            from tape_vision import teach_fields
            teach.update(teach_fields(self.cfg), method="human-aligned teach (tape markers)")
        Path(self.args.teach).write_text(json.dumps(teach, indent=1, ensure_ascii=False))
        return teach

    def do_calibrate(self):
        """측정 없이 시작: 받침 찾기(제자리 회전) → 12 cm 후진·전진으로 거리 배율과 카메라 방향 오프셋을 잰다.
        검정 표식 모드: 찾기 회전 없음(두 표식이 처음부터 보여야 함), 중심 간격·카메라 높이는 설정 실측값 그대로,
        후진·전진으로는 초점 배율만 맞추고 직진 중 yaw 가 일정한지 확인한다."""
        tape = self.mode == "tape"
        self.stage("SEARCH")
        self.teach = {"position_scale": 1.0}
        step = math.radians(8.0)
        r = None
        for i in range(6):
            self.arm.view()
            r = self.look("search_%d" % i)
            self.log("look", tag="search_%d" % i, **{k: r.get(k) for k in ("ok", "reason", "cam", "image")})
            if r.get("ok"):
                break
            reason = str(r.get("reason", "")) + " " + " ".join(str(k) for k in (r.get("reasons") or {}))
            if tape:
                # 한쪽 표식만 보이거나 잘려도 반대쪽 위치를 추측해 돌지 않는다
                raise Stop("search: both tape markers not seen (%s) — 두 표식이 화면 안에 보이게 로봇을 놓고 다시"
                           % reason.strip())
            if reason.startswith("left post") or "border (left)" in reason:
                self.execute([("turn", step)], 0.05)
            elif reason.startswith("right post") or "border (right)" in reason:
                self.execute([("turn", -step)], 0.05)
            else:
                raise Stop("search: pallet not seen (%s)" % reason.strip())
        if not r or not r.get("ok"):
            raise Stop("search: both posts not in view after turns")
        # 받침을 화면 가운데로: 가장자리에서는 렌즈 왜곡·끝 구멍 흔들림으로 인식값이 틀어진다(10/30 실기, 10/31 시뮬)
        center_tol = math.radians(float(self.cfg["align"].get("center_tol_deg", 4.0)))
        for k in range(3):
            # 카메라 → 받침 앞면 중심(0, 0) 방향에서 카메라가 지금 보는 방향을 뺀 각 = 돌 각도(+ 왼쪽)
            bearing = wrap(math.atan2(-r["cam"]["y"], -r["cam"]["x"]) - r["cam"]["heading"])
            if abs(bearing) <= center_tol:
                break
            if k == 2:
                # 회전 중심이 카메라 뒤 0.34 m 라 돌면 카메라도 옆으로 간다. 2번 돌아도 벗어나 있으면 멈춘다(Codex r2)
                raise Stop("center: pallet still %.1f deg off-center after 2 turns" % math.degrees(bearing))
            if abs(math.degrees(bearing)) > float(self.cfg["base"]["max_turn_deg"]):
                raise Stop("center: turn %.1f deg exceeds max_turn_deg" % math.degrees(bearing))
            # 회전 중심이 카메라 뒤 camera_forward_m 에 있어 bearing 만큼 돌면 카메라도 옆으로 가서 넘친다
            # (10/31 실기: +6.6° 돌자 -9.3°). 회전 중심에서 본 받침 방향만큼 돈다
            d = math.hypot(r["cam"]["x"], r["cam"]["y"])
            c_fwd = float(self.cfg["robot"]["camera_forward_m"])
            turn = math.atan2(d * math.sin(bearing), d * math.cos(bearing) + c_fwd)
            self.log("center", bearing_deg=round(math.degrees(bearing), 2), turn_deg=round(math.degrees(turn), 2),
                     distance_m=round(d, 3), attempt=k)
            self.execute([("turn", turn)], 0.05)
            self.arm.view()
            r = self.look("center_%d" % k)
            self.log("look", tag="center_%d" % k, **{kk: r.get(kk) for kk in ("ok", "reason", "cam", "image")})
            if not r.get("ok"):
                raise Stop("center: pallet lost after turning (%s)" % r.get("reason"))
        self.stage("CALIBRATE")
        d = float(self.args.cal_distance)
        looks = [r]
        moved_odom = self.execute([("straight", -d)], 0.05)
        self.arm.view()   # 주행 중 수납했으면 보기 자세로(keep_view_while_driving 이면 그대로라 움직이지 않음)
        back = self.look("cal_back")
        if not back.get("ok"):
            # 후진 사진이 나쁘면 다시 전진하지 않고 그 자리에서 멈춘다(Codex r2)
            self.log("look", tag="cal_1", **{kk: back.get(kk) for kk in ("ok", "reason", "cam", "spread", "image")})
            raise Stop("calibrate vision 1 (after backing up): %s" % back.get("reason"))
        looks.append(back)
        moved_odom += self.execute([("straight", d)], 0.05)
        self.arm.view()
        looks.append(self.look("cal_fwd"))
        for k, lk in enumerate(looks):
            self.log("look", tag="cal_%d" % k, **{kk: lk.get(kk) for kk in ("ok", "reason", "cam", "spread", "image",
                                                                         "inliers", "samples_y_mm", "tape_centres_px")})
            if not lk.get("ok"):
                raise Stop("calibrate vision %d: %s" % (k, lk.get("reason")))
        if tape:
            self.calibrate_tape(looks, moved_odom, d)
            return
        height, scale = self.fit_height(looks, d)
        if height is not None:
            looks = [dict(lk, cam=c) for lk, c in zip(looks, self._resolved)]
        c = [np.array([lk["cam"]["x"], lk["cam"]["y"]]) for lk in looks]
        d01, d12 = c[1] - c[0], c[2] - c[1]
        moved = np.linalg.norm(d01) + np.linalg.norm(d12)
        if moved < 0.5 * d:
            raise Stop("calibrate: camera barely moved (%.3f m for %.2f m odom)" % (moved, 2 * d))
        if height is None:
            scale = 2 * d / moved
        th_back = math.atan2(-d01[1], -d01[0])
        th_fwd = math.atan2(d12[1], d12[0])
        th_motion = math.atan2(math.sin(th_back) + math.sin(th_fwd), math.cos(th_back) + math.cos(th_fwd))
        heading = float(np.mean([lk["cam"]["heading"] for lk in looks]))
        cam_yaw_motion = wrap(heading - th_motion)
        # 12 cm 이동으로 잰 카메라 방향은 끝 구멍 흔들림에 민감(10/30 실기 12° 오차) → 기본은 설정값
        if self.cfg["robot"].get("calibrate_camera_yaw", False):
            cam_yaw = cam_yaw_motion
        else:
            cam_yaw = math.radians(float(self.cfg["robot"].get("camera_yaw_deg", 0.0)))
        r_cfg = self.cfg["robot"]
        gap = float(self.cfg["insert"]["gap_m"])
        teach = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "method": "motion calibration (no tape)",
                 "focal_scale": float(looks[-1].get("focal_scale", 1.0)), "position_scale": scale,
                 "camera_height_m": height, "row_length_m": getattr(self, "row_length", None),
                 "hole_polarity": getattr(self, "hole_polarity", None),
                 "mount": {"forward": float(r_cfg["camera_forward_m"]), "left": float(r_cfg["camera_left_m"]),
                           "yaw": cam_yaw},
                 "predock_x": -(float(r_cfg["fork_tip_forward_m"]) + gap),
                 "target_y": float(self.cfg["marker"]["pocket_left_m"]) - float(r_cfg["fork_center_left_m"]),
                 "gap_m": gap, "motion_heading_deg": math.degrees(th_motion),
                 "heading_back_fwd_deg": [math.degrees(th_back), math.degrees(th_fwd)],
                 "camera_yaw_by_motion_deg": math.degrees(cam_yaw_motion),
                 "camera_heading_deg": math.degrees(heading), "cal_distance_m": d,
                 "note": "카메라 앞 오프셋·포크 끝 위치·좌우 오프셋은 설정 가정값. 배율·카메라 방향만 움직여서 잼"}
        Path(self.args.teach).write_text(json.dumps(teach, indent=1, ensure_ascii=False))
        self.log("teach_saved", **teach)
        self.summary["calibration"] = teach

    def calibrate_tape(self, looks, moved_odom, d):
        """검정 표식 모드 보정: 세 사진의 표식 중심 중앙값으로 초점 배율만 맞춘다(중심 간격·카메라 높이·roll 은 설정 실측값).
        직진 후진·전진이라 카메라 yaw 가 거의 같아야 한다 → 다르면 yaw 추정이 불안정한 것이므로 정지."""
        from tape_vision import fit_focal, focal_from_distance, teach_fields
        cal = (self.cfg["tape"].get("calibrate") or {})
        centres = []
        for k, lk in enumerate(looks):
            c = lk.get("tape_centres_px")
            if not c:
                raise Stop("calibrate: photo %d has no tape marker centres" % k)
            centres.append((c["left"], c["right"]))
        dists = list(moved_odom[:2]) if len(moved_odom) >= 2 else [d, d]   # odom 실제 이동(후진, 전진)
        k, rms, cams = fit_focal(self.cfg, centres, dists)
        if k is None:
            raise Stop("calibrate: %s" % rms)
        heads = [math.degrees(q["heading"]) for q in cams]
        self.log("focal_fit", focal_scale=round(k, 4), rms_m=round(rms, 4), odom_m=[round(v, 4) for v in dists],
                 headings_deg=[round(h, 2) for h in heads], cam_x_m=[round(q["x"], 4) for q in cams],
                 cam_y_m=[round(q["y"], 4) for q in cams], pitches_deg=[round(math.degrees(q["pitch_down"]), 2) for q in cams])
        lim = float(cal.get("max_focal_change", 0.15))
        if abs(k - 1.0) > lim or k <= 0.802 or k >= 1.198:
            raise Stop("calibrate: focal scale %.3f outside 1 +- %.2f — 중심 간격(%.3f m)·화각 가정·받침 고정 확인"
                       % (k, lim, float(self.cfg["tape"]["center_spacing_m"])))
        if rms > float(cal.get("max_fit_rms_m", 0.008)):
            raise Stop("calibrate: camera motion does not match odom (rms %.1f mm)" % (rms * 1000))
        k_motion, source = k, "12 cm motion fit (about +-3 %)"
        if getattr(self.args, "camera_distance", None):
            # 줄자 거리(첫 사진 자세에서 렌즈 ↔ 받침 앞면 수평)가 있으면 그것으로 정하고 이동 맞춤과 대조한다
            k_tape, seen = focal_from_distance(self.cfg, centres[0], float(self.args.camera_distance))
            if k_tape is None:
                raise Stop("calibrate: %s" % seen)
            self.log("focal_tape", focal_scale=round(k_tape, 4), measured_m=self.args.camera_distance,
                     motion_focal_scale=round(k_motion, 4))
            if abs(k_tape - k_motion) > float(cal.get("max_tape_motion_disagree", 0.06)):
                raise Stop("calibrate: tape distance focal %.3f vs motion focal %.3f disagree — 줄자 위치·odom 확인"
                           % (k_tape, k_motion))
            k, source = k_tape, "tape-measured camera distance %.3f m" % float(self.args.camera_distance)
            from tape_vision import TapeEstimator
            est = TapeEstimator(self.cfg, k)
            cams = [est.solve_centres(a, b)[0] for a, b in centres]
            heads = [math.degrees(q["heading"]) for q in cams]
        if max(heads) - min(heads) > float(cal.get("max_heading_change_deg", 1.5)):
            raise Stop("calibrate: camera yaw changed %.2f deg over straight back/forward — yaw 불안정 또는 바퀴 틀어짐"
                       % (max(heads) - min(heads)))
        c = [np.array([q["x"], q["y"]]) for q in cams]
        d01, d12 = c[1] - c[0], c[2] - c[1]
        th_back = math.atan2(-d01[1], -d01[0])
        th_fwd = math.atan2(d12[1], d12[0])
        th_motion = math.atan2(math.sin(th_back) + math.sin(th_fwd), math.cos(th_back) + math.cos(th_fwd))
        heading = float(np.mean([q["heading"] for q in cams]))
        cam_yaw_motion = wrap(heading - th_motion)
        r_cfg = self.cfg["robot"]
        cam_yaw = cam_yaw_motion if r_cfg.get("calibrate_camera_yaw", False) else \
            math.radians(float(r_cfg.get("camera_yaw_deg", 0.0)))
        gap = float(self.cfg["insert"]["gap_m"])
        teach = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "method": "tape motion calibration (focal only)",
                 "focal_scale": float(k), "focal_source": source, "focal_scale_by_motion": float(k_motion),
                 "position_scale": 1.0, "fit_rms_m": rms, "odom_m": dists,
                 "mount": {"forward": float(r_cfg["camera_forward_m"]), "left": float(r_cfg["camera_left_m"]),
                           "yaw": cam_yaw},
                 "predock_x": -(float(r_cfg["fork_tip_forward_m"]) + gap),
                 "target_y": pocket_left(self.cfg) - float(r_cfg["fork_center_left_m"]),
                 "gap_m": gap, "motion_heading_deg": math.degrees(th_motion),
                 "camera_yaw_by_motion_deg": math.degrees(cam_yaw_motion), "camera_heading_deg": [round(h, 3) for h in heads],
                 "cal_distance_m": d, "tape_centres_px": centres,
                 "note": "중심 간격·카메라 높이·roll·표식 높이는 설정 실측/가정값 그대로. 초점 배율만 움직여서 맞춤. "
                         "카메라 앞 오프셋·포크 끝·좌우 오프셋은 설정 가정값"}
        teach.update(teach_fields(self.cfg))
        Path(self.args.teach).write_text(json.dumps(teach, indent=1, ensure_ascii=False))
        self.log("teach_saved", **teach)
        self.summary["calibration"] = teach

    def fit_height(self, looks, d):
        """보정 사진 3장(후진 전·후진 후·전진 후)을 카메라 높이를 바꿔 가며 다시 풀어, 카메라 이동이 odom 과
        맞는 높이를 찾는다(받침 모드). 반환 (height, 1.0) 또는 (None, None).
        구멍 검출은 사진마다 한 번만 하고(높이·간격과 무관), 거친 격자 → 최적점 주변 고운 격자 순으로 푼다."""
        if self.cfg.get("vision", {}).get("mode") != "pallet":
            return None, None
        import cv2
        from pallet_vision import PalletEstimator
        base = PalletEstimator(self.cfg)
        feats = []
        for lk in looks:
            path = str(lk.get("image") or "").replace(".jpg", "_raw.png")
            img = cv2.imread(path) if path else None
            if img is None:
                return None, None
            f = base.features(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
            if f.get("reason"):
                return None, None
            feats.append(f)
        pol = {f.get("bright_holes") for f in feats}
        if len(pol) != 1:
            raise Stop("calibrate: hole polarity changed between photos (%s) — set pallet.hole_polarity" % pol)
        self.hole_polarity = "bright" if pol.pop() else "dark"

        def score(L, h):
            est = PalletEstimator(self.cfg)
            est.height = float(h)
            est.p = dict(est.p, row_length_m=float(L))   # L = 구멍 줄 양 끝 사이 길이(m)
            cams = []
            for f in feats:
                est.last = None
                r = est.estimate_features(f)
                if not r.get("ok"):
                    return None
                cams.append(r["cam"])
            c = [np.array([q["x"], q["y"]]) for q in cams]
            pitches = [q["pitch_down"] for q in cams]
            err = ((np.linalg.norm(c[1] - c[0]) - d) ** 2 + (np.linalg.norm(c[2] - c[1]) - d) ** 2
                   + (0.3 * (max(pitches) - min(pitches))) ** 2)
            return err, float(h), cams, float(L)

        H_LO, H_HI = 0.25, 0.85
        best = None
        L_LO, L_HI = 0.20, 0.45
        for L in np.arange(L_LO, L_HI + 0.0001, 0.01):
            for h in np.arange(H_LO, H_HI, 0.02):
                r = score(L, h)
                if r and (best is None or r[0] < best[0]):
                    best = r
        if best is not None:
            L0, h0 = best[3], best[1]
            for L in np.arange(max(L_LO, L0 - 0.01), min(L_HI + 0.0001, L0 + 0.011), 0.002):
                for h in np.arange(max(H_LO, h0 - 0.03), min(H_HI, h0 + 0.031), 0.005):
                    r = score(L, h)
                    if r and r[0] < best[0]:
                        best = r
        if best is None:
            return None, None
        self.log("height_fit", camera_height_m=round(best[1], 3), row_length_m=round(best[3], 4),
                 rms_m=round(math.sqrt(best[0] / 2), 4), pitches_deg=[round(math.degrees(q["pitch_down"]), 2) for q in best[2]])
        if (math.sqrt(best[0] / 2) > 0.02 or best[1] <= H_LO + 0.005 or best[1] >= H_HI - 0.02
                or best[3] <= L_LO + 0.005 or best[3] >= L_HI - 0.005):
            raise Stop("calibrate: camera height fit poor (h=%.2f, row %.3f m, rms %.3f m)"
                       % (best[1], best[3], math.sqrt(best[0] / 2)))
        self.row_length = best[3]
        self._resolved = best[2]
        return best[1], 1.0

    def do_align(self):
        self.stage("ALIGN")
        a = self.cfg["align"]
        gain = float(a.get("lateral_gain", 1.0))
        steps = int(getattr(self.args, "align_steps", 0) or 0)
        executed = 0
        prev = None
        for it in range(int(a["max_iterations"]) + 1):
            self.stage("ALIGN")
            pose, err = self.look_pose("align_%d" % it, recover_view=True)
            if self.mode == "tape" and a.get("autonomous_view", False):
                from autonomy import visible_look_x
                try:
                    view_x = visible_look_x(self.cfg, self.teach, self.last_camera_pose["pitch_down"], self.look_target[0])
                except ValueError as exc:
                    raise Stop("view planning: %s" % exc)
                if view_x < self.look_target[0] - 0.001:
                    self.look_target = (view_x, self.look_target[1], self.look_target[2])
                    self.log("view_target", robot_distance_m=-view_x, margin_px=a.get("visibility_margin_px", 64))
            if prev is not None and abs(prev["planned"]) > 0.005:
                # 실기 넘침 진단(10/30: 계획 13.6 cm 에 24 cm): 계획한 옆 이동 대비 실제로 바뀐 옆 위치
                moved = pose[1] - prev["y"]
                check = {"planned_cm": round(prev["planned"] * 100, 2), "seen_cm": round(moved * 100, 2),
                         "ratio": round(moved / prev["planned"], 3)}
                self.log("lateral_check", **check)
                self.summary.setdefault("lateral_check", []).append(check)
            # 옆 오차가 클 때만 이득을 줄여 넘침을 막는다(작은 오차는 그대로 고쳐 허용치 안으로)
            ty = self.look_target[1]
            if abs(ty - pose[1]) > float(a["lateral_tol_m"]) * 2:
                ty = pose[1] + gain * (ty - pose[1])
            target = (self.look_target[0], ty, self.look_target[2])
            moves, note = plan_moves(pose, target, self.tol(look=True),
                                     math.radians(float(a["max_path_angle_deg"])), max_back=0.6)
            prev = {"y": pose[1], "planned": target[1] - pose[1]} if moves else None
            self.log("plan", iteration=it, note=note, moves=moves, lateral_gain=gain)
            if moves is None:
                raise Stop("plan: " + note)
            if not moves:
                if self.last_measurement_mode == "single_endpoint+odometry":
                    self.execute([("straight", -0.10)], float(self.cfg["base"]["align_speed_m_s"]))
                    self.look_target = (self.look_target[0] - 0.10, self.look_target[1], self.look_target[2])
                    self.log("full_view_check", reason="final yaw verification needs two observed markers")
                    continue
                self.summary["aligned"] = {"lateral_cm": round(err["lateral"] * 100, 2),
                                           "yaw_deg": round(math.degrees(err["yaw"]), 2),
                                           "along_cm": round(err["along"] * 100, 2), "iterations": it}
                self.save_state("aligned", pose=pose)
                return pose
            if it == int(a["max_iterations"]):
                break
            for kind, value in moves:
                if kind == "turn" and abs(math.degrees(value)) > float(self.cfg["base"]["max_turn_deg"]):
                    raise Stop("turn %.1f deg exceeds max_turn_deg" % math.degrees(value))
            if steps and executed >= steps:
                # 첫 실물: 한 번 움직이고 다시 본 결과(lateral_check)를 사람이 확인한 뒤 이어 간다(Codex r2)
                raise Stop("align step limit %d reached — lateral_check %s 확인 후 다시 align"
                           % (steps, (self.summary.get("lateral_check") or ["-"])[-1]))
            self.execute(moves, float(self.cfg["base"]["align_speed_m_s"]))
            executed += 1
            if self.args.dry_run:
                self.summary["aligned"] = {"dry_run_plan": moves}
                return pose
        raise Stop("not aligned after %d iterations" % int(a["max_iterations"]))

    def fix_yaw(self, err, tag):
        """카메라 없이 가는 구간 전에 남은 각도를 제자리 회전으로 줄인다(insert.correct_yaw_deg, 표식 모드만 켬).
        0.5 m 를 1° 로 가면 포크 끝이 약 9 mm 쓸린다. 회전 중심이 포크 끝 뒤라 회전 자체의 옆 이동은 작다."""
        lim = self.ins_cfg.get("correct_yaw_deg")
        if lim is None or abs(err["yaw"]) <= math.radians(float(lim)):
            return
        self.log("yaw_fix", tag=tag, yaw_deg=round(math.degrees(err["yaw"]), 2))
        self.execute([("turn", -err["yaw"])], float(self.cfg["base"]["insert_speed_m_s"]))

    def do_insert(self):
        self.stage("INSERT")
        ins, a = dict(self.cfg["insert"]), self.cfg["align"]
        if self.mode == "tape":
            ins.update((self.cfg.get("tape") or {}).get("insert") or {})
        self.ins_cfg = ins
        total = self.insert_total
        retries = 0
        while True:
            pose, err = self.look_pose("insert_start")
            tol = self.tol()
            if abs(err["lateral"]) > tol["lateral"] * 1.5 or abs(err["yaw"]) > tol["yaw"] * 1.5:
                if retries >= int(self.cfg["limits"]["max_retries"]):
                    raise Stop("not aligned at insert start (retries used)")
                retries += 1
                self.summary["realigns"] += 1
                self.do_align()
                continue
            self.fix_yaw(err, "insert_start")
            x0 = pose[0]
            goal_x = self.target[0] + total     # 완전 삽입 때 로봇 x (P). 카메라가 보일 때마다 남은 거리를 이것으로 고친다
            traveled, remaining, restart = 0.0, goal_x - x0, False
            if remaining > float(ins.get("max_total_m", 0.9)):
                raise Stop("insert distance %.2f m from vision is too long (look point too far?)" % remaining)
            if remaining < -float(ins["distance_tol_m"]):
                raise Stop("already past the insert target by %.3f m at start (vision)" % -remaining)
            while remaining > float(ins["distance_tol_m"]):
                blind = float(ins["blind_final_m"])
                last = remaining <= blind + 0.04
                seg = remaining if last else min(float(ins["segment_m"]), remaining - blind)
                self.stage("INSERT %.0f cm left" % (remaining * 100))
                if last and getattr(self.args, "pause_before_final", False):
                    # 첫 실물: 카메라 없이 가는 마지막 구간 전에 멈춘다. 줄자로 포크 끝 → 가로대 입구를 재서
                    # (남은 거리 - depth_m) 와 맞으면 insert 를 다시 실행(가까이서 다시 보고 마지막 구간만 간다)
                    # 처음부터 마지막 구간인 배치도 이동 전에 확인해야 한다.
                    self.save_state("paused_before_final", remaining=round(remaining, 4))
                    raise Stop("paused before final %.3f m blind (fork tip -> bar entrance should be about %.3f m by tape)"
                               % (remaining, remaining - float(ins["depth_m"])))
                if last and self.front is not None:
                    seg = self.final_by_range(seg)
                else:
                    self.execute([("straight", seg)], float(self.cfg["base"]["insert_speed_m_s"]))
                traveled += seg
                remaining -= seg
                if self.args.dry_run or last or remaining <= float(ins["distance_tol_m"]):
                    break  # 마지막 구간은 카메라 확인 없이(가까우면 받침이 화면 밖) odom 또는 전방 초음파로
                # 이 거리에서는 받침이 보여야 한다(blind_final_m 이 보이는 한계). 안 보이면 가림·고장으로 보고 정지
                pose, err = self.look_pose("insert_%03dcm" % round(traveled * 100))
                disagree = (pose[0] - x0) - traveled
                remaining = goal_x - pose[0]
                self.log("insert_check", traveled_odom_m=round(traveled, 4), vision_minus_odom_m=round(disagree, 4),
                         remaining_by_vision_m=round(remaining, 4))
                if abs(disagree) > float(a["odom_vision_disagree_m"]):
                    raise Stop("odom/vision disagree %.3f m (wheel slip or blocked?)" % disagree)
                if (abs(err["lateral"]) > float(a["realign_lateral_m"])
                        or abs(err["yaw"]) > math.radians(float(a["realign_yaw_deg"]))):
                    if retries >= int(self.cfg["limits"]["max_retries"]):
                        raise Stop("drifted during insert, retries used")
                    retries += 1
                    self.summary["realigns"] += 1
                    self.log("realign", reason="drift", lateral_cm=round(err["lateral"] * 100, 2),
                             yaw_deg=round(math.degrees(err["yaw"]), 2))
                    self.execute([("straight", -(pose[0] - x0))], float(self.cfg["base"]["align_speed_m_s"]))
                    self.do_align()
                    restart = True
                    break
                if remaining < -float(ins["distance_tol_m"]):
                    raise Stop("over-inserted by %.3f m (vision)" % -remaining)
                self.fix_yaw(err, "insert_%03dcm" % round(traveled * 100))
            if restart:
                continue
            self.summary["inserted"] = {"odom_m": round(traveled, 4), "target_m": round(goal_x - x0, 4),
                                        "front_range": self.summary.get("front_range"),
                                        "depth_m": float(ins["depth_m"]), "gap_m": float(self.teach["gap_m"]),
                                        "last_check": {"lateral_cm": round(err["lateral"] * 100, 2),
                                                       "yaw_deg": round(math.degrees(err["yaw"]), 2)}}
            self.save_state("inserted", traveled=traveled)
            return

    def final_by_range(self, seg):
        """마지막 구간: 전방 초음파 거리로 멈춘다. 반환 = odom 이동 거리.

        목표 읽기 = front_range.stop_cm(현장 실측, 완전 삽입 때 값)이 있으면 그것, 없으면 지금 값 - seg.
        초음파로 잰 이동과 odom 이동이 max_disagree_m 넘게 다르면 정지(다른 면을 보거나 바퀴가 미끄러짐)."""
        fr = self.cfg["front_range"]
        r0, age = self.front.reading()
        if r0 is None or age > float(fr.get("stale_s", 0.5)):
            raise Stop("front ultrasonic has no fresh reading before the final insert")
        r0 /= 100.0
        target = float(fr["stop_cm"]) / 100.0 if fr.get("stop_cm") else r0 - seg
        expect = r0 - target
        # 초음파가 말하는 남은 거리와 카메라·odom 남은 거리가 맞아야 한다(짧게 가고 성공 처리 방지, Codex r2).
        # 2 cm 미만은 초음파가 못 잰다. 주행 중 불일치는 끝난 뒤에만 판정한다(odom 은 helper 가 끝나야 나옴)
        if target < 0.02 or abs(expect - seg) > float(fr.get("max_disagree_m", 0.05)):
            raise Stop("ultrasonic says %.3f m to go (target %.3f m), vision/odom says %.3f m" % (expect, target, seg))
        last = {"cm": None}

        def check():
            value, a = self.front.reading()
            if value is None or a > float(fr.get("stale_s", 0.5)):
                return "fail:front ultrasonic reading lost"
            last["cm"] = value
            return "stop" if value / 100.0 <= target else None

        self.summary["moves"].append(["straight_until_range", round(seg, 4)])
        self.log("move", what="straight_until_range", max_m=round(seg + 0.03, 4), start_m=round(r0, 4),
                 target_m=round(target, 4))
        data = self.base.straight_until(min(expect, seg) + 0.03, float(self.cfg["base"]["insert_speed_m_s"]), check)
        moved = float(data.get("traveled_m", 0.0))
        by_range = r0 - (last["cm"] or r0 * 100) / 100.0
        self.summary["front_range"] = {"start_m": round(r0, 4), "end_m": None if last["cm"] is None
                                       else round(last["cm"] / 100.0, 4), "target_m": round(target, 4),
                                       "odom_m": round(moved, 4), "by_range_m": round(by_range, 4)}
        self.log("front_range", **self.summary["front_range"])
        if abs(by_range - moved) > float(fr.get("max_disagree_m", 0.05)):
            raise Stop("ultrasonic travel %.3f m vs odom %.3f m disagree" % (by_range, moved))
        if last["cm"] is None or last["cm"] / 100.0 > target + 0.01:
            raise Stop("final insert ended before the ultrasonic target (%.3f m left)" %
                       ((last["cm"] or 0) / 100.0 - target))
        return moved

    def do_lift(self):
        self.stage("LIFT")
        self.arm.stow_for_lift()
        state = self.read_state()
        if not self.args.fake and not self.args.dry_run:
            if not state or state.get("stage") != "inserted" or time.time() - state.get("at", 0) > 600:
                raise Stop("lift needs a successful insert within 10 min (state: %s)" % (state or {}).get("stage"))
        res = self.lift.lift_up()
        self.log("lift", **res)
        self.summary["lift"] = res
        if res.get("result") not in ("complete", "height_limit", "dry"):
            raise Stop("lift result %s" % res)
        self.stage("RETREAT")
        self.save_state("lifted")
        self.execute([("straight", -float(self.cfg["retreat"]["distance_m"]))],
                     float(self.cfg["base"]["retreat_speed_m_s"]))
        self.save_state("done")

    # -------------------------------------------------------------- state
    def save_state(self, stage, **extra):
        if self.args.dry_run:
            return
        data = {"stage": stage, "at": time.time(), "run_dir": str(self.run_dir)}
        data.update({k: (list(v) if isinstance(v, tuple) else v) for k, v in extra.items()})
        STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        STATE_FILE.write_text(json.dumps(data, ensure_ascii=False))

    def read_state(self):
        try:
            return json.loads(STATE_FILE.read_text())
        except (OSError, ValueError):
            return None

    # -------------------------------------------------------------- main
    def run(self):
        mode = self.args.mode
        try:
            if mode == "teach":
                self.do_teach()
                return True
            if mode in ("calibrate", "auto", "approach") and not self.args.dry_run:
                self.base.prepare()
            if mode in ("calibrate", "auto", "approach"):
                self.do_calibrate()
                if mode == "calibrate":
                    self.summary["ok"] = True
                    return True
            self.load_teach()
            if mode in ("align", "insert", "lift", "run") and not self.args.dry_run:
                self.base.prepare()
            if mode == "approach":
                # 진행기 pickup 단계: 정렬·삽입까지만. 상승은 진행기의 lift 단계(포크 위치 추적)가 한다
                self.do_align()
                self.do_insert()
                self.arm.stow_for_lift()
                self.summary["ok"] = True
                return True
            if mode == "auto":
                mode = "run"
            if mode in ("align", "run"):
                self.do_align()
            if mode in ("insert", "run"):
                self.do_insert()
            if mode in ("run", "lift"):
                if self.cfg["insert"]["confirm_before_lift"] and not self.args.yes_lift:
                    if not sys.stdin.isatty():
                        raise Stop("inserted; lift needs operator confirmation — 터미널에서 직접 `lift` 실행")
                    answer = input("포크가 제대로 들어갔고 뒤 %.2f m 가 비었나요? (상승 뒤 자동 후진) 상승하려면 yes: "
                                   % float(self.cfg["retreat"]["distance_m"])).strip().lower()
                    if answer != "yes":
                        raise Stop("operator declined lift")
            if mode in ("lift", "run"):
                self.do_lift()
            self.summary["ok"] = True
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
            if mode != "teach" and not self.args.dry_run and getattr(self.base, "ready", True):
                self.base.stop()
            self.cam.status("%s: %s" % (mode, "OK" if self.summary["ok"] else "STOP " + self.summary.get("reason", "")[:40]))
            self.summary["duration_s"] = round(time.monotonic() - self.t0, 1)
            (self.run_dir / "summary.json").write_text(json.dumps(self.summary, indent=1, ensure_ascii=False,
                                                                  default=str))
            write_csv_row(self.summary, self.run_dir)
            self.events.close()


def write_csv_row(s, run_dir):
    path = LOG_ROOT / "runs.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists()
    al = s.get("aligned") or {}
    ins = s.get("inserted") or {}
    row = {"run_dir": Path(run_dir).name, "mode": s["mode"], "dry_run": s["dry_run"], "fake": s["fake"],
           "ok": s["ok"], "reason": s.get("reason", ""), "aligned_lateral_cm": al.get("lateral_cm", ""),
           "aligned_yaw_deg": al.get("yaw_deg", ""), "aligned_along_cm": al.get("along_cm", ""),
           "insert_odom_m": ins.get("odom_m", ""), "insert_target_m": ins.get("target_m", ""),
           "lift": (s.get("lift") or {}).get("result", ""), "looks": s["looks"], "realigns": s["realigns"],
           "duration_s": s.get("duration_s", "")}
    with path.open("a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)


def build(args, cfg, run_dir):
    def logger_holder():
        holder = {}
        return holder, (lambda kind, **f: holder["p"].log(kind, **f))
    holder, log = logger_holder()
    if not getattr(args, "teach", None):
        args.teach = str(default_teach_path(cfg))
    if args.fake:
        from camera_server import CameraService
        world = args.world
        service = CameraService(cfg, world.frame, args.teach, Path(run_dir) / "looks", allow_fault=True)
        threading.Thread(target=service.run_forever, daemon=True).start()

        class LocalCamera:
            def look(self, tag, n=None, timeout=None, target_hint=None):
                return service.look(n, timeout, tag, target_hint)

            def status(self, text):
                service.status_text = text

        camera = LocalCamera()
        camera.service = service
        arm, base, lift = FakeArm(world, log), FakeBase(world, log), FakeLift(world)
    else:
        camera = HttpCamera(int(cfg["camera"]["http_port"]))
        if args.dry_run:
            arm = DryLog(log, "arm") if not args.arm_in_dry_run else RealArm(cfg, log)
            base, lift = DryLog(log, "base"), DryLog(log, "lift")
            base.ready = False
        else:
            arm = RealArm(cfg, log) if cfg["arm"]["enabled"] else DryLog(log, "arm")
            base = RealBase(cfg, log) if args.mode != "teach" else DryLog(log, "base")
            lift = RealLift(cfg, log) if args.mode in ("lift", "run", "auto") else DryLog(log, "lift")
            if args.mode == "teach":
                base.ready = False
    p = Pickup(cfg, args, camera, arm, base, lift, run_dir)
    holder["p"] = p
    fr = cfg.get("front_range") or {}
    if fr.get("enabled") and not args.fake and not args.dry_run and args.mode in ("insert", "run", "auto", "approach"):
        sys.path.insert(0, str(ROOT / "scripts"))
        from field_video.backends import Ultrasonic
        if not fr.get("port"):
            raise SystemExit("front_range.enabled 인데 port 가 비어 있음(별도 보드 by-id 경로)")
        sensor = Ultrasonic(fr["port"], float(fr.get("stale_s", 0.5)), output=lambda line: log("range_out", line=line))
        sensor.open()
        p.front = sensor
    return p


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["status", "look", "teach", "calibrate", "auto", "approach", "align", "insert",
                                     "lift", "run"])
    ap.add_argument("--cal-distance", type=float, default=0.12, help="calibrate: 후진·전진 거리(m)")
    ap.add_argument("--pause-before-final", action="store_true",
                    help="insert: 카메라 없이 가는 마지막 구간 전에 멈춤(줄자 확인 뒤 insert 다시)")
    ap.add_argument("--align-steps", type=int, default=0,
                    help="align: 이만큼 움직이고 다시 본 뒤 멈춤(0 = 끝까지). 첫 실물은 1 로 lateral_check 확인")
    ap.add_argument("--config", default=str(HERE / "config.yaml"))
    ap.add_argument("--target-side", choices=("left", "right"), help="두 팔레트가 보일 때 첫 목표 선택. 이후 이동 피드백으로 같은 목표 유지")
    ap.add_argument("--teach", default=None, help="보정 파일. 기본: 모드별(tape → teach_tape.json, 그 밖 → teach.json)")
    ap.add_argument("--gap", type=float, help="teach: predock 에서 포크 끝 ↔ 포켓 입구 거리(m, 줄자)")
    ap.add_argument("--camera-distance", type=float,
                    help="teach / calibrate(tape): 시작 자세에서 카메라 렌즈 ↔ 받침 앞면(시트) 수평 거리(m, 줄자). 초점 배율 보정")
    ap.add_argument("--dry-run", action="store_true", help="카메라만 실제, 팔·바퀴·지게팔은 기록만")
    ap.add_argument("--arm-in-dry-run", action="store_true", help="dry-run 에서도 팔 보기 자세는 실제로 움직인다")
    ap.add_argument("--arm-view", action="store_true",
                    help="look: 팔만 보기 자세 B 로 옮긴 뒤 본다(바퀴·지게팔 명령 없음). 실측·정지 확인용")
    ap.add_argument("--yes-lift", action="store_true", help="run: 상승 전 사람 확인 생략(시뮬·반복 시험용)")
    ap.add_argument("--fake", action="store_true", help="합성 카메라·가짜 장치(데스크톱 시험)")
    ap.add_argument("--name", default="")
    args = ap.parse_args(argv)
    args.world = None
    return args


def main(argv=None):
    args = parse(argv)
    cfg = yaml.safe_load(Path(args.config).read_text())
    if not args.teach:
        args.teach = str(default_teach_path(cfg))
    if args.mode == "status":
        cam = HttpCamera(int(cfg["camera"]["http_port"]))
        out = {"vision_mode": cfg.get("vision", {}).get("mode"), "camera": cam.state(), "teach_file": args.teach,
               "teach": Path(args.teach).exists(), "state": None}
        if out["vision_mode"] == "tape" and out["teach"]:
            from tape_vision import teach_problem
            try:
                out["teach_problem"] = teach_problem(json.loads(Path(args.teach).read_text()), cfg)
            except ValueError as exc:
                out["teach_problem"] = "unreadable: %s" % exc
        try:
            out["state"] = json.loads(STATE_FILE.read_text())
        except (OSError, ValueError):
            pass
        for dev in ("/dev/arm_servo", cfg["lift"]["port"], "/dev/video%s" % cfg["camera"]["device"]):
            out[dev] = os.path.exists(dev)
        print(json.dumps(out, indent=1, ensure_ascii=False))
        return 0
    if args.mode == "look":
        # 정지 확인: 바퀴·지게팔 명령 없음(--arm-view 면 팔만 보기 자세 B 로). 카메라 새 프레임 중앙값 → 줄자 실측과 대조할 값
        if args.arm_view:
            try:
                RealArm(cfg, lambda ev, **kw: print(json.dumps(dict(kw, event=ev), ensure_ascii=False),
                                                    flush=True)).view()
            except Stop as exc:
                print("arm: %s" % exc, flush=True)
                return 1
        cam = HttpCamera(int(cfg["camera"]["http_port"]))
        r = cam.look(args.name or "look_cli")
        c = r.get("cam") or {}
        out = {"ok": r.get("ok"), "reason": r.get("reason"), "mode": r.get("mode"), "frames": "%s/%s" % (
            r.get("good"), r.get("frames_seen")), "image": r.get("image")}
        if c:
            out["camera_to_face_m"] = round(-c["x"], 4)
            out["camera_left_of_centre_m"] = round(c["y"], 4)
            out["camera_heading_deg"] = round(math.degrees(c["heading"]), 2)
            out["pitch_down_deg"] = round(float(r.get("pitch_down_deg") or 0), 2)
            out["spread"] = r.get("spread")
            out["tape_centres_px"] = r.get("tape_centres_px")
            out["yaw_sd_deg"] = r.get("heading_sigma_deg")
            out["yaw_per_roll"] = r.get("heading_per_roll")
            teach = None
            if Path(args.teach).exists():
                teach = json.loads(Path(args.teach).read_text())
                if cfg.get("vision", {}).get("mode") == "tape":
                    from tape_vision import teach_problem
                    if teach_problem(teach, cfg):
                        teach = None
            if teach:
                mount = {k: teach["mount"][k] for k in ("forward", "left", "yaw")}
                pose = robot_from_camera(scaled(c, teach), mount)
                err = fork_errors(pose, (teach["predock_x"], teach["target_y"], 0.0))
                out["robot_centre_to_face_m"] = round(-pose[0], 4)
                out["fork_lateral_cm"] = round(err["lateral"] * 100, 2)
                out["robot_yaw_deg"] = round(math.degrees(err["yaw"]), 2)
        print(json.dumps(out, indent=1, ensure_ascii=False))
        return 0 if r.get("ok") else 1
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = LOG_ROOT / ("%s_%s%s%s" % (stamp, args.mode, "_dry" if args.dry_run else "",
                                          ("_" + args.name) if args.name else ""))
    p = build(args, cfg, run_dir)
    ok = p.run()
    print("RESULT %s %s -> %s" % ("OK" if ok else "STOP", p.summary.get("reason", ""), run_dir), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
