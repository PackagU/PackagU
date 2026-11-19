#!/usr/bin/env python3
"""택배 집기 카메라 서버 — 팔 카메라를 혼자 열고 받침을 계속 인식, 인식 화면을 웹으로 보여 준다.
인식 방식은 설정 vision.mode: tape(검정 표식 2개, 10/31) / pallet(구멍 줄) / marker(ArUco 시트). 모드끼리 넘어가지 않는다.

    ~/pickup_venv/bin/python apps/pickup_real/camera_server.py            # Jetson 호스트
    → 노트북/데스크톱 브라우저: http://<JETSON>:8093  (인식 화면, 좌우·각도 오차)

로봇·팔·지게팔은 움직이지 않는다. 실행기(pickup_real.py)는 GET /api/look 으로 "요청 이후의 새 프레임" 인식값만
받아 쓴다. 프레임이 안 오거나(유실) 오래됐으면(정지 영상) look 이 실패하고 실행기는 멈춘다.

HTTP
  /                 실시간 영상·인식 선택      /stream.mjpg   주석 영상(MJPEG)      /live.mjpg  실시간 영상
  /frame.jpg        최신 주석 한 장 (?view=live 로 실시간 원본)
  /api/state        최신 인식값·fps·나이     /api/look?n=6&timeout=6&tag=xx         새 프레임 n 개 중앙값
  /api/status?text= 화면 상단 단계 문구      /api/fault?kind=image_loss|image_stale|none (--allow-fault 일 때만)
  /api/labels?n=6&timeout=6&want=402        새 프레임에서 송장 숫자 판독(숫자별 읽힌 장 수·위치), 화면에 표시
"""
import argparse
import collections
import json
import math
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from geometry import fork_errors, robot_from_camera  # noqa: E402
from vision import MarkerEstimator  # noqa: E402


def aggregate(good, c):
    """모은 인식값 → 중앙값. 중앙값에서 허용 흩어짐의 절반 넘게 떨어진 프레임은 버린다(끝 구멍 하나 흔들린 프레임이
    한 장만 있어도 전체가 거부되던 문제, 10/30 실기·10/31 시뮬). 남은 프레임이 3/4 미만이면 흔들림으로 거부.
    검정 표식 모드는 표식 중심 화소도 중앙값에서 max_jump_px 넘게 튄 프레임을 버린다(프레임마다 튀는 인식 거부)."""
    cams = np.array([[e["cam"]["x"], e["cam"]["y"], e["cam"]["heading"]] for e in good])
    half_y = float(c["max_spread_m"]) / 2.0
    half_h = math.radians(float(c["max_spread_deg"])) / 2.0
    med = np.median(cams, axis=0)
    # x(앞뒤, 삽입 거리 계산용)는 거리 때문에 원래 더 흔들려 y 의 두 배까지 허용(Codex r3: x 미검사 지적)
    keep = ((np.abs(cams[:, 1] - med[1]) <= half_y) & (np.abs(cams[:, 2] - med[2]) <= half_h)
            & (np.abs(cams[:, 0] - med[0]) <= 2 * half_y))
    tape = [e.get("tape") for e in good]
    centres = None
    if all(t and t.get("left") and t.get("right") for t in tape):
        px = np.array([t["left"] + t["right"] for t in tape], float)      # (n, 4): uL vL uR vR
        jump = np.abs(px - np.median(px, axis=0)).max(axis=1)
        if c.get("max_jump_px") is not None:
            keep &= jump <= float(c["max_jump_px"])
        centres = (px, jump)
    need = int(math.ceil(0.75 * len(good)))
    used = [e for e, k in zip(good, keep) if k]
    inl = cams[keep] if keep.sum() else cams
    med = np.median(inl, axis=0)
    spread = inl.max(axis=0) - inl.min(axis=0)
    out = {"cam": {"x": float(med[0]), "y": float(med[1]), "heading": float(med[2])},
           "measurement_mode": "single_endpoint+odometry" if any(e.get("measurement_mode") == "single_endpoint+odometry" for e in good)
                               else "two_markers",
           "spread": {"x": float(spread[0]), "y": float(spread[1]), "heading_deg": float(math.degrees(spread[2]))},
           "inliers": int(keep.sum()), "outliers": int(len(good) - keep.sum()),
           # 프레임별 값(진단용): 흔들림이 한두 장 튐인지, 두 값 사이 왕복인지, 계속 흐르는지 본다
           "samples_y_mm": [round(v * 1000, 1) for v in cams[:, 1]],
           "samples_x_mm": [round(v * 1000, 1) for v in cams[:, 0]],
           "samples_heading_deg": [round(math.degrees(v), 2) for v in cams[:, 2]],
           "reason": None}
    src = used or good
    out.update(reprojection_px=float(np.median([e["reprojection_px"] for e in src])),
               pitch_down_deg=float(math.degrees(np.median([e["cam"]["pitch_down"] for e in src]))),
               height_above_sheet=float(np.median([e["cam"]["height_above_sheet"] for e in src])))
    if centres is not None:
        px, jump = centres
        sel = px[keep] if keep.sum() else px
        m = np.median(sel, axis=0)
        # 보정(calibrate)이 초점 배율을 다시 풀 때 쓰는 중앙값 중심. 단위 px, (u, v)
        out["tape_centres_px"] = {"left": [round(float(m[0]), 3), round(float(m[1]), 3)],
                                  "right": [round(float(m[2]), 3), round(float(m[3]), 3)]}
        out["centre_jump_px"] = [round(float(v), 2) for v in jump]
        out["heading_sigma_deg"] = float(np.median([t.get("heading_sigma_deg") or 0.0 for t in tape]))
        out["heading_per_roll"] = float(np.median([t.get("heading_per_roll") or 0.0 for t in tape]))
    if keep.sum() < need:
        full = cams.max(axis=0) - cams.min(axis=0)
        extra = "" if centres is None else " / centre jump max %.1f px" % float(centres[1].max())
        out["reason"] = "unstable: spread y %.1f mm / %.1f deg / x %.1f mm%s (%d/%d frames agree)" % (
            full[1] * 1000, math.degrees(full[2]), full[0] * 1000, extra, int(keep.sum()), len(good))
    return out


def vision_mode(cfg):
    return cfg.get("vision", {}).get("mode", "marker")


def default_teach(cfg):
    """모드별 보정 파일. 검정 표식 모드는 구멍 모드 teach.json 과 섞이지 않게 따로 둔다."""
    if vision_mode(cfg) == "tape":
        from tape_vision import TEACH_FILE
        return HERE / TEACH_FILE
    return HERE / "teach.json"


def look_limits(cfg):
    """한 번 보기 허용 흔들림. 검정 표식 모드는 tape.look 값이 camera 절보다 우선."""
    c = dict(cfg["camera"])
    if vision_mode(cfg) == "tape":
        c.update((cfg.get("tape") or {}).get("look") or {})
    return c


def make_estimator(cfg, scale, root):
    if vision_mode(cfg) == "tape":
        from tape_vision import TapeEstimator   # 구멍 모드로 넘어가지 않는다
        return TapeEstimator(cfg, scale, root)
    if vision_mode(cfg) == "pallet":
        from pallet_vision import PalletEstimator
        est = PalletEstimator(cfg, scale, root)
        if cfg.get("_teach_height"):
            est.height = float(cfg["_teach_height"])
        return est
    return MarkerEstimator(cfg, scale, root)

DEFAULT_TEACH = HERE / "teach.json"


def load_teach(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def teach_target(teach):
    return (teach["predock_x"], teach["target_y"], 0.0)


def teach_mount(teach):
    return {"forward": teach["mount"]["forward"], "left": teach["mount"]["left"], "yaw": teach["mount"]["yaw"]}


class V4L2Source:
    """계속 촬영하고 최신 한 장만 보관한다. 인식 대기 중에도 USB 영상을 읽는다."""
    def __init__(self, device, width, height, preview_fps=15):
        self.device, self.width, self.height = int(device), int(width), int(height)
        self.preview_interval = 1.0 / max(1.0, float(preview_fps))
        self.condition = threading.Condition()
        self._stop = threading.Event()
        self.frame, self.jpeg, self.error = None, None, None
        self.frames, self.jpeg_seq, self._read_seq = 0, 0, 0
        self.stamp, self.capture_stamp, self.jpeg_stamp, self.fps = 0.0, None, None, 0.0
        self._open()
        self.thread = threading.Thread(target=self._capture, daemon=True)
        self.thread.start()

    def _open(self):
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("cannot open /dev/video%s" % self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def _capture(self):
        last, count, encoded, failures = time.monotonic(), 0, 0.0, 0
        try:
            while not self._stop.is_set():
                try:
                    ok, frame = self.cap.read()
                    if not ok:
                        raise RuntimeError("no frame from camera")
                    now = time.monotonic()
                    with self.condition:
                        self.frame, self.capture_stamp, self.error = frame, now, None
                        self.frames += 1
                        self.condition.notify_all()
                    failures = 0
                    count += 1
                    if now - last >= 1.0:
                        self.fps, last, count = count / (now - last), now, 0
                    if now - encoded >= self.preview_interval:
                        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                        if ok:
                            with self.condition:
                                self.jpeg, self.jpeg_stamp = buf.tobytes(), now
                                self.jpeg_seq += 1
                        encoded = now
                except Exception as exc:
                    with self.condition:
                        self.error = "capture: %s: %s" % (type(exc).__name__, exc)
                    failures += 1
                    if failures >= 5:
                        self.cap.release()
                        try:
                            self._open()
                        except Exception as reopen_error:
                            self.error = "camera reconnect: %s" % reopen_error
                        failures = 0
                    self._stop.wait(0.2)
        except Exception as exc:
            self.error = "capture stopped: %s: %s" % (type(exc).__name__, exc)
        finally:
            self.cap.release()

    def __call__(self):
        with self.condition:
            self.condition.wait_for(lambda: self.frames > self._read_seq or self._stop.is_set(), timeout=1.0)
            if self.frames <= self._read_seq or self._stop.is_set():
                return None
            self._read_seq, self.stamp = self.frames, self.capture_stamp
            return self.frame

    def preview(self):
        with self.condition:
            return self.jpeg_seq, self.jpeg, self.jpeg_stamp

    def state(self):
        with self.condition:
            return {"capture_frames": self.frames, "capture_fps": round(self.fps, 1),
                    "capture_age_s": None if self.capture_stamp is None else round(time.monotonic() - self.capture_stamp, 3),
                    "capture_error": self.error}

    def stop(self):
        self._stop.set()
        with self.condition:
            self.condition.notify_all()


class CameraService:
    def __init__(self, cfg, source, teach_path=None, log_dir=None, allow_fault=False):
        self.cfg, self.source = cfg, source
        self.mode = vision_mode(cfg)
        self.teach_path = Path(teach_path) if teach_path else default_teach(cfg)
        self.teach_error = None
        self.log_dir = Path(log_dir) if log_dir else ROOT / "logs" / "pickup_real" / "looks"
        self.allow_fault = allow_fault
        self.lock = threading.Lock()
        self.history = collections.deque(maxlen=120)  # (stamp, est)
        self.jpeg, self.latest, self.status_text = None, None, "idle"
        self.fault, self.frames, self.fps, self.camera_error = None, 0, 0.0, None
        self.teach, self.focal_scale, self.estimator = None, None, None
        self.preview_target_hint = None
        self.scene_hint = None   # 마지막 look 의 택배/숫자 목표 조건(프레임별 근거 계산 여부)
        self.label_scan_until, self.label_want = 0.0, None
        self.label_history = collections.deque(maxlen=60)   # (stamp, [labels])
        self.reload_teach()
        self._stop = threading.Event()

    def scene_wanted(self):
        """프레임별 택배/숫자 근거가 필요하면 숫자 판독 여부(bool), 필요 없으면 None."""
        if self.mode != "tape":
            return None
        hints = [h for h in (self.scene_hint, self.preview_target_hint) if isinstance(h, dict)]
        if not any(h.get("parcel") or h.get("label") for h in hints):
            return None
        return any(bool(h.get("label")) for h in hints)

    def reload_teach(self):
        teach = load_teach(self.teach_path)
        if self.mode == "tape":
            # 검정 표식 모드: 이 모드용으로 만든 보정만 쓴다. 옛 구멍용 길이·극성·높이는 적용하지 않는다
            from tape_vision import teach_problem
            problem = teach_problem(teach, self.cfg) if teach else None
            self.teach_error = problem
            if problem:
                teach = None
            scale = float(teach.get("focal_scale", 1.0)) if teach else 1.0
            if self.estimator is None or scale != self.focal_scale:
                self.estimator = make_estimator(self.cfg, scale, ROOT)
                self.focal_scale = scale
            self.teach = teach
            return
        scale = float(teach.get("focal_scale", 1.0)) if teach else 1.0
        if self.estimator is None or scale != self.focal_scale:
            self.estimator = make_estimator(self.cfg, scale, ROOT)
            self.focal_scale = scale
        before = (getattr(self.estimator, "height", None), (getattr(self.estimator, "p", None) or {}).get("hole_pitch_m"),
                  (getattr(self.estimator, "p", None) or {}).get("row_length_m"), getattr(self.estimator, "polarity", None))
        if teach and teach.get("hole_polarity") in ("dark", "bright") and hasattr(self.estimator, "polarity"):
            self.estimator.polarity = teach["hole_polarity"]
            self.estimator.bright = teach["hole_polarity"] == "bright"
        if teach and teach.get("camera_height_m") and hasattr(self.estimator, "height"):
            self.estimator.height = float(teach["camera_height_m"])
        if teach and teach.get("hole_pitch_m") and hasattr(self.estimator, "p"):
            self.estimator.p = dict(self.estimator.p, hole_pitch_m=float(teach["hole_pitch_m"]))
        if teach and teach.get("row_length_m") and hasattr(self.estimator, "p"):
            self.estimator.p = dict(self.estimator.p, row_length_m=float(teach["row_length_m"]))
        after = (getattr(self.estimator, "height", None), (getattr(self.estimator, "p", None) or {}).get("hole_pitch_m"),
                 (getattr(self.estimator, "p", None) or {}).get("row_length_m"), getattr(self.estimator, "polarity", None))
        if before != after and hasattr(self.estimator, "last"):
            self.estimator.last = None   # 높이·간격이 바뀌면 옛 해에서 시작하지 않는다
        self.teach = teach

    # ---------------------------------------------------------------- capture loop
    def step(self):
        frame = self.source()
        now = getattr(self.source, "stamp", time.monotonic())
        if frame is None:
            self.camera_error = "no frame from camera"
            return False
        if self.fault == "image_loss":  # 시험용: 프레임이 안 온 것과 같게
            return True
        est = self.estimator.estimate(frame)
        est["stamp"] = now
        want = self.scene_wanted()
        if want is not None:
            # 택배/송장 숫자로 목표를 고를 때만: 표식 쌍마다 위쪽 상자 색·라벨 숫자 근거를 프레임에 붙인다
            from autonomy import tape_hypotheses
            from parcel_target import scene_evidence
            pairs = [(h["tape"]["left"], h["tape"]["right"]) for h in tape_hypotheses(self.estimator, est) if h.get("tape")]
            est["_scene"] = scene_evidence(frame, pairs, want_labels=want)
        raw_est = est
        if self.preview_target_hint and self.mode == "tape":
            from autonomy import select_target
            est = select_target(self.estimator, raw_est, self.preview_target_hint, self.teach)
        est["robot"] = None
        if est.get("ok") and self.teach and all(k in self.teach for k in ("predock_x", "target_y", "mount")):
            k = float(self.teach.get("position_scale", 1.0))
            cam = dict(est["cam"], x=est["cam"]["x"] * k, y=est["cam"]["y"] * k)
            pose = robot_from_camera(cam, teach_mount(self.teach))
            est["robot"] = list(pose)
            est["errors"] = fork_errors(pose, teach_target(self.teach))
        lines = [self.status_text[:58]]
        if est.get("measurement_mode") == "single_endpoint+odometry":
            lines.append("partial view: heading from odometry")
        if est.get("ok"):
            c = est["cam"]
            if est.get("errors"):
                e = est["errors"]
                lines.append("fork lateral %+5.1f cm  yaw %+5.1f deg" % (e["lateral"] * 100, math.degrees(e["yaw"])))
                lines.append("to predock %+5.1f cm  (sheet %.2f m)" % (-e["along"] * 100, -c["x"]))
            else:
                lines.append("cam lateral %+5.1f cm  heading %+5.1f deg" % (c["y"] * 100, math.degrees(c["heading"])))
                lines.append("sheet %.2f m  (%s)" % (-c["x"], "teach rejected" if self.teach_error else "no teach yet"))
        if self.mode == "tape":
            # 이 모드는 측정 4개·미지수 4개라 재투영 오차가 항상 0 → 대신 화소 잡음에 대한 yaw 흔들림을 보인다
            sig = (est.get("tape") or {}).get("heading_sigma_deg")
            lines.append("fps %.1f  yaw sd %s" % (self.fps, "%.2f deg" % sig if est.get("ok") and sig is not None else "-"))
        else:
            lines.append("fps %.1f  reproj %s" % (self.fps, "%.2f" % est["reprojection_px"] if est.get("ok") else "-"))
        shown = self.estimator.annotate(frame, est, lines)
        if time.monotonic() < self.label_scan_until:
            # 송장 숫자 찾기(정면 자세): 프레임마다 판독해 모으고 화면에 상자·숫자를 그린다
            from parcel_target import read_labels
            labels = read_labels(frame)
            with self.lock:
                self.label_history.append((now, labels))
            for lb in labels:
                x, y, w, h = lb["bbox"]
                hit = lb.get("text") and lb.get("text") == self.label_want
                colour = (0, 220, 0) if hit else ((0, 200, 255) if lb.get("text") else (0, 0, 255))
                cv2.rectangle(shown, (x, y), (x + w, y + h), colour, 3 if hit else 2)
                cv2.putText(shown, lb.get("text") or "?", (x, max(18, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
        ok, buf = cv2.imencode(".jpg", shown, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with self.lock:
            if self.fault != "image_stale":  # 시험용: 정지 영상(값이 더 이상 새로워지지 않음)
                self.history.append((now, raw_est))
                self.latest = est
            self.jpeg = buf.tobytes() if ok else self.jpeg
            self._frame, self._shown = frame, shown
            self.frames += 1
        self.camera_error = None
        return True

    IDLE_HZ = 2.0   # 요청(look·labels)이 없을 때 인식 주기. 쉬지 않고 돌면 CPU 280 % → ROS odom 끊김(192호 2026-11-01)

    def busy(self, seconds):
        """look·scan_labels 동안은 전속력으로 인식한다."""
        self.busy_until = max(getattr(self, "busy_until", 0.0), time.monotonic() + float(seconds))

    def run_forever(self):
        last, count = time.monotonic(), 0
        while not self._stop.is_set():
            if time.monotonic() > getattr(self, "busy_until", 0.0):
                self._stop.wait(1.0 / self.IDLE_HZ)
            try:
                if self.step():
                    count += 1
                else:
                    self._stop.wait(0.2)
            except Exception as exc:
                self.camera_error = "recognition: %s: %s" % (type(exc).__name__, exc)
                self._stop.wait(0.2)
            now = time.monotonic()
            if now - last >= 1.0:
                self.fps, last, count = count / (now - last), now, 0

    def stop(self):
        self._stop.set()
        if hasattr(self.source, "stop"):
            self.source.stop()

    def preview(self, live=False):
        if live and hasattr(self.source, "preview"):
            return self.source.preview()
        with self.lock:
            return self.frames, self.jpeg, self.latest["stamp"] if self.latest else None

    # ---------------------------------------------------------------- look
    def look(self, n=None, timeout=None, tag="look", target_hint=None):
        """요청 시각 이후의 새 인식값 n 개를 모아 중앙값. 실패하면 ok=False 와 이유."""
        c = look_limits(self.cfg)
        n = int(n or c["min_frames"])
        timeout = float(timeout or c["acquire_timeout_s"])
        self.busy(timeout + 1.0)
        self.reload_teach()
        self.scene_hint = dict(target_hint) if isinstance(target_hint, dict) and (
            target_hint.get("parcel") or target_hint.get("label")) else None
        if not target_hint:
            # 목표 지정 없는 요청(시작 점검의 깨우기 등)이면 이전 look 의 지정을 지운다. 남아 있으면 매 프레임
            # 송장 숫자 판독이 돌아 인식 속도가 절반 아래로 떨어진다(2026-11-03 강의실 1d: 4.5 fps 로 시작 점검 정지).
            self.preview_target_hint = None
        since = time.monotonic()
        deadline = since + timeout
        good, seen, reasons = [], 0, collections.Counter()
        selected_cache = {}
        while time.monotonic() < deadline:
            with self.lock:
                fresh = [(s, e) for s, e in self.history if s > since]
            if target_hint and self.mode == "tape":
                from autonomy import select_target
                for s, e in fresh:
                    if s not in selected_cache:
                        selected_cache[s] = select_target(self.estimator, e, target_hint, self.teach)
                fresh = [(s, selected_cache[s]) for s, _ in fresh]
            seen = len(fresh)
            good = [e for s, e in fresh if e.get("ok")]
            reasons = collections.Counter(e.get("reason", "?") for s, e in fresh if not e.get("ok"))
            if len(good) >= n:
                break
            time.sleep(0.03)
        now = time.monotonic()
        result = {"tag": tag, "ok": False, "frames_seen": seen, "good": len(good), "elapsed_s": round(now - since, 2),
                  "reasons": dict(reasons), "fault": self.fault, "camera_error": self.camera_error,
                  "focal_scale": self.focal_scale, "k_source": self.estimator.k_source, "mode": self.mode,
                  "teach_error": self.teach_error}
        if target_hint and self.mode == "tape" and seen:
            summaries = [e.get("target_summary") for s, e in fresh if e.get("target_summary")]
            evidence = [e.get("target_evidence") for e in good if e.get("target_evidence")]
            if summaries:
                result["target_summary"] = summaries[-1]
            if evidence:
                result["target_evidence"] = evidence[-1]
        if seen == 0:
            result["reason"] = "image loss or frozen: no new frame in %.1f s" % timeout
        elif len(good) < n:
            top = reasons.most_common(1)
            result["reason"] = "markers not seen in %d/%d new frames%s" % (
                seen - len(good), seen, " (mostly: %s)" % top[0][0] if top else "")
            if self.mode == "tape":
                from autonomy import recovery_hint
                result["recovery"] = recovery_hint(fresh, n, now, float(c["max_age_s"]))
        else:
            good = good[-n:]
            age = now - good[-1]["stamp"]
            agg = aggregate(good, c)
            problem = agg.pop("reason")
            result.update(agg, age_s=round(age, 3))
            if age > float(c["max_age_s"]):
                result["reason"] = "stale: newest sample %.2f s old" % age
            elif problem:
                result["reason"] = problem
            else:
                result["ok"] = True
        if result.get("ok") and target_hint:
            self.preview_target_hint = dict(target_hint)
        self._save(tag, result)
        return result

    def scan_labels(self, n=6, timeout=6.0, tag="labels", want=None):
        """지금부터 새 프레임 n 장에서 송장 숫자를 읽어 모은다. 숫자별 읽힌 장 수·화소 위치 중앙값."""
        self.busy(float(timeout or 6.0) + 1.0)
        n, timeout = int(n or 6), float(timeout or 6.0)
        since = time.monotonic()
        self.label_want = str(want) if want else None
        self.label_scan_until = since + timeout + 3.0   # 결과를 본 뒤에도 잠시 화면에 표시
        deadline = since + timeout
        fresh = []
        while time.monotonic() < deadline:
            with self.lock:
                fresh = [(s, l) for s, l in self.label_history if s > since]
            if len(fresh) >= n:
                break
            time.sleep(0.05)
        texts = {}
        for s, labels in fresh:
            for lb in labels:
                if lb.get("text"):
                    texts.setdefault(lb["text"], []).append((lb["u"], lb["v"], lb["bbox"]))
        out = []
        for t, rows in sorted(texts.items(), key=lambda kv: -len(kv[1])):
            out.append({"text": t, "frames": len(rows), "u": float(np.median([r[0] for r in rows])),
                        "v": float(np.median([r[1] for r in rows])), "bbox": rows[-1][2]})
        result = {"tag": tag, "ok": bool(fresh), "frames_seen": len(fresh), "labels": out, "want": self.label_want,
                  "unread": sum(1 for s, l in fresh for lb in l if not lb.get("text")),
                  "reason": None if fresh else "image loss: no new frame in %.1f s" % timeout,
                  "width": int(self.cfg["camera"]["width"]), "fx": float(self.estimator.K[0, 0]),
                  "cx": float(self.estimator.K[0, 2])}
        self._save(tag, result)
        return result

    def _save(self, tag, result):
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            stem = "%s_%s" % (time.strftime("%H%M%S"), "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in tag))
            with self.lock:
                frame, shown = getattr(self, "_frame", None), getattr(self, "_shown", None)
            if shown is not None:
                cv2.imwrite(str(self.log_dir / (stem + ".jpg")), shown)
                cv2.imwrite(str(self.log_dir / (stem + "_raw.png")), frame)
            (self.log_dir / (stem + ".json")).write_text(json.dumps(result, indent=1, ensure_ascii=False))
            result["image"] = str(self.log_dir / (stem + ".jpg"))
        except OSError as exc:
            result["save_error"] = str(exc)

    def state(self):
        with self.lock:
            est = self.latest
        out = {"status": self.status_text, "fps": round(self.fps, 1), "frames": self.frames, "fault": self.fault,
               "camera_error": self.camera_error, "teach": bool(self.teach), "k_source": self.estimator.k_source,
               "mode": self.mode, "teach_file": self.teach_path.name, "teach_error": self.teach_error}
        if est:
            out["age_s"] = round(time.monotonic() - est["stamp"], 3)
            out["ok"] = bool(est.get("ok")) and out["age_s"] <= float(self.cfg["camera"]["max_age_s"]) and not self.camera_error
            out["ids"] = est.get("ids")
            out["reason"] = est.get("reason")
            if est.get("tape"):
                out["tape"] = est["tape"]
            if est.get("ok"):
                out["cam"] = est["cam"]
                out["errors"] = est.get("errors")
        if hasattr(self.source, "state"):
            out.update(self.source.state())
        return out


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>택배 집기 인식 화면</title><style>
body{margin:0;background:#111;color:#eee;font:16px sans-serif}main{display:flex;flex-wrap:wrap;gap:16px;padding:16px}
img{max-width:100%;width:800px;border:2px solid #333}#s{min-width:280px}b{font-size:28px}.ok{color:#5f5}.bad{color:#f66}
td{padding:4px 10px}button{padding:8px;margin:4px}#video{max-width:100%}
</style></head><body><main><div id="video"><img id="camera" alt="카메라 연결 중">
<p><button id="view">인식 표시 보기</button> <span id="videoState">연결 중</span></p></div>
<div id="s"><h2>택배 집기 · 팔 카메라</h2>
<table><tr><td>단계</td><td id="st">-</td></tr><tr><td>모드·보정</td><td id="md">-</td></tr><tr><td>인식</td><td id="mk">-</td></tr>
<tr><td>포크 좌우 오차</td><td><b id="lat">-</b> cm</td></tr><tr><td>진입 각도 오차</td><td><b id="yaw">-</b> °</td></tr>
<tr><td>predock 까지</td><td id="al">-</td></tr><tr><td>시트 거리</td><td id="d">-</td></tr>
<tr><td>인식 프레임 나이</td><td id="age">-</td></tr><tr><td>인식 fps</td><td id="fps">-</td></tr>
<tr><td>촬영 fps</td><td id="captureFps">-</td></tr></table>
<p style="color:#999">좌우 + = 포크가 포켓 중심보다 왼쪽. 각도 + = 로봇이 왼쪽으로 틀어짐.<br>teach 전에는 카메라 기준 값.</p></div></main>
<script>
const el=id=>document.getElementById(id);
let live=true,lastSeq=null,lastNew=performance.now(),objectUrl=null;
el('view').onclick=()=>{live=!live;lastSeq=null;lastNew=performance.now();
el('view').textContent=live?'인식 표시 보기':'실시간 영상 보기'};
// 한 번에 한 장만 요청한다. 느린 연결에 영상이 쌓이지 않고, 단절 뒤 다음 요청으로 복구된다.
async function preview(){const start=performance.now(),mode=live,abort=new AbortController();
const timer=setTimeout(()=>abort.abort(),1800);let delay=65;
try{const r=await fetch('/frame.jpg?view='+(mode?'live':'annotated'),{cache:'no-store',signal:abort.signal});
if(!r.ok)throw Error('영상 대기');const blob=await r.blob();
if(mode!==live)return;
const seq=r.headers.get('X-Frame-Sequence'),age=Number(r.headers.get('X-Frame-Age'));
if(seq!==lastSeq){lastSeq=seq;lastNew=performance.now();const url=URL.createObjectURL(blob);
el('camera').src=url;if(objectUrl)URL.revokeObjectURL(objectUrl);objectUrl=url}
const stale=age>1||performance.now()-lastNew>1500;
el('videoState').textContent=stale?'영상 멈춤 · 재연결 중':(live?'실시간 영상':'인식 표시 · 처리 속도로 갱신');
el('videoState').className=stale?'bad':'ok';el('camera').style.opacity=stale?'0.4':'1';
}catch(e){el('videoState').textContent='영상 연결 끊김 · 재시도 중';el('videoState').className='bad';
el('camera').style.opacity='0.4';delay=500}
finally{clearTimeout(timer);setTimeout(preview,Math.max(0,delay-(performance.now()-start)))}}
async function tick(){const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),1800);
try{const r=await fetch('/api/state',{cache:'no-store',signal:abort.signal});if(!r.ok)throw Error('state');const s=await r.json();
el('st').textContent=s.status;el('fps').textContent=s.fps;
el('md').textContent=(s.mode||'-')+' · '+(s.teach_file||'')+(s.teach_error?' 거부: '+s.teach_error:(s.teach?' 사용':' 없음'));el('captureFps').textContent=s.capture_fps??s.fps;
el('age').textContent=(s.age_s??'-')+' s';
el('mk').textContent=s.ok?'받침 인식':(s.capture_error||s.camera_error||s.reason||'인식 대기 / 오래된 프레임');
el('mk').className=s.ok?'ok':'bad';
if(s.ok&&s.errors){el('lat').textContent=(s.errors.lateral*100).toFixed(1);el('yaw').textContent=(s.errors.yaw*57.2958).toFixed(1);
el('al').textContent=(-s.errors.along*100).toFixed(1)+' cm'}else if(s.ok){el('lat').textContent=(s.cam.y*100).toFixed(1);
el('yaw').textContent=(s.cam.heading*57.2958).toFixed(1);el('al').textContent='teach 필요'}else{el('lat').textContent='-';el('yaw').textContent='-'}
el('d').textContent=s.ok?(-s.cam.x).toFixed(2)+' m':'-'}catch(e){el('st').textContent='서버 연결 끊김';
for(const id of ['lat','yaw','al','d','age','fps','captureFps'])el(id).textContent='-';el('mk').textContent='인식 상태 확인 불가';el('mk').className='bad'}
finally{clearTimeout(timer);setTimeout(tick,300)}}preview();tick();
</script></body></html>"""


class ThreadingServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def make_handler(service):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, data, code=200):
            body = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urllib.parse.urlparse(self.path)
            q = {k: v[0] for k, v in urllib.parse.parse_qs(url.query).items()}
            if url.path == "/":
                body = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            elif url.path == "/frame.jpg":
                seq, jpeg, stamp = service.preview(q.get("view") == "live")
                if jpeg is None:
                    return self._json({"error": "no frame yet"}, 503)
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Frame-Sequence", str(seq))
                self.send_header("X-Frame-Age", "%.3f" % (time.monotonic() - stamp) if stamp is not None else "999")
                self.end_headers()
                self.wfile.write(jpeg)
            elif url.path in ("/stream.mjpg", "/live.mjpg"):
                self.connection.settimeout(3.0)
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    sent = None
                    while not service._stop.is_set():
                        seq, jpeg, stamp = service.preview(url.path == "/live.mjpg")
                        if stamp is not None and time.monotonic() - stamp > 2.0:
                            return
                        if jpeg is not None and seq != sent:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                            self.wfile.write(jpeg + b"\r\n")
                            self.wfile.flush()
                            sent = seq
                        service._stop.wait(0.02)
                except OSError:
                    return
            elif url.path == "/api/state":
                self._json(service.state())
            elif url.path == "/api/look":
                try:
                    hint = json.loads(q["target_hint"]) if q.get("target_hint") else None
                    if hint:
                        self._json(service.look(q.get("n"), q.get("timeout"), q.get("tag", "look"), hint))
                    else:
                        self._json(service.look(q.get("n"), q.get("timeout"), q.get("tag", "look")))
                except (ValueError, TypeError, KeyError) as exc:
                    self._json({"ok": False, "reason": "invalid target selection: %s" % exc}, 400)
            elif url.path == "/api/labels":
                return self._json(service.scan_labels(q.get("n"), q.get("timeout"), q.get("tag", "labels"), q.get("want")))
            elif url.path == "/api/status":
                service.status_text = q.get("text", "")[:80]
                self._json({"ok": True})
            elif url.path == "/api/fault":
                if not service.allow_fault:
                    return self._json({"ok": False, "reason": "start with --allow-fault"}, 403)
                kind = q.get("kind", "none")
                service.fault = None if kind == "none" else kind
                self._json({"ok": True, "fault": service.fault})
            else:
                self._json({"error": "not found"}, 404)

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(HERE / "config.yaml"))
    ap.add_argument("--teach", default=None, help="기본: 모드별(tape → teach_tape.json, 그 밖 → teach.json)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=0, help="0 이면 설정 camera.http_port")
    ap.add_argument("--log-dir", default=str(ROOT / "logs" / "pickup_real" / "looks"))
    ap.add_argument("--allow-fault", action="store_true", help="/api/fault 로 영상 유실·정지 영상 주입 허용(시험용)")
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    c = cfg["camera"]
    service = CameraService(cfg, V4L2Source(c["device"], c["width"], c["height"], c.get("preview_fps", 15)), args.teach, args.log_dir,
                            args.allow_fault)
    threading.Thread(target=service.run_forever, daemon=True).start()
    port = args.port or int(c["http_port"])
    server = ThreadingServer((args.host, port), make_handler(service))
    print("camera server on :%d  (mode=%s, k=%s, teach=%s %s%s)" % (
        port, service.mode, service.estimator.k_source, service.teach_path.name, bool(service.teach),
        "" if not service.teach_error else " REJECTED: " + service.teach_error), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        service.stop()


if __name__ == "__main__":
    main()
