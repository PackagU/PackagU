#!/usr/bin/env python3
"""직진 전용 집기(straight_pickup.py) 오프라인 시험 — 실제 장치 없이 합성 영상·가짜 바퀴·가짜 지게팔 줄로 판단만 본다.

    python3 apps/pickup_real/test_straight_pickup.py
합성 카메라는 roll 4.4°(10/31 실제 정면 배치에서 역산한 값)로 그려, 설정 roll 0° 모델이 yaw 를 잘못 보는 상황을 재현한다.
"""
import copy
import math
from pathlib import Path
import tempfile
import time
import unittest

import cv2
import numpy as np
import yaml

import parcel_target
import straight_geometry as sg
import straight_pickup as sp
import tape_synth
import tape_vision
from autonomy import select_target, tape_hypotheses

HERE = Path(__file__).parent
CFG = yaml.safe_load((HERE / "config.yaml").read_text())
TEACH = dict(tape_vision.teach_fields(CFG), mount={"forward": 0.08, "left": -0.11, "yaw": 0.0},
             predock_x=-0.60, target_y=0.0, gap_m=0.30, focal_scale=1.0, position_scale=1.0)
REAL_FRAME = HERE / "testdata" / "two_pallets_parcel_left_20261031.jpg"
REAL_CENTRES = {"left": [108.47, 83.89], "right": [350.17, 65.24]}   # 같은 프레임 PNG 원본의 표식 중심(10/31 22:21)


class World:
    """정답 자세(포켓 좌표: 앞면 원점, x 안쪽, y 왼쪽). 제어기는 이 값을 읽지 않고 렌더링 영상·odom 보고만 받는다."""

    def __init__(self, robot=(-1.266, 0.0, 0.0), roll=4.4, pitch=34.2, true_per_odom=1.0, yaw_per_m=0.0,
                 fail_move=None):
        self.robot = list(robot)
        self.roll, self.pitch = roll, math.radians(pitch)
        self.true_per_odom, self.yaw_per_m = true_per_odom, yaw_per_m
        self.fail_move = fail_move
        self.est = tape_vision.TapeEstimator(CFG, 1.0)
        self.tex, self.origin = tape_synth.face_texture(spacing=0.34, seed=4)
        self.lifted = False
        self.moves = []
        self.turn_scale = 1.05      # 10/31 실기 소각 과회전과 같은 방향

    def frame(self):
        x, y, h = self.robot
        cam = (x + .08 * math.cos(h) + .11 * math.sin(h), y + .08 * math.sin(h) - .11 * math.cos(h), h,
               self.est.dh, self.pitch)
        tilt = getattr(self, "tilt", 0.0)
        if 0 < tilt < 1:
            cam = cam[:4] + (self.pitch - math.radians(25.0) * tilt,)   # 팔을 정면 쪽으로 기울이면 덜 내려다본다
        img = tape_synth.render(self.tex, self.origin, self.est.K, cam, roll_deg=self.roll)
        if self.lifted:
            cv2.rectangle(img, (0, 0), (639, 200), (40, 90, 160), -1)   # 정면 자세에서 올라온 상자
        return img


class FakeCamera:
    def __init__(self, world):
        self.w = world
        self.hints = []

    def look(self, tag, target_hint=None):
        self.hints.append(target_hint)
        if (target_hint and target_hint.get("parcel") and getattr(self.w, "parcel_hidden_below", None) is not None
                and getattr(self.w, "tilt", 0.0) < self.w.parcel_hidden_below):
            return {"ok": False, "reason": "target: no parcel above any pallet (best 0.000, 0 px)"}
        est = self.w.est.estimate(self.w.frame())
        if not est.get("ok"):
            return {"ok": False, "reason": est.get("reason")}
        t = est["tape"]
        return {"ok": True, "tape_centres_px": {"left": t["left"], "right": t["right"]}, "cam": est["cam"],
                "pitch_down_deg": math.degrees(est["cam"]["pitch_down"]), "good": 12, "frames_seen": 12,
                "target_evidence": {"parcel_frac": 0.25} if target_hint and target_hint.get("parcel") else None}

    def state(self):
        return {"fps": 9.0, "age_s": 0.05, "camera_error": None}

    def status(self, text):
        pass

    def frame(self):
        return self.w.frame()

    def labels(self, n=8, timeout=6.0, want=None, tag="labels"):
        return {"ok": True, "frames_seen": n, "labels": list(getattr(self.w, "labels", []))}


class FakeBase:
    def __init__(self, world, probe=None):
        self.w = world
        self.ready = False
        self.prepared = 0
        self._probe = probe or {"ok": True, "scan_hz": 7.6, "scan_age_s": 0.05, "odom_hz": 50.0, "odom_age_s": 0.01,
                                "drive_ready": True, "gate_ready": False, "software_stop": True, "cmd_vel_publishers": [],
                                "rear": {"nearest_m": 3.7, "points": 40}}

    def probe(self):
        return dict(self._probe)

    def prepare(self):
        self.prepared += 1
        self.ready = True

    def straight(self, d, speed, max_yaw_deg, max_lat):
        assert self.ready, "moved before gate prepare"
        self.w.moves.append(d)
        if self.w.fail_move is not None and len(self.w.moves) == self.w.fail_move:
            return {"ok": False, "reason": "yaw changed 1.30 deg (> 1.00) while driving straight", "forward_m": d * 0.4,
                    "lateral_m": 0.0, "yaw_change_deg": 1.3}
        dyaw = self.w.yaw_per_m * abs(d)
        x, y, h = self.w.robot
        true = d * self.w.true_per_odom
        hm = h + dyaw / 2
        self.w.robot = [x + true * math.cos(hm), y + true * math.sin(hm), h + dyaw]
        return {"ok": True, "forward_m": d, "lateral_m": 0.0, "yaw_change_deg": math.degrees(dyaw)}

    def turn(self, rad):
        assert self.ready, "turned before gate prepare"
        true = rad * self.w.turn_scale
        x, y, h = self.w.robot
        self.w.robot = [x, y, h + true]
        self.w.moves.append(("turn", rad))
        return {"ok": True, "turned_deg": math.degrees(true)}

    def stop(self):
        self.ready = False


class FakeArm:
    def __init__(self, world=None):
        self.poses = []
        self.w = world

    def _set(self, name, frac):
        self.poses.append(name)
        if self.w is not None:
            self.w.tilt = frac

    def view(self):
        self._set("view", 0.0)

    def front(self):
        self._set("front", 1.0)

    def tilt(self, frac):
        self._set("tilt%+.2f" % frac, frac)


BANNER = ["Motor ready", "u: Up (ultrasonic stop)", "d: Down (no ultrasonic stop)",
          "Sensor: Up only; No echo does not stop Up", "Up stop distance: 10.0 cm"]


class FakeLift:
    """u 마다 펌웨어 후보와 같은 줄을 시각과 함께 만든다. reach_at = (몇 번째 u, 몇 번째 이동 중 확인)에서 상자 도달."""

    def __init__(self, world, reach_at=(3, 7), pre_cm=150.0, banner=BANNER, reboot_at=None, echo_every=1):
        self.w, self.reach_at, self.pre_cm, self.banner = world, reach_at, pre_cm, banner
        self.reboot_at, self.echo_every = reboot_at, echo_every
        self.k = 0
        self.closed = False

    def open(self):
        return list(self.banner)

    def up_once(self, timeout):
        self.k += 1
        t = time.monotonic()
        lines = [(t, "Up start")]
        tp = 0.00033
        tt = t + 0.10
        if self.k == 1 and self.pre_cm < 10:
            lines += [(tt, "Echo: 500 us | Distance: %.2f cm" % self.pre_cm),
                      (tt + 0.001, "Up stopped: distance below stop threshold")]
            return {"sent": t, "result": "height_limit", "error": None, "lines": lines}
        lines.append((tt, "Echo: 8700 us | Distance: %.2f cm" % self.pre_cm))
        if self.reboot_at == self.k:
            lines += [(tt + 0.5, "Echo: 1" + " " * 40 + "Motor ready"), (tt + 0.501, "u: Up (ultrasonic stop)")]
            return {"sent": t, "result": "error", "error": "lift up end line not seen within 15 s", "lines": lines}
        for j in range(1, 40):
            tt += 320 * tp
            reach = self.reach_at is not None and self.k == self.reach_at[0] and j == self.reach_at[1]
            if reach:
                self.w.lifted = True
                lines += [(tt, "Echo: 480 us | Distance: 8.30 cm"), (tt + 0.001, "Up stopped: distance below stop threshold")]
                return {"sent": t, "result": "height_limit", "error": None, "lines": lines}
            if j % self.echo_every == 0:
                lines.append((tt, "Echo: 8700 us | Distance: %.2f cm" % self.pre_cm))
        lines.append((t + 0.10 + 12800 * tp, "Up complete"))
        return {"sent": t, "result": "complete", "error": None, "lines": lines}

    def close(self):
        self.closed = True


class Args:
    def __init__(self, **kw):
        self.target_label = kw.get("target_label")
        self.plan_only = kw.get("plan_only", False)
        self.no_lift = kw.get("no_lift", False)
        self.scan_label = kw.get("scan_label", False)
        self.align = kw.get("align", False)
        self.roll_deg = kw.get("roll_deg")
        self.roll_from_reference = False
        self.fake = True


def run(world, lift=None, base=None, args=None, cfg=None):
    cam, arm = FakeCamera(world), FakeArm(world)
    base = base or FakeBase(world)
    lift = lift if lift is not None else FakeLift(world)
    est = tape_vision.TapeEstimator(CFG, 1.0)
    d = tempfile.mkdtemp(prefix="straight_test_")
    p = sp.StraightPickup(cfg or CFG, args or Args(), est, TEACH, cam, arm, base, lift, d)
    ok = p.run()
    return ok, p, cam, arm, base


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.est = tape_vision.TapeEstimator(CFG, 1.03)
        self.mount = TEACH["mount"]

    def test_real_frame_lateral_is_roll_independent_and_roll_is_recovered(self):
        lats = [sg.markers_in_robot(self.est, REAL_CENTRES, math.radians(r), self.mount)[0]["lateral"] for r in (0, 2, 4, 5)]
        self.assertLess(max(lats) - min(lats), 0.02)
        roll, why = sg.roll_for_square_face(self.est, REAL_CENTRES, self.mount)
        self.assertIsNone(why)
        self.assertAlmostEqual(math.degrees(roll), 4.4, delta=0.3)
        m, _ = sg.markers_in_robot(self.est, REAL_CENTRES, roll, self.mount)
        self.assertLess(abs(m["lateral"]), 0.01)
        self.assertAlmostEqual(m["forward"] - 0.30, 0.966, delta=0.01)

    def test_projection_round_trip(self):
        roll = math.radians(4.4)
        m, _ = sg.markers_in_robot(self.est, REAL_CENTRES, roll, self.mount)
        for key, p in (("left", m["L"]), ("right", m["R"])):
            u, v = sg.project_robot_point(self.est, p, m["pitch"], roll, self.mount)
            self.assertAlmostEqual(u, REAL_CENTRES[key][0], delta=0.05)
            self.assertAlmostEqual(v, REAL_CENTRES[key][1], delta=0.05)

    def test_synthetic_rolled_camera_reference(self):
        w = World(robot=(-1.266, 0.0, 0.0), roll=4.4)
        est = tape_vision.TapeEstimator(CFG, 1.0)
        r = est.estimate(w.frame())
        self.assertTrue(r["ok"], r.get("reason"))
        self.assertGreater(abs(math.degrees(r["cam"]["heading"])), 6)   # roll 0 모델은 각도를 크게 틀리게 본다
        c = {"left": r["tape"]["left"], "right": r["tape"]["right"]}
        roll, _ = sg.roll_for_square_face(est, c, self.mount)
        self.assertAlmostEqual(math.degrees(roll), 4.4, delta=0.4)


class ParcelTargetTests(unittest.TestCase):
    def test_real_frame_picks_pallet_with_parcel(self):
        img = cv2.imread(str(REAL_FRAME))
        self.assertIsNotNone(img)
        est = tape_vision.TapeEstimator(CFG, 1.03)
        r = est.estimate(img)
        r["stamp"] = 0.0
        self.assertIn("ambiguous", r.get("reason", ""))
        pairs = [(h["tape"]["left"], h["tape"]["right"]) for h in tape_hypotheses(est, r)]
        self.assertEqual(len(pairs), 2)
        r["_scene"] = parcel_target.scene_evidence(img, pairs, want_labels=True)
        s = select_target(est, r, {"parcel": True}, TEACH)
        self.assertTrue(s["ok"], s.get("reason"))
        self.assertLess(abs(s["tape"]["left"][0] - 108.5), 2)
        self.assertGreater(s["target_evidence"]["parcel_frac"], 0.1)
        none = select_target(est, r, {"label": "402"}, TEACH)
        self.assertFalse(none["ok"])
        self.assertIn("402", none["reason"])

    def test_real_frame_choose_by_bearing(self):
        img = cv2.imread(str(REAL_FRAME))
        est = tape_vision.TapeEstimator(CFG, 1.03)
        r = est.estimate(img)
        r["stamp"] = 0.0
        hyps = tape_hypotheses(est, r)
        b = [parcel_target.hypothesis_bearing_deg(est, h) for h in hyps]
        left = min(range(2), key=lambda i: hyps[i]["tape"]["left"][0])
        s = select_target(est, r, {"bearing_deg": b[left] + 2.0}, TEACH)
        self.assertTrue(s["ok"], s.get("reason"))
        self.assertAlmostEqual(s["tape"]["left"][0], 108.5, delta=2)
        far = select_target(est, r, {"bearing_deg": 40.0}, TEACH)
        self.assertFalse(far["ok"])

    def test_label_reading_and_choice(self):
        img = np.full((480, 640, 3), (190, 195, 200), np.uint8)
        for x0, text in ((60, "402"), (380, "310")):
            cv2.rectangle(img, (x0, 40), (x0 + 200, 230), (60, 110, 170), -1)   # 골판지
            cv2.rectangle(img, (x0 + 25, 80), (x0 + 175, 177), (250, 250, 250), -1)   # 흰 라벨 28:18
            for i, ch in enumerate(text):
                gx = x0 + 25 + 18 + i * 42
                for (u, v, du, dv) in label_strokes(ch):
                    cv2.rectangle(img, (int(gx + 15 + (u - du / 2) * 40), int(128 - (v + dv / 2) * 40)),
                                  (int(gx + 15 + (u + du / 2) * 40), int(128 - (v - dv / 2) * 40)), (20, 20, 20), -1)
        labels = parcel_target.read_labels(img)
        texts = sorted(l["text"] for l in labels if l.get("text"))
        self.assertEqual(texts, ["310", "402"], labels)
        ev = parcel_target.scene_evidence(img, [((40, 260), (280, 260)), ((360, 260), (600, 260))], want_labels=True)
        hyps = [{"tape": {"left": [40.0, 260.0], "right": [280.0, 260.0]}, "ok": True},
                {"tape": {"left": [360.0, 260.0], "right": [600.0, 260.0]}, "ok": True}]
        chosen, why, _ = parcel_target.choose(hyps, ev, {"label": "310"})
        self.assertIsNone(why)
        self.assertEqual(chosen["tape"]["left"][0], 360.0)
        self.assertEqual(parcel_target.label_conflict(hyps[0], ev, "310"), "402")


def label_strokes(ch):
    """계획서 라벨 규격의 블록 숫자(글자 7 × 12 cm, 획 1.8 cm)를 0.1 m = 1 단위로."""
    w, h, t = 0.7, 1.2, 0.18
    seg = {'a': (0, h / 2 - t / 2, w, t), 'd': (0, -h / 2 + t / 2, w, t), 'g': (0, 0, w, t),
           'f': (-w / 2 + t / 2, h / 4, t, h / 2), 'b': (w / 2 - t / 2, h / 4, t, h / 2),
           'e': (-w / 2 + t / 2, -h / 4, t, h / 2), 'c': (w / 2 - t / 2, -h / 4, t, h / 2)}
    return [seg[s] for s in parcel_target.DIGIT_SEGMENTS[ch]]


class RunnerTests(unittest.TestCase):
    def test_full_straight_insert_lift_retreat(self):
        w = World(true_per_odom=0.97)
        ok, p, cam, arm, base = run(w)
        self.assertTrue(ok, p.summary.get("reason"))
        fwd = [d for d in w.moves if d > 0]
        self.assertEqual(w.moves[-1], -CFG["retreat"]["distance_m"])
        self.assertTrue(all(d > 0 for d in w.moves[:-1]), w.moves)         # 삽입까지 후진 없음
        self.assertGreaterEqual(len(fwd), 2)                                # 영상 구간 + 마지막 구간
        depth = 0.30 + w.robot[0] + CFG["retreat"]["distance_m"] * 0.97   # 삽입 끝 포크 끝 x(앞면 = 0)
        self.assertAlmostEqual(depth, CFG["straight"]["insert_depth_m"], delta=0.012)
        self.assertEqual(arm.poses, ["view", "view", "front"])
        self.assertTrue(cam.hints[0].get("parcel"))
        self.assertTrue(all("expected_robot" in h for h in cam.hints[2:]))
        ev = p.summary["lift"]["evidence"]
        self.assertEqual(ev["up_commands"], 3)
        self.assertEqual(ev["pulses_above_entry"], 2 * 12800 + 7 * 320)
        self.assertGreater(ev["camera_change"], 4.0)
        self.assertEqual(p.summary["carry_height"]["lower_plan"]["ok"], False)   # 2240 펄스 나머지는 d 로 못 맞춤
        self.assertIn("lifted_verified", p.summary["states"])
        self.assertGreaterEqual(p.summary["recording"]["frames"], 1)
        self.assertTrue((Path(p.run_dir) / "frames" / "index.csv").exists())

    def test_replay_of_20261031_night_run_with_deeper_insert(self):
        # 실기 조건 재현: 기준 robot→앞면 1.266 m·roll 4.41°, 영상/odom 1.058(바퀴 odom 이 약 5 % 짧게 봄), 작은 yaw 흔들림
        w = World(robot=(-1.266, -0.0006, 0.0), roll=4.41, pitch=34.2, true_per_odom=1.058, yaw_per_m=math.radians(0.2))
        ok, p, *_ = run(w)
        self.assertTrue(ok, p.summary.get("reason"))
        depth = 0.30 + w.robot[0] + CFG["retreat"]["distance_m"] * 1.058
        self.assertAlmostEqual(depth, 0.20, delta=0.015)
        self.assertAlmostEqual(p.summary["inserted"]["vision_over_odom"], 1.058, delta=0.02)

    def test_align_to_offset_pallet_then_straight_insert(self):
        # 숫자로 고른 팔레트가 정면이 아님: 왼쪽 0.18 m·yaw 6° 에서 정렬(회전 과회전 5 %) 뒤 직진 삽입
        w = World(robot=(-1.30, 0.18, math.radians(6)), roll=4.4)
        ok, p, *_ = run(w, args=Args(align=True, roll_deg=4.4))
        self.assertTrue(ok, p.summary.get("reason"))
        self.assertIn("aligned", p.summary)
        turns = [i for i, m in enumerate(w.moves) if isinstance(m, tuple)]
        straight_after = [m for m in w.moves[max(turns) + 1:]]
        self.assertTrue(all(not isinstance(m, tuple) for m in straight_after))   # 마지막 회전 뒤로는 직진만
        depth = 0.30 + w.robot[0] + CFG["retreat"]["distance_m"] * math.cos(w.robot[2])
        self.assertAlmostEqual(depth, 0.20, delta=0.02)
        self.assertLess(abs(w.robot[1]), 0.015)

    def test_scan_label_found_and_missing(self):
        w = World(roll=4.4)
        w.labels = [{"text": "402", "frames": 7, "u": 230.0, "v": 300.0}, {"text": "310", "frames": 6, "u": 520.0, "v": 300.0}]
        ok, p, cam, arm, base = run(w, args=Args(target_label="402", scan_label=True, roll_deg=4.4, no_lift=True))
        self.assertTrue(ok, p.summary.get("reason"))
        self.assertEqual(arm.poses[:2], ["tilt%+.2f" % CFG["straight"]["label_tilts"][0], "view"])   # 첫 기울기에서 읽힘
        self.assertIn("bearing_deg", cam.hints[0])
        self.assertLess(p.bearing, 0)          # 화면 왼쪽(u 230) = 왼쪽 방향
        w2 = World(roll=4.4)
        w2.labels = [{"text": "310", "frames": 8, "u": 300.0, "v": 300.0}]
        ok, p, *_ = run(w2, args=Args(target_label="402", scan_label=True, roll_deg=4.4, no_lift=True))
        self.assertFalse(ok)
        self.assertIn("label 402 not read", p.summary["reason"])
        self.assertEqual(w2.moves, [])

    def test_plan_only_does_not_move(self):
        w = World()
        ok, p, cam, arm, base = run(w, args=Args(plan_only=True), lift=None)
        self.assertTrue(ok, p.summary.get("reason"))
        self.assertEqual(w.moves, [])
        self.assertEqual(base.prepared, 0)
        self.assertGreater(p.summary["plan"]["blind_m"], 0.3)
        self.assertLess(p.summary["plan"]["blind_m"], CFG["straight"]["max_blind_m"])

    def test_parcel_cut_off_in_view_b_is_found_by_tilting(self):
        # 11/1 192호: 팔레트가 화면 맨 위라 상자가 잘림 → 팔을 기울여 찾고 그 방향으로 보기 자세 B 에서 직진
        w = World()
        w.parcel_hidden_below = 0.25
        ok, p, cam, arm, base = run(w)
        self.assertTrue(ok, p.summary.get("reason"))
        self.assertEqual(p.summary["parcel_search"]["tilt"], 0.3)
        self.assertEqual(arm.poses[:4], ["view", "tilt+0.15", "tilt+0.30", "view"])
        # 기울인 자세에서 잰 방향 = 보기 자세 B 에서 같은 팔레트 방향(허용 8도보다 훨씬 작게)
        w2 = World()
        est = w2.est.estimate(w2.frame())
        t = est["tape"]
        b0 = sp.pixel_bearing_deg(w2.est.K, (t["left"][0] + t["right"][0]) / 2, (t["left"][1] + t["right"][1]) / 2,
                                  est["cam"]["pitch_down"])
        self.assertLess(abs(p.summary["parcel_search"]["bearing_deg"] - b0), 1.0)
        self.assertIn({"bearing_deg": p.summary["parcel_search"]["bearing_deg"], "bearing_tol_deg": 8.0}, cam.hints)
        self.assertGreater(len(w.moves), 2)

    def test_parcel_never_found_stops_without_moving(self):
        w = World()
        w.parcel_hidden_below = 2.0
        ok, p, cam, arm, base = run(w)
        self.assertFalse(ok)
        self.assertIn("search: parcel pallet not found", p.summary["reason"])
        self.assertEqual(w.moves, [])
        self.assertEqual(arm.poses[-1], "view")

    def test_off_centre_pallet_stops_before_moving(self):
        w = World(robot=(-1.266, 0.07, 0.0))
        ok, p, *_ = run(w)
        self.assertFalse(ok)
        self.assertIn("fork centre line", p.summary["reason"])
        self.assertEqual(w.moves, [])

    def test_vision_odom_disagreement_stops_before_blind(self):
        w = World(true_per_odom=0.6)      # 바퀴가 미끄러지거나 팔레트를 밂
        ok, p, *_ = run(w)
        self.assertFalse(ok)
        self.assertIn("disagree", p.summary["reason"])
        self.assertEqual(len(w.moves), 1)
        self.assertFalse(w.lifted)

    def test_helper_yaw_stop_is_reported(self):
        w = World(fail_move=2)
        ok, p, *_ = run(w)
        self.assertFalse(ok)
        self.assertIn("yaw changed", p.summary["reason"])
        self.assertFalse(w.lifted)

    def test_stale_lidar_blocks_everything(self):
        w = World()
        base = FakeBase(w, probe={"ok": True, "scan_hz": 0.0, "scan_age_s": None, "odom_hz": 50, "odom_age_s": 0.01,
                                  "drive_ready": True, "cmd_vel_publishers": [], "rear": {"nearest_m": 3.0}})
        ok, p, *_ = run(w, base=base)
        self.assertFalse(ok)
        self.assertIn("lidar", p.summary["reason"])
        self.assertEqual(w.moves, [])

    def test_rear_blocked_stops_in_preflight(self):
        w = World()
        probe = FakeBase(w).probe()
        probe["rear"] = {"nearest_m": 0.5, "points": 10}
        ok, p, *_ = run(w, base=FakeBase(w, probe=probe))
        self.assertFalse(ok)
        self.assertIn("rear clearance", p.summary["reason"])

    def test_lift_sensor_blocked_before_lifting_no_retreat(self):
        w = World()
        ok, p, *_ = run(w, lift=FakeLift(w, pre_cm=6.0))
        self.assertFalse(ok)
        self.assertIn("before lifting", p.summary["reason"])
        self.assertTrue(all(d > 0 for d in w.moves))

    def test_lift_reboot_stops(self):
        w = World()
        ok, p, *_ = run(w, lift=FakeLift(w, reboot_at=2))
        self.assertFalse(ok)
        self.assertIn("rebooted", p.summary["reason"])
        self.assertTrue(all(d > 0 for d in w.moves))

    def test_box_never_reaches_sensor_no_retreat(self):
        w = World()
        cfg = copy.deepcopy(CFG)
        cfg["lift_carry"]["max_up_commands"] = 3
        ok, p, *_ = run(w, lift=FakeLift(w, reach_at=None), cfg=cfg)
        self.assertFalse(ok)
        self.assertIn("did not reach", p.summary["reason"])
        self.assertEqual(p.summary["lift"]["evidence"]["up_commands"], 3)
        self.assertTrue(all(d > 0 for d in w.moves))

    def test_banner_mismatch_stops_without_up(self):
        w = World()
        lift = FakeLift(w, banner=["Motor ready", "something else"])
        ok, p, *_ = run(w, lift=lift)
        self.assertFalse(ok)
        self.assertIn("banner", p.summary["reason"])
        self.assertEqual(lift.k, 0)

    def test_missing_echo_lines_use_time(self):
        w = World()
        lift = FakeLift(w, reach_at=(1, 20), echo_every=3)
        rec = sp.parse_up(lift.up_once(15), 10.0, 0.00033)
        self.assertEqual(rec["end"], "height_limit")
        self.assertEqual(rec["pulses"], 20 * 320)
        self.assertIn("time", rec["pulse_source"])

    def test_lower_plan(self):
        self.assertEqual(sp.lower_plan(3 * 12800 + 100), {"d_commands": 3, "residual_pulses": 100, "ok": True})
        self.assertTrue(sp.lower_plan(2 * 12800 - 200)["ok"])
        self.assertFalse(sp.lower_plan(12800 + 5000)["ok"])


class RealArmViewsTests(unittest.TestCase):
    """실제 팔 래퍼(RealArmViews)를 가짜 시리얼 드라이버로 본다. FakeArm 은 기울기마다 이름이 달라 아래 버그를 못 잡았다."""

    def test_consecutive_tilts_each_send_a_move(self):
        # 2026-11-03 강의실 1c: 기울기 0.5 다음 0.35·0.2 가 같은 이름 "tilt" 라 RealArm._go 가 건너뛰어 팔이 그대로였다.
        cfg = copy.deepcopy(CFG)
        cfg["arm"]["move_ms"], cfg["arm"]["settle_s"] = 0, 0
        views = sp.RealArmViews(cfg, lambda *a, **k: None)
        sent = []

        class Driver:
            pos = {}

            def send_pose(self, name, ms, poses):
                sent.append(name)
                self.pos = dict(poses[name])

            def read_positions(self):
                return dict(self.pos)

            def stop_all(self):
                pass

            def close(self):
                pass

        views.arm.driver = Driver()
        for frac in (0.5, 0.35, 0.2):
            views.tilt(frac)
        self.assertEqual(sent, ["tilt+0.50", "tilt+0.35", "tilt+0.20"])
        v, f = views.arm.poses["view"], views.arm.poses["front"]
        self.assertEqual(views.arm.driver.pos["002"], int(round(v["002"] + 0.2 * (f["002"] - v["002"]))))
        views.tilt(0.2)   # 같은 기울기 반복은 이미 그 자세라 다시 보내지 않는다
        self.assertEqual(len(sent), 3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
