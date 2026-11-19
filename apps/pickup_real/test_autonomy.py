#!/usr/bin/env python3
"""동적 장치 없이 정적 팔레트 끝점 계산·목표 연속성·자동 시야 복구를 검사."""
import copy
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np
import yaml

import autonomy
import pickup_real
import tape_synth
import tape_vision
from geometry import robot_from_camera

HERE = Path(__file__).parent


class AutonomyTests(unittest.TestCase):
    def setUp(self):
        self.cfg = yaml.safe_load((HERE / "config.yaml").read_text())
        self.est = tape_vision.TapeEstimator(self.cfg)
        self.teach = dict(tape_vision.teach_fields(self.cfg),
                          mount={"forward": 0.08, "left": -0.11, "yaw": 0.0},
                          predock_x=-0.60, target_y=0.0, gap_m=0.30, focal_scale=1.0, position_scale=1.0)
        self.pitch = math.radians(34)
        self.texture, self.origin = tape_synth.face_texture(spacing=0.34, seed=3)

    def render(self, robot, texture=None):
        x, y, h = robot
        mount = self.teach["mount"]
        cam = (x + .08 * math.cos(h) + .11 * math.sin(h),
               y + .08 * math.sin(h) - .11 * math.cos(h), h, self.est.dh, self.pitch)
        frame = tape_synth.render(self.texture if texture is None else texture, self.origin, self.est.K, cam)
        result = self.est.estimate(frame)
        result["stamp"] = time.monotonic()
        return result

    def test_default_still_rejects_two_pallet_candidates(self):
        texture, _ = tape_synth.face_texture(spacing=.34, seed=5,
                                             decoys=[(-.10, -.17, .030, .020, True), (.20, -.17, .030, .020, True)])
        result = self.render((-1.0, .05, 0), texture)
        self.assertFalse(result["ok"])
        self.assertIn("ambiguous", result["reason"])
        selected = autonomy.select_target(self.est, result, {"side": "left"}, self.teach)
        self.assertTrue(selected["ok"], selected.get("reason"))
        pose = robot_from_camera(selected["cam"], self.teach["mount"])
        again = autonomy.select_target(self.est, result, {"expected_robot": pose}, self.teach)
        self.assertTrue(again["ok"])
        np.testing.assert_allclose(again["cam"]["x"], selected["cam"]["x"])

    def test_neighbour_cannot_replace_known_target(self):
        result = self.render((-1.0, .05, 0))
        self.assertTrue(result["ok"], result.get("reason"))
        selected = autonomy.select_target(self.est, result, {"expected_robot": [-1.0, -.4, 0]}, self.teach)
        self.assertFalse(selected["ok"])
        self.assertIn("odometry", selected["reason"])

    def test_corner_shadow_does_not_hide_a_complete_selected_pallet(self):
        frame = tape_synth.render(self.texture, self.origin, self.est.K,
                                  (-.92, -.11, 0, self.est.dh, self.pitch))
        cv2.rectangle(frame, (0, 0), (20, 12), (0, 0, 0), -1)
        result = self.est.estimate(frame)
        result["stamp"] = time.monotonic()
        self.assertTrue(result["ok"])
        self.assertTrue(result["_feats"]["cut"])
        selected = autonomy.select_target(self.est, result, {"side": "left"}, self.teach)
        self.assertTrue(selected["ok"], selected.get("reason"))

    def test_one_visible_endpoint_measures_cm_with_known_pitch_and_heading(self):
        robot = (-.65, .02, 0)
        result = self.render(robot)
        self.assertFalse(result["ok"], "test needs an actually clipped endpoint")
        selected = autonomy.select_target(self.est, result,
                                         {"expected_robot": robot, "allow_partial": True, "pitch_down": self.pitch}, self.teach)
        self.assertTrue(selected["ok"], selected.get("reason"))
        self.assertEqual(selected["measurement_mode"], "single_endpoint+odometry")
        pose = robot_from_camera(selected["cam"], self.teach["mount"])
        self.assertLess(math.hypot(pose[0] - robot[0], pose[1] - robot[1]), .008)

    def test_partial_needs_previously_known_geometry(self):
        result = self.render((-.65, .02, 0))
        selected = autonomy.select_target(self.est, result, {"side": "left"}, self.teach)
        self.assertFalse(selected["ok"])

    def test_middle_occlusion_is_not_replaced_with_estimated_marker(self):
        texture, _ = tape_synth.face_texture(spacing=.34, seed=3, hide=("left",))
        robot = (-1.15, 0, 0)
        result = self.render(robot, texture)
        selected = autonomy.select_target(self.est, result,
                                         {"expected_robot": robot, "allow_partial": True, "pitch_down": self.pitch}, self.teach)
        self.assertFalse(selected["ok"])

    def test_geometry_chooses_farther_view_for_right_offset_camera(self):
        x = autonomy.visible_look_x(self.cfg, self.teach, self.pitch, -.75)
        self.assertLess(x, -.80)
        self.assertGreaterEqual(x, -1.05)
        result = self.render((x, 0, 0))
        self.assertTrue(result["ok"], result.get("reason"))
        self.assertGreater(min(result["tape"][k][0] for k in ("left", "right")), 50)

    def test_no_recovery_hint_from_stale_or_missing_images(self):
        now = time.monotonic()
        edge = {"_feats": {"edge": "left", "candidates": [{}]}, "reason": "marker cut"}
        self.assertIsNone(autonomy.recovery_hint([(now-2, edge)] * 12, 12, now, .5))
        self.assertIsNone(autonomy.recovery_hint([], 12, now, .5))
        hint = autonomy.recovery_hint([(now-.02, edge)] * 12, 12, now, .5)
        self.assertEqual(hint["edge"], "left")

    def test_dark_frame_with_clipped_floor_edge_never_triggers_recovery(self):
        result = self.est.estimate(np.zeros((480, 640, 3), np.uint8))
        now = time.monotonic()
        self.assertIsNone(autonomy.recovery_hint([(now-.01, result)] * 12, 12, now, .5))

    def make_runner(self, responses):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        args = pickup_real.parse(["align", "--fake", "--target-side", "left"])
        cam, base = Mock(), Mock()
        cam.look.side_effect = responses
        base.turn.return_value = {"turned_deg": 3.0}
        runner = pickup_real.Pickup(self.cfg, args, cam, Mock(), base, Mock(), Path(tmp.name))
        self.addCleanup(runner.events.close)
        runner.teach = self.teach
        runner.mount = self.teach["mount"]
        runner.target = (-.6, 0, 0)
        return runner, base

    def test_runner_automatically_recovers_edge_then_observes(self):
        good = self.render((-1.0, 0, 0))
        good["pitch_down_deg"] = 34
        runner, base = self.make_runner([
            {"ok": False, "recovery": {"kind": "edge", "edge": "left"}}, good])
        runner.look_pose("test", recover_view=True)
        base.turn.assert_called_once()
        self.assertEqual(runner.summary["view_recoveries"], 1)

    def test_blind_image_loss_never_causes_search_motion(self):
        runner, base = self.make_runner([{"ok": False, "reason": "image loss", "frames_seen": 0}])
        with self.assertRaises(pickup_real.Stop):
            runner.look_pose("test", recover_view=True)
        base.turn.assert_not_called()
        base.straight.assert_not_called()

    def test_recovery_has_finite_attempt_budget(self):
        edge = {"ok": False, "recovery": {"kind": "edge", "edge": "left"}}
        runner, base = self.make_runner([copy.deepcopy(edge) for _ in range(4)])
        with self.assertRaises(pickup_real.Stop):
            runner.look_pose("test", recover_view=True)
        self.assertEqual(base.turn.call_count, 3)

    def test_autonomous_alignment_with_actual_right_camera_offset(self):
        from test_pickup_real import TapeWorld
        cfg = copy.deepcopy(self.cfg)
        cfg["camera"].update(min_frames=4, acquire_timeout_s=3.0)
        for start in [(-1.15, .08, math.radians(5)), (-.75, .03, math.radians(-1)), (-1.10, -.06, math.radians(-4))]:
            with self.subTest(start=start), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                world = TapeWorld(cfg, start, seed=21)
                teach = copy.deepcopy(self.teach)
                teach["focal_scale"] = float(world.K_true[0, 0] / self.est.K[0, 0])
                path = root / "teach_tape.json"
                path.write_text(json.dumps(teach))
                args = pickup_real.parse(["align", "--fake", "--teach", str(path), "--target-side", "left"])
                args.world = world
                with patch.object(pickup_real, "STATE_FILE", root / "state.json"), patch.object(pickup_real, "LOG_ROOT", root):
                    runner = pickup_real.build(args, cfg, root / "run")
                    try:
                        ok = runner.run()
                    finally:
                        runner.cam.service.stop()
                self.assertTrue(ok, runner.summary.get("reason"))
                self.assertLess(abs(world.pose[1]), .025)
                self.assertLess(abs(math.degrees(world.pose[2])), 2.0)
                self.assertFalse(world.lifted)

    def test_single_command_fake_pickup_keeps_target_through_partial_view(self):
        from test_pickup_real import TapeWorld, tape_evaluate
        cfg = copy.deepcopy(self.cfg)
        cfg["camera"].update(min_frames=4, acquire_timeout_s=3.0)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            world = TapeWorld(cfg, (-1.15, .08, math.radians(5)), seed=21)
            teach = copy.deepcopy(self.teach)
            teach["focal_scale"] = float(world.K_true[0, 0] / self.est.K[0, 0])
            path = root / "teach_tape.json"
            path.write_text(json.dumps(teach))
            args = pickup_real.parse(["run", "--fake", "--yes-lift", "--teach", str(path), "--target-side", "left"])
            args.world = world
            with patch.object(pickup_real, "STATE_FILE", root / "state.json"), patch.object(pickup_real, "LOG_ROOT", root):
                runner = pickup_real.build(args, cfg, root / "run")
                try:
                    ok = runner.run()
                finally:
                    runner.cam.service.stop()
            self.assertTrue(ok, runner.summary.get("reason"))
            self.assertTrue(world.lifted)
            errors = tape_evaluate(world, cfg)
            self.assertLess(abs(errors["lateral_mm"]), 25)
            self.assertLess(abs(errors["yaw_deg"]), 2)
            self.assertLess(abs(errors["depth_mm"] - 150), 30)


if __name__ == "__main__":
    unittest.main()
