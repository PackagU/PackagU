#!/usr/bin/env python3
"""실제 장치 없이 마지막 삽입 구간의 현장 확인 경계를 검사한다."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import yaml

import pickup_real


class InsertPauseTests(unittest.TestCase):
    def make_pickup(self, root, poses, pause=True):
        cfg = yaml.safe_load((Path(__file__).parent / "config.yaml").read_text())
        extra = ["--pause-before-final"] if pause else []
        args = pickup_real.parse(["insert", "--fake", "--teach", str(root / "teach.json"), *extra])
        p = pickup_real.Pickup(cfg, args, Mock(), Mock(), Mock(), Mock(), root / "run")
        self.addCleanup(p.events.close)
        p.target = (-0.60, 0.0, 0.0)
        p.insert_total = 0.45
        p.teach = {"gap_m": 0.30}
        p.look_pose = Mock(side_effect=[(v, {"lateral": 0.0, "yaw": 0.0}) for v in poses])
        p.execute = Mock(return_value=[])
        return p

    def test_near_start_pauses_before_any_wheel_command(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            p = self.make_pickup(root, [(-0.67, 0.0, 0.0)])
            with patch.object(pickup_real, "STATE_FILE", root / "state.json"):
                with self.assertRaisesRegex(pickup_real.Stop, "paused before final"):
                    p.do_insert()
            p.execute.assert_not_called()
            self.assertEqual(json.loads((root / "state.json").read_text())["stage"], "paused_before_final")
            self.assertNotIn("inserted", p.summary)

    def test_far_start_approaches_then_pauses_before_final(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            p = self.make_pickup(root, [(-0.75, 0.0, 0.0), (-0.65, 0.0, 0.0)])
            with patch.object(pickup_real, "STATE_FILE", root / "state.json"):
                with self.assertRaisesRegex(pickup_real.Stop, "paused before final"):
                    p.do_insert()
            self.assertEqual(p.execute.call_count, 1)
            moves, speed = p.execute.call_args.args
            self.assertEqual(moves[0][0], "straight")
            self.assertAlmostEqual(moves[0][1], 0.10)
            self.assertEqual(speed, 0.03)
            self.assertNotIn("inserted", p.summary)

    def test_confirmed_insert_without_pause_can_finish(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            p = self.make_pickup(root, [(-0.67, 0.0, 0.0)], pause=False)
            with patch.object(pickup_real, "STATE_FILE", root / "state.json"):
                p.do_insert()
            self.assertEqual(p.execute.call_count, 1)
            self.assertAlmostEqual(p.execute.call_args.args[0][0][1], 0.52)
            self.assertEqual(json.loads((root / "state.json").read_text())["stage"], "inserted")


if __name__ == "__main__":
    unittest.main()
