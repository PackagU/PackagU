#!/usr/bin/env python3
"""Safety geometry and perception/controller contracts, without Gazebo."""
import ast
import math
from pathlib import Path
import unittest

import numpy as np
import yaml
from geometry import compose, engagement, marker_points, relative, rpy_matrix, stop_range
from configuration import validate

HERE = Path(__file__).resolve().parent


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.c = yaml.safe_load((HERE/'config.yaml').read_text())

    def test_stop_uses_sensor_fork_and_insertion(self):
        self.assertAlmostEqual(stop_range(self.c), .07)
        self.c['height_range']['stop_below'] = .777
        self.assertAlmostEqual(stop_range(self.c), .07)
        self.c['front_range']['mount_xyz'][0] += .01
        self.assertAlmostEqual(stop_range(self.c), .06)

    def test_sensor_angle_affects_stop_geometry(self):
        self.c['front_range']['mount_rpy'][2] = .2
        self.assertAlmostEqual(stop_range(self.c), .07/math.cos(.2))

    def test_reflection_face_offset_is_not_insertion_distance(self):
        self.c['pallet']['parcel_front_offset_m'] = .012
        self.assertAlmostEqual(stop_range(self.c), .082)

    def test_configuration_enforces_fork_and_sensor_clearances(self):
        validate(self.c)
        self.c['pallet']['insertion_depth'] = .18
        with self.assertRaises(AssertionError):
            validate(self.c)

    def test_prior_run_config_snapshot_remains_replayable(self):
        self.c['robot'].pop('tine_width')
        self.c['pallet'].pop('parcel_size_m')
        self.c['pallet'].pop('parcel_front_offset_m')
        validate(self.c)
        self.assertEqual(self.c['pallet']['parcel_size_m'], [.28, .30, .40])
        self.assertAlmostEqual(stop_range(self.c), .07)

    def test_good_engagement_and_preserved_relative_pose(self):
        base, entry = [.9, .2, .3], compose([.9, .2, .3], [.11, 0, 0])
        self.assertTrue(engagement(self.c, base, entry)['ok'])
        np.testing.assert_allclose(compose(base, relative(base, entry)), entry, atol=1e-10)

    def test_attach_rejects_lateral_yaw_shallow_and_overinserted(self):
        for entry in ([.11, .12, 0], [.11, 0, .20], [.20, 0, 0], [.04, 0, 0]):
            self.assertFalse(engagement(self.c, [0, 0, 0], entry)['ok'], entry)

    def test_swept_fork_must_fit_not_just_tip(self):
        self.c['pallet']['max_yaw'] = .6
        self.c['pallet']['pocket_width'] = .18
        self.assertFalse(engagement(self.c, [0, 0, .3], [.10, .07, 0])['ok'])

    def test_control_has_no_truth_inputs(self):
        for name in ('controller.py', 'vision.py'):
            source = (HERE/name).read_text()
            for forbidden in ('/sim/ground_truth', '/gazebo/model_states', 'SetEntityState', 'GetEntityState', "['pallet_pose']"):
                self.assertNotIn(forbidden, source)
            ast.parse(source)

    def test_no_lateral_velocity_command(self):
        self.assertNotIn('linear.y =', (HERE/'controller.py').read_text())

    def test_faceted_image_pose_recovers_camera_to_fork_offsets(self):
        import cv2
        from vision import fit_pocket, OPTICAL_TO_BODY
        obj = np.concatenate([marker_points(self.c, f) for f in self.c['pallet']['marker_facets']])
        K = np.array([[400., 0, 320.5], [0, 400., 240.5], [0, 0, 1.]])
        D = np.zeros(5)
        mount = np.array(self.c['camera']['mount_xyz'])
        for x, y, yaw, z in ((1.45, .08, 0, 0), (.7, -.05, -.1, 0), (.11, 0, .02, .034), (1.1, .1, .2, 0)):
            R = OPTICAL_TO_BODY.T @ rpy_matrix([0, 0, yaw])
            t = OPTICAL_TO_BODY.T @ (np.array([x, y, z])-mount)
            uv = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], t, K, D)[0].reshape(-1, 2)
            result = fit_pocket(self.c, obj, uv, K, D)
            self.assertIsNotNone(result)
            np.testing.assert_allclose(result['pocket_base'], [x, y, yaw], atol=1e-5)
        self.assertIsNone(fit_pocket(self.c, obj[:4], uv[:4], K, D))


if __name__ == '__main__':
    unittest.main(verbosity=2)
