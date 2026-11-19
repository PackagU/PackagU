#!/usr/bin/env python3
"""Rendered noncoplanar ArUco fixture -> pocket pose. No simulator truth input."""
import collections
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from geometry import compose, marker_points, rpy_matrix

OPTICAL_TO_BODY = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)


def stamp(m):
    return m.header.stamp.sec+m.header.stamp.nanosec*1e-9


def pose(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


def fit_pocket(c, obj, uv, K, D):
    """Image geometry only. Both fixture faces are required to resolve frontal yaw."""
    if len(obj) < 8 or np.linalg.matrix_rank(obj-obj.mean(axis=0)) < 3:
        return None
    Rbc = rpy_matrix(c['camera']['mount_rpy']) @ OPTICAL_TO_BODY
    mount = np.array(c['camera']['mount_xyz'])
    ok, rv, tv = cv2.solvePnP(obj, uv, K, D, flags=cv2.SOLVEPNP_SQPNP)
    if not ok or not np.all(np.isfinite(tv)):
        return None
    Rcm = cv2.Rodrigues(rv)[0]
    if (Rcm @ obj.mean(axis=0)+tv[:, 0])[2] <= 0:
        return None
    Rbm, xyz = Rbc @ Rcm, mount+Rbc @ tv[:, 0]
    yaw = math.atan2(Rbm[1, 0], Rbm[0, 0])

    def residual(q):
        points = (rpy_matrix([0, 0, q[2]]) @ obj.T).T+q[[0, 1, 3]]
        optical = (Rbc.T @ (points-mount).T).T
        pixel = optical @ K.T
        return (pixel[:, :2]/pixel[:, 2:3]-uv).ravel()

    initial = [np.clip(xyz[0], .031, 4.99), np.clip(xyz[1], -4.99, 4.99),
               np.clip(yaw, -1.29, 1.29), np.clip(xyz[2], -.099, .199)]
    fit = least_squares(residual, initial, max_nfev=30,
                        bounds=([.03, -5, -1.3, -.10], [5, 5, 1.3, .20]))
    err = float(np.sqrt(np.mean(residual(fit.x)**2)))
    distance = float(np.linalg.norm(Rcm @ obj.mean(axis=0)+tv[:, 0]))
    if err > c['camera']['max_reprojection_px'] or not c['camera']['range_min'] <= distance <= c['camera']['range_max']:
        return None
    return dict(pocket_base=fit.x[:3].tolist(), image_height_offset_m=float(fit.x[3]), reprojection_px=err)


class Vision(Node):
    def __init__(self, c, out, gui=False):
        super().__init__('pickup_rendered_vision', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.c, self.out, self.gui = c, Path(out), gui
        self.out.mkdir(exist_ok=True)
        self.K, self.D, self.last_saved, self.latest = None, None, -100, None
        self.odom, self.filtered = collections.deque(maxlen=300), collections.deque(maxlen=5)
        self.fault = None
        self.rng = np.random.default_rng(c['seed']+31)
        self.pub = self.create_publisher(String, '/pickup/estimate', 10)
        self.create_subscription(CameraInfo, c['camera']['info_topic'], self.info, qos_profile_sensor_data)
        self.create_subscription(Image, c['camera']['topic'], self.image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: self.odom.append((stamp(m), pose(m))), 20)
        self.create_subscription(String, '/pickup/fault', lambda m: setattr(self, 'fault', json.loads(m.data)), 10)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.facets = c['pallet']['marker_facets']
        self.objects = {f['id']: marker_points(c, f) for f in self.facets}

    def info(self, m):
        self.K, self.D = np.array(m.k).reshape(3, 3), np.array(m.d)
        path = self.out/'camera_info.json'
        if not path.exists():
            path.write_text(json.dumps(dict(k=list(m.k), d=list(m.d), width=m.width, height=m.height)))

    def image(self, m):
        fault = (self.fault or {}).get('kind')
        if fault == 'image_loss' or self.rng.random() < self.c['camera']['dropout_probability']:
            return
        if self.K is None or not self.odom:
            return
        t = stamp(m)
        od = min(self.odom, key=lambda h: abs(h[0]-t))
        if abs(od[0]-t) > .15:
            return
        channels = 4 if m.encoding in ('rgba8', 'bgra8') else 3
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width*channels].reshape(m.height, m.width, channels)
        frame = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a[:, :, :3].copy()
        raw = frame.copy() if t-self.last_saved > 2 else None
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.params)
        found = {} if ids is None else {int(i): corner for i, corner in zip(ids.flatten(), corners)}
        output = None
        if all(f['id'] in found for f in self.facets):
            obj = np.concatenate([self.objects[f['id']] for f in self.facets]).astype(np.float64)
            uv = np.concatenate([found[f['id']].reshape(4, 2) for f in self.facets]).astype(np.float64)
            output = fit_pocket(self.c, obj, uv, self.K, self.D)
            if output is not None:
                self.filtered.append(compose(od[1], output['pocket_base']))
                output.update(stamp=t, pocket_odom=np.median(np.array(self.filtered), axis=0).tolist(),
                              source='rendered_aruco_faceted_23_24', image_odom_dt=od[0]-t)
                if fault == 'image_stale':
                    output['stamp'] = self.fault['stamp']-1.0
                self.pub.publish(String(data=json.dumps(output)))
                self.latest = output
        relevant = [i for i in self.objects if i in found]
        if relevant:
            cv2.aruco.drawDetectedMarkers(frame, [found[i] for i in relevant], np.array(relevant).reshape(-1, 1))
        label = 'BOTH MARKERS REQUIRED' if output is None else 'entry x=%.3f y=%.3f yaw=%.1f deg' % (*output['pocket_base'][:2], math.degrees(output['pocket_base'][2]))
        cv2.putText(frame, label, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .52, (0, 255, 255), 1)
        if t-self.last_saved > 2:
            self.last_saved = t
            cv2.imwrite(str(self.out/f'{t:09.3f}_raw.png'), raw)
            cv2.imwrite(str(self.out/f'{t:09.3f}.png'), frame)
            cv2.imwrite(str(self.out/'latest.png'), frame)
        if self.gui:
            cv2.imwrite(str(self.out/'latest.png'), frame)
