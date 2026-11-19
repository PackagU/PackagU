#!/usr/bin/env python3
"""Simulation sensor relay, guarded actuator mock and independent truth evaluator.

Ground truth never leaves this node as a control target. Kinematic attach preserves
the current relative transform and is explicitly not a force/payload simulation.
"""
import copy
import csv
import json
import math
from pathlib import Path

import numpy as np
from gazebo_msgs.msg import ContactsState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from controller import fresh, valid_range
from geometry import compose, engagement, predicted_range, relative
from vision import pose, stamp


class WorldIO(Node):
    def __init__(self, c, case, out):
        super().__init__('pickup_sensor_and_evaluator', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.c, self.case, self.out = c, case, Path(out)
        self.rngs = {key: np.random.default_rng(c['seed']+index) for index, key in enumerate(('approach', 'lift_limit'), 1)}
        self.truth, self.parcel, self.parcel_z, self.attached_rel = None, None, 0.0, None
        self.lift, self.attached_lift, self.max_lift = 0.0, 0.0, 0.0
        self.raw, self.published, self.last_forward = {}, {}, {}
        self.fault_t, self.approach_t = None, None
        self.phase, self.reason, self.state = 'STARTUP', None, {}
        self.estimate, self.odom_t, self.odom_pose, self.joint_t = None, None, None, None
        self.request, self.request_t = Twist(), None
        self.lift_request, self.lift_request_t = None, None
        self.contacts_count, self.collisions = 0, []
        self.attach_event, self.pending = None, None
        self.stop_time, self.max_v_after_deadline, self.max_w_after_deadline = None, 0.0, 0.0
        self.last_row, self.last_lift, self.last_fault = -1, -1, -1
        self.command = self.create_publisher(Twist, '/sim/cmd_vel_drive', 10)
        self.lift_pub = self.create_publisher(JointTrajectory, '/set_lift_trajectory', 10)
        self.attachment_pub = self.create_publisher(String, '/pickup/attachment', 10)
        self.fault_pub = self.create_publisher(String, '/pickup/fault', 10)
        self.range_pubs = {}
        for key, conf in [('approach', c['front_range']), ('lift_limit', c['height_range'])]:
            self.range_pubs[key] = self.create_publisher(Range, conf['topic'], 10)
            self.create_subscription(Range, '/pickup/raw/'+key, lambda m, k=key: self.sensor(k, m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/ground_truth', lambda m: setattr(self, 'truth', m), 20)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.models, 10)
        self.create_subscription(Odometry, '/odom', self.odometry, 20)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(String, '/pickup/state', self.on_state, 10)
        self.create_subscription(String, '/pickup/estimate', lambda m: setattr(self, 'estimate', json.loads(m.data)), 10)
        self.create_subscription(Twist, '/pickup/request_vel', self.velocity, 10)
        self.create_subscription(Float64, '/pickup/request_lift', self.request_lift, 10)
        self.create_subscription(String, '/pickup/attach_request', self.attach, 10)
        self.create_subscription(ContactsState, '/pickup/contacts', self.contacts, 20)
        self.set_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.rows_file = (self.out/'evaluation.csv').open('w', buffering=1)
        self.rows = csv.writer(self.rows_file)
        self.rows.writerow(['sim_time', 'phase', 'robot_x', 'robot_y', 'robot_yaw', 'parcel_x', 'parcel_y',
                            'lateral_error_m', 'angle_error_rad', 'insertion_depth_m', 'front_range_m', 'lift_m',
                            'linear_speed_mps', 'angular_speed_radps', 'fault_active', 'collision'])
        self.create_timer(.025, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def models(self, m):
        if 'pickup_parcel' in m.name:
            p = m.pose[m.name.index('pickup_parcel')]
            q = p.orientation
            self.parcel = [p.position.x, p.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
            self.parcel_z = p.position.z

    def joints(self, m):
        if 'lift_joint' in m.name:
            self.joint_t = stamp(m)
            self.lift = m.position[m.name.index('lift_joint')]
            self.max_lift = max(self.max_lift, self.lift)

    def odometry(self, m):
        self.odom_t, self.odom_pose = stamp(m), pose(m)

    def on_state(self, m):
        self.state = json.loads(m.data)
        self.phase, self.reason = self.state['phase'], self.state.get('reason')
        if self.phase == 'APPROACH' and self.approach_t is None:
            self.approach_t = self.now()

    def velocity(self, m):
        self.request, self.request_t = m, self.now()

    def request_lift(self, m):
        self.lift_request, self.lift_request_t = m.data, self.now()

    def sensor(self, key, message):
        self.raw[key] = message
        cfg = self.c['front_range' if key == 'approach' else 'height_range']
        rng = self.rngs[key]
        m = copy.deepcopy(message)
        if rng.random() < cfg['dropout_probability']:
            return
        fault = self.case.get('fault') if self.fault_t is not None and key == 'approach' else None
        if fault == 'range_loss':
            return
        if fault == 'range_stale':
            t = self.fault_t-1.0
            m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t%1)*1e9)
        if fault == 'range_outlier':
            m.range = .035 # In-range but inconsistent with image: must not trigger a false successful dock.
        elif fault == 'range_nan':
            m.range = float('nan')
        elif math.isfinite(m.range):
            m.range += float(rng.normal(0, cfg['noise_stddev']))
        self.published[key] = (stamp(m), m.range)
        self.range_pubs[key].publish(m)

    def contacts(self, m):
        self.contacts_count += 1
        for s in m.states:
            names = s.collision1_name+' '+s.collision2_name
            if 'packagu_sim' in names and ('lead_screw_carrier' not in names or self.attached_rel is None):
                if len(self.collisions) < 20:
                    self.collisions.append(dict(stamp=self.now(), pair=names))

    def attach(self, m):
        if self.attach_event is not None:
            self.attachment_pub.publish(String(data=json.dumps({'accepted': self.attach_event['accepted']})))
            return
        if self.truth is None or self.parcel is None:
            return
        base = pose(self.truth)
        metrics = engagement(self.c, base, self.parcel, self.lift-self.parcel_z)
        stopped = abs(self.truth.twist.twist.linear.x) < .006 and abs(self.truth.twist.twist.angular.z) < .02
        accepted = metrics['ok'] and stopped and not self.collisions and self.guard_ok()
        self.attach_event = dict(accepted=accepted, geometry=metrics, stamp=self.now(),
                                 stop_range_m=self.published.get('approach', (0, None))[1],
                                 validation='kinematic_attach_with_geometry_gate', physical_load_validated=False)
        if accepted:
            self.attached_rel = relative(base, self.parcel)
            self.attached_lift = self.lift
            unchanged = compose(base, self.attached_rel)
            self.attach_event['relative_pose'] = self.attached_rel
            self.attach_event['entry_pose_at_attach'] = self.parcel.copy()
            self.attach_event['snap_distance_m'] = math.dist(unchanged[:2], self.parcel[:2])
            self.attach_event['snap_yaw_rad'] = abs(unchanged[2]-self.parcel[2])
        # Actuator feedback is binary. Evaluation coordinates/errors stay only in log files.
        self.attachment_pub.publish(String(data=json.dumps({'accepted': bool(accepted)})))
        (self.out/'attach.json').write_text(json.dumps(self.attach_event, indent=2))

    def guard_ok(self):
        t = self.now()
        if not fresh(self.odom_t, t, self.c['control']['odom_max_age']):
            return False
        if self.estimate is None or not fresh(self.estimate['stamp'], t, self.c['camera']['max_age']):
            return False
        u = self.published.get('approach')
        if u is None or not fresh(u[0], t, self.c['front_range']['max_age']) or not valid_range(u[1], self.c['front_range']):
            return False
        if self.phase in ('APPROACH', 'VERIFY_INSERTION', 'ATTACH_WAIT', 'LIFT', 'BACK_OUT'):
            expected = predicted_range(self.c, relative(self.odom_pose, self.estimate['pocket_odom']))
            if abs(u[1]-expected) > self.c['front_range']['camera_disagreement']:
                return False
        return True

    def lift_to(self, target):
        traj = JointTrajectory()
        traj.header.frame_id = 'world'
        traj.joint_names = ['lift_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(target)]
        point.time_from_start = Duration(nanosec=100000000)
        traj.points = [point]
        self.lift_pub.publish(traj)

    def tick(self):
        t = self.now()
        if self.case.get('fault') and self.approach_t is not None and t-self.approach_t >= .8:
            if self.fault_t is None:
                self.fault_t = t
            if t-self.last_fault > .1:
                self.last_fault = t
                self.fault_pub.publish(String(data=json.dumps(dict(kind=self.case['fault'], stamp=self.fault_t))))
        safe = self.guard_ok() and not self.collisions
        cmd = self.request if safe and fresh(self.request_t, t, self.c['control']['command_timeout']) else Twist()
        self.command.publish(cmd)
        if self.attached_rel is not None and t-self.last_lift >= .10:
            self.last_lift = t
            target = self.lift
            h = self.published.get('lift_limit')
            good_height = h and fresh(h[0], t, self.c['height_range']['max_age']) and valid_range(h[1], self.c['height_range'])
            if safe and good_height and fresh(self.joint_t, t, self.c['control']['odom_max_age']) and self.phase == 'LIFT' and h[1] >= self.c['height_range']['stop_below']:
                if fresh(self.lift_request_t, t, self.c['control']['command_timeout']):
                    target = max(self.lift, min(self.lift_request, self.c['control']['lift_target']))
            self.lift_to(target)
        if self.truth is None or self.parcel is None:
            return
        base = pose(self.truth)
        if self.attached_rel is not None and (self.pending is None or self.pending.done()):
            entry = compose(base, self.attached_rel)
            req = SetEntityState.Request()
            req.state.name, req.state.reference_frame = 'pickup_parcel', 'world'
            req.state.pose.position.x, req.state.pose.position.y = entry[:2]
            req.state.pose.position.z = self.lift-self.attached_lift
            req.state.pose.orientation.z, req.state.pose.orientation.w = math.sin(entry[2]/2), math.cos(entry[2]/2)
            self.pending = self.set_client.call_async(req)
        v = math.hypot(self.truth.twist.twist.linear.x, self.truth.twist.twist.linear.y)
        w = abs(self.truth.twist.twist.angular.z)
        if self.fault_t is not None:
            if t > self.fault_t+.05 and v < .004 and w < .015 and self.stop_time is None:
                self.stop_time = t-self.fault_t
            if t >= self.fault_t+.65:
                self.max_v_after_deadline = max(self.max_v_after_deadline, v)
                self.max_w_after_deadline = max(self.max_w_after_deadline, w)
        if t-self.last_row >= .10:
            self.last_row = t
            e = engagement(self.c, base, self.parcel, self.lift-self.parcel_z)
            self.rows.writerow([t, self.phase, *base, *self.parcel[:2], e['lateral_error_m'], e['angle_error_rad'],
                                e['insertion_depth_m'], self.published.get('approach', (0, None))[1], self.lift,
                                v, w, self.fault_t is not None, bool(self.collisions)])

    def result(self):
        fault = self.case.get('fault')
        final = engagement(self.c, pose(self.truth), self.parcel, self.lift-self.parcel_z) if self.truth and self.parcel else None
        true_stop = predicted_range(self.c, relative(pose(self.truth), self.parcel)) if self.truth and self.parcel else None
        safe_fault = (self.phase == 'FAILED' and self.fault_t is not None and self.max_lift < .003
                      and self.attached_rel is None and self.max_v_after_deadline < .005
                      and self.max_w_after_deadline < .02 and self.stop_time is not None and not self.collisions)
        carry_error = None
        moved = None
        if self.attached_rel is not None and self.parcel:
            expected = compose(pose(self.truth), self.attached_rel)
            carry_error = math.dist(expected[:2], self.parcel[:2])
            moved = math.dist(self.parcel[:2], self.attach_event['entry_pose_at_attach'][:2])
        nominal = (self.phase == 'DONE' and self.attach_event and self.attach_event['accepted'] and not self.collisions
                   and self.max_lift >= self.c['control']['lift_minimum'] and carry_error is not None
                   and carry_error < .012 and moved is not None and moved > .30 and self.contacts_count > 0)
        return dict(passed=bool(safe_fault if fault else nominal), expected='safe_stop_no_lift' if fault else 'pickup_and_back_out',
                    phase=self.phase, reason=self.reason, attach=self.attach_event, final_geometry=final,
                    max_lift_m=self.max_lift, stop_latency_s=self.stop_time, fault=fault, fault_stamp=self.fault_t,
                    max_speed_after_fault_deadline_mps=self.max_v_after_deadline,
                    max_yaw_speed_after_fault_deadline_radps=self.max_w_after_deadline,
                    collision=bool(self.collisions), contacts_messages=self.contacts_count, collisions=self.collisions,
                    carry_relative_error_m=carry_error, parcel_back_out_m=moved,
                    stop_distance_m=true_stop, final_range_measured_m=self.published.get('approach', (0, None))[1],
                    height_limit_range_m=self.published.get('lift_limit', (0, None))[1],
                    lift_stop_reason=self.state.get('lift_stop_reason'),
                    validation='rendered_camera + Gazebo_ray_ranges + geometry_gated_kinematic_attach',
                    physical_payload_validated=False, seed=self.c['seed'], physics_rate=1.0)
