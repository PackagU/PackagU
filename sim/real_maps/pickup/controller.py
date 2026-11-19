#!/usr/bin/env python3
"""Differential-drive pickup FSM. Input: image estimates, ranges, encoder odom, joints.

Never imports Gazebo services or subscribes to simulator/evaluator truth topics.
"""
import collections
import json
import math
from pathlib import Path

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float64, String

from geometry import compose, engagement, predicted_range, relative, stop_range, wrap
from vision import pose, stamp


def fresh(t, now, maximum):
    # Gazebo /clock is discrete; sensor stamps may lead it by one publish period.
    return t is not None and -.12 <= now-t <= maximum


def valid_range(value, cfg):
    return value is not None and math.isfinite(value) and cfg['range_min'] <= value <= cfg['range_max']


class Controller(Node):
    def __init__(self, c, out):
        super().__init__('pickup_controller', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.c, self.p, self.out = c, c['control'], Path(out)
        self.phase, self.phase_t, self.started = 'WAIT_TARGET', None, None
        self.est, self.odom, self.us, self.height = None, None, None, None
        self.lift, self.joint_t, self.attachment = 0.0, None, None
        self.bad_since, self.retry, self.target = None, 0, None
        self.est_hist = collections.deque(maxlen=7)
        self.last_est_t = None
        self.stable, self.last_note, self.last_lift = 0, -1, -1
        self.attach_sent = False
        self.failure, self.terminal_t = None, None
        self.lift_stop_reason = None
        self.cmd = self.create_publisher(Twist, '/pickup/request_vel', 10)
        self.lift_pub = self.create_publisher(Float64, '/pickup/request_lift', 10)
        self.state_pub = self.create_publisher(String, '/pickup/state', 10)
        self.attach_pub = self.create_publisher(String, '/pickup/attach_request', 10)
        self.create_subscription(String, '/pickup/estimate', self.estimate, 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', (stamp(m), pose(m))), 20)
        self.create_subscription(Range, c['front_range']['topic'], lambda m: setattr(self, 'us', (stamp(m), m.range)), 10)
        self.create_subscription(Range, c['height_range']['topic'], lambda m: setattr(self, 'height', (stamp(m), m.range)), 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(String, '/pickup/attachment', lambda m: setattr(self, 'attachment', json.loads(m.data)), 10)
        self.trace = (self.out/'controller.jsonl').open('w', buffering=1)
        self.create_timer(1/self.p['rate_hz'], self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def estimate(self, m):
        e = json.loads(m.data)
        self.est = e
        if e['stamp'] != self.last_est_t:
            self.est_hist.append(e['pocket_odom'])
            self.last_est_t = e['stamp']

    def joints(self, m):
        if 'lift_joint' in m.name:
            self.lift, self.joint_t = m.position[m.name.index('lift_joint')], stamp(m)

    def move(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd.publish(msg) # linear.y always 0: no lateral actuator

    def hold_lift(self, target=None):
        if self.now()-self.last_lift >= .10:
            self.last_lift = self.now()
            self.lift_pub.publish(Float64(data=float(self.lift if target is None else target)))

    def enter(self, phase, **data):
        self.phase, self.phase_t, self.stable = phase, self.now(), 0
        self.move()
        self.log(event='transition', **data)
        print(f'PICKUP {phase} {data}', flush=True)

    def log(self, **extra):
        measured = None if self.us is None or not math.isfinite(self.us[1]) else self.us[1]
        self.trace.write(json.dumps(dict(t=self.now(), phase=self.phase, retry=self.retry, lift_m=self.lift,
                                        range_m=measured, **extra), allow_nan=False)+'\n')

    def fail(self, reason):
        self.failure, self.terminal_t = reason, self.now()
        self.enter('FAILED', reason=reason)
        self.hold_lift()

    def sensor_problem(self):
        t = self.now()
        if self.odom is None or not fresh(self.odom[0], t, self.p['odom_max_age']):
            return 'ODOM_STALE'
        if self.est is None or not fresh(self.est['stamp'], t, self.c['camera']['max_age']):
            return 'IMAGE_STALE_OR_LOST'
        u = self.us
        if u is None or not fresh(u[0], t, self.c['front_range']['max_age']):
            return 'RANGE_STALE_OR_LOST'
        if not valid_range(u[1], self.c['front_range']):
            return 'RANGE_INVALID'
        local = relative(self.odom[1], self.est['pocket_odom'])
        expected = predicted_range(self.c, local)
        if self.phase in ('APPROACH', 'VERIFY_INSERTION', 'ATTACH_WAIT', 'LIFT', 'BACK_OUT') and abs(u[1]-expected) > self.c['front_range']['camera_disagreement']:
            return 'RANGE_CAMERA_DISAGREEMENT'
        if self.phase in ('LIFT', 'BACK_OUT'):
            h = self.height
            if h is None or not fresh(h[0], t, self.c['height_range']['max_age']) or not valid_range(h[1], self.c['height_range']):
                return 'HEIGHT_RANGE_INVALID_OR_STALE'
            if not fresh(self.joint_t, t, self.p['odom_max_age']):
                return 'LIFT_FEEDBACK_STALE'
        return None

    def predock(self, radial=False):
        target = np.median(np.array(self.est_hist), axis=0).tolist()
        if radial:
            # First move to a better visual vantage point. A frontal distant planar
            # marker has weak yaw observability; refine its normal from closer images.
            target[2] = math.atan2(target[1]-self.odom[1][1], target[0]-self.odom[1][0])
        self.target = compose(target, [-self.p['predock_distance'], 0, 0])
        self.enter('ALIGN_AT_DISTANCE', target=self.target)

    def drive(self, target, reverse=False):
        current = self.odom[1]
        dx, dy = target[0]-current[0], target[1]-current[1]
        d = math.hypot(dx, dy)
        if d < .012:
            err = wrap(target[2]-current[2])
            if abs(err) < .02:
                self.move()
                return True
            self.move(0, np.clip(1.8*err, -self.p['angular_speed'], self.p['angular_speed']))
            return False
        bearing = wrap(math.atan2(dy, dx)-current[2]-(math.pi if reverse else 0))
        speed = min(self.p['linear_speed'], .7*d) * (-1 if reverse else 1)
        self.move(speed if abs(bearing) < .7 else 0, np.clip(2*bearing, -self.p['angular_speed'], self.p['angular_speed']))
        return False

    def tick(self):
        t = self.now()
        if self.started is None:
            self.started, self.phase_t = t, t
        if self.phase in ('DONE', 'FAILED'):
            self.move()
            self.hold_lift()
            self.publish()
            return
        if t-self.started > self.p['total_timeout']:
            self.fail('TOTAL_TIMEOUT')
        elif t-self.phase_t > (self.p['acquire_timeout'] if self.phase == 'WAIT_TARGET' else self.p['stage_timeout']):
            self.fail('STAGE_TIMEOUT_'+self.phase)
        else:
            problem = self.sensor_problem()
            if problem:
                self.move()
                if self.phase == 'LIFT':
                    self.hold_lift()
                if self.bad_since is None:
                    self.bad_since = t
                    self.log(event='sensor_stop', reason=problem)
                if self.phase != 'WAIT_TARGET' and t-self.bad_since > self.p['sensor_failure_timeout']:
                    self.fail(problem)
            else:
                self.bad_since = None
                self.step()
        if t-self.last_note > .2:
            self.log(estimate=self.est, odom=self.odom)
            self.last_note = t
        self.publish()

    def publish(self):
        self.state_pub.publish(String(data=json.dumps(dict(phase=self.phase, reason=self.failure, retry=self.retry,
                                                           stamp=self.now(), lift_m=self.lift,
                                                           lift_stop_reason=self.lift_stop_reason))))

    def step(self):
        t = self.now()
        if self.phase == 'WAIT_TARGET':
            self.move()
            if len(self.est_hist) >= 5:
                self.predock(radial=True)
        elif self.phase == 'ALIGN_AT_DISTANCE':
            if self.drive(self.target):
                self.enter('CHECK_ALIGNMENT')
        elif self.phase == 'CHECK_ALIGNMENT':
            self.move()
            if t-self.phase_t < .9:
                return
            target = self.est['pocket_odom']
            cross = relative(target, self.odom[1])[1]
            yaw = wrap(target[2]-self.odom[1][2])
            if abs(cross) < self.p['alignment_lateral'] and abs(yaw) < self.p['alignment_yaw']:
                self.enter('APPROACH')
            else:
                self.retry += 1
                if self.retry > self.p['max_retries']:
                    self.fail('ALIGN_RETRY_LIMIT')
                else:
                    self.target = compose(self.odom[1], [-self.p['realign_retreat_distance'], 0, 0])
                    self.enter('RETREAT_REALIGN', reason='leave room for differential-drive lateral correction')
        elif self.phase in ('APPROACH', 'VERIFY_INSERTION'):
            target = self.est['pocket_odom']
            cross = relative(target, self.odom[1])[1]
            yaw = wrap(target[2]-self.odom[1][2])
            u, stop = self.us[1], stop_range(self.c)
            if abs(cross) > self.p['realign_lateral'] or abs(yaw) > self.p['realign_yaw']:
                self.retry += 1
                if self.retry > self.p['max_retries']:
                    self.fail('APPROACH_ALIGNMENT_RETRY_LIMIT')
                else:
                    back = self.p['realign_retreat_distance']
                    self.target = compose(self.odom[1], [-max(back, self.p['predock_distance']+back-(u+self.c['front_range']['mount_xyz'][0])), 0, 0])
                    self.enter('RETREAT_REALIGN', cross=cross, yaw=yaw)
            elif u < stop-self.c['pallet']['insertion_tolerance']:
                self.fail('APPROACH_OVERSHOOT')
            elif self.phase == 'APPROACH':
                if u <= stop+.003:
                    self.enter('VERIFY_INSERTION', stop_range_m=u, desired_stop_range_m=stop)
                else:
                    v = min(self.p['approach_speed'], .6*(u-stop))
                    self.move(v, np.clip(1.8*yaw-1.2*cross, -.09, .09))
            elif t-self.phase_t >= self.p['stop_settle']:
                observed = engagement(self.c, self.odom[1], target, self.lift)
                if not observed['ok']:
                    self.fail('OBSERVED_INSERTION_UNSAFE')
                else:
                    self.enter('ATTACH_WAIT', observed=observed)
                    self.attach_pub.publish(String(data='{"cmd":"attach"}'))
        elif self.phase == 'RETREAT_REALIGN':
            if self.drive(self.target, reverse=True):
                self.enter('SETTLE_REOBSERVE')
        elif self.phase == 'SETTLE_REOBSERVE':
            self.move()
            # Replace images captured during reverse acceleration with stationary frames
            # before using the marker normal to plan the next lateral correction.
            if t-self.phase_t >= 1.0:
                self.predock()
        elif self.phase == 'ATTACH_WAIT':
            self.move()
            if self.attachment is not None:
                if self.attachment['accepted']:
                    self.enter('LIFT')
                else:
                    self.fail('ATTACH_GATE_REJECTED')
            elif t-self.phase_t > 2:
                self.fail('ATTACH_RESPONSE_TIMEOUT')
        elif self.phase == 'LIFT':
            self.move()
            if self.height[1] < self.c['height_range']['stop_below'] or self.lift >= self.p['lift_target']-.003:
                self.lift_stop_reason = 'height_limit_10cm' if self.height[1] < self.c['height_range']['stop_below'] else 'configured_lift_target'
                self.hold_lift()
                if self.lift < self.p['lift_minimum']:
                    self.fail('HEIGHT_LIMIT_BEFORE_CLEARANCE')
                else:
                    self.target = compose(self.odom[1], [-self.p['retreat_distance'], 0, 0])
                    self.enter('BACK_OUT', lift_stop_reason=self.lift_stop_reason, height_range_m=self.height[1])
            else:
                self.hold_lift(min(self.p['lift_target'], self.lift+self.p['lift_speed']*.15))
        elif self.phase == 'BACK_OUT':
            self.hold_lift()
            if self.drive(self.target, reverse=True):
                self.terminal_t = t
                self.enter('DONE')
