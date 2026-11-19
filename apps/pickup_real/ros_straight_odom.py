#!/usr/bin/env python3
"""직진 전용 이동·센서 점검 helper (ROS 컨테이너 안에서 실행).

이동: /cmd_vel 에 linear.x 만 보낸다(angular.z 는 항상 0 — 회전 명령 없음). 명령은 기존 경로
/cmd_vel → nav_safety_gate → /cmd_vel_safe → OpenCR 를 그대로 지난다. 이동 누계는 명령값이 아니라
시작 odom 자세 대비 실제 odom 자세로 계산한다(정지 명령 뒤 굴러간 거리도 settle 동안 포함).
  forward_m  = 시작 heading 방향으로 간 거리(+ 전진)   lateral_m = 시작 heading 왼쪽(+)으로 벗어난 거리
  yaw_change_deg = 시작 대비 odom yaw 변화
다음이면 즉시 0 을 보내고 실패로 끝낸다: yaw 변화·좌우 이탈 한도 초과, odom 끊김, gate ready=false,
속도 명령 대비 실제 속도가 오래 낮음(막힘·미끄럼), 시간 초과.

점검(--probe): 움직이지 않는다. scan/odom 주기·나이, /drive/ready, gate ready·stop·status, /cmd_vel 발행자 수,
라이다 후방(로봇 뒤 띠) 가장 가까운 거리를 잰다.

결과는 마지막 줄 `STRAIGHT_ODOM_RESULT {json}`.
"""
import argparse
import json
import math
import sys
import time

MAX_DISTANCE_M = 1.2
MAX_SPEED_MPS = 0.06
MIN_SPEED_MPS = 0.02
ACCEL_MPS2 = 0.05
ODOM_STALE_SEC = 0.5
GATE_WAIT_SEC = 6.0
DISCOVERY_SEC = 10.0
GATE_DROP_TOL = 1.0
HOLD_K, HOLD_MAX_W = 2.0, 0.25   # 방향 유지: rad 오차 x2 → rad/s, 최대 0.25
DONE_TOL_M = 0.003
TAG = "STRAIGHT_ODOM_RESULT "


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def relative(start, now):
    """시작 자세 (x, y, yaw) 기준 지금 자세 → (forward, lateral, dyaw)."""
    dx, dy = now[0] - start[0], now[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return dx * c + dy * s, -dx * s + dy * c, wrap(now[2] - start[2])


def plan_speed(elapsed, remaining, cruise):
    accel_limited = MIN_SPEED_MPS + ACCEL_MPS2 * elapsed
    decel_limited = max(MIN_SPEED_MPS, remaining * 0.8)
    return max(MIN_SPEED_MPS, min(cruise, accel_limited, decel_limited))


class Stall:
    """명령 속도 대비 실제 odom 속도가 window 동안 ratio 미만이면 막힘. 가속 구간(grace) 뒤부터 본다."""

    def __init__(self, grace_s=1.5, window_s=2.0, ratio=0.3):
        self.grace, self.window, self.ratio = grace_s, window_s, ratio
        self.samples = []   # (t, forward, cmd)
        self.low_since = None

    def update(self, t, forward, cmd):
        self.samples.append((t, forward, cmd))
        while self.samples and t - self.samples[0][0] > 0.5:
            self.samples.pop(0)
        if t < self.grace or len(self.samples) < 3:
            return False
        dt = self.samples[-1][0] - self.samples[0][0]
        if dt < 0.3:
            return False
        speed = abs(self.samples[-1][1] - self.samples[0][1]) / dt
        cmd_mean = sum(abs(c) for _, _, c in self.samples) / len(self.samples)
        if speed < self.ratio * cmd_mean:
            self.low_since = t if self.low_since is None else self.low_since
            return t - self.low_since >= self.window
        self.low_since = None
        return False


def emit(**fields):
    print(TAG + json.dumps(fields, ensure_ascii=False, sort_keys=True), flush=True)


def validate(args):
    if args.probe or args.prepare:
        return
    if not math.isfinite(args.distance) or not 0.01 <= abs(args.distance) <= MAX_DISTANCE_M:
        raise ValueError("distance must be 0.01..%.2f m in magnitude" % MAX_DISTANCE_M)
    if not math.isfinite(args.speed) or not MIN_SPEED_MPS <= args.speed <= MAX_SPEED_MPS:
        raise ValueError("speed must be %.2f..%.2f m/s" % (MIN_SPEED_MPS, MAX_SPEED_MPS))
    yaw_cap = 12.0 if args.hold_heading else 5.0   # 방향 유지 보정이 있는 운반 이동만 넓게
    if not 0.1 <= args.max_yaw_deg <= yaw_cap or not 0.005 <= args.max_lateral_m <= 0.08:
        raise ValueError("yaw/lateral limits out of range")


NAV2_IDLE_NODES = ("behavior_server", "velocity_smoother", "controller_server")


def prepare_gate(node, rclpy, args, msgs):
    """주행 준비를 한 번에(ros2 CLI 세 번 대신, 192호 2026-11-01 35 초 → 수 초): gate·OpenCR 노드 확인,
    /cmd_vel 다른 발행자(쉬는 Nav2 는 --allow-idle-nav2 일 때 허용)와 1 초 동안 메시지 0 확인, 소프트웨어 정지 해제."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Bool
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    seen = {"stopped": None, "gate": None, "cmd": 0}
    node.create_subscription(Bool, "/nav_safety/stopped", lambda m: seen.__setitem__("stopped", bool(m.data)), latched)
    node.create_subscription(Bool, "/nav_safety/ready", lambda m: seen.__setitem__("gate", bool(m.data)), latched)
    node.create_subscription(msgs["Twist"], "/cmd_vel", lambda m: seen.__setitem__("cmd", seen["cmd"] + 1), 10)
    stop_pub = node.create_publisher(Bool, "/nav_safety/stop", 10)
    t0 = time.monotonic()
    missing = ["nav_safety_gate", "packagu_opencr_bridge"]
    while time.monotonic() - t0 < DISCOVERY_SEC:
        rclpy.spin_once(node, timeout_sec=0.05)
        names = set(node.get_node_names())
        missing = [n for n in ("nav_safety_gate", "packagu_opencr_bridge") if n not in names]
        if not missing and seen["stopped"] is not None and time.monotonic() - t0 > 1.0:
            break
    if missing:
        return {"ok": False, "reason": "drive stack node missing: %s" % missing}
    allow = NAV2_IDLE_NODES if args.allow_idle_nav2 else ()
    pubs = [p.node_name for p in node.get_publishers_info_by_topic("/cmd_vel") if p.node_name != node.get_name()]
    others = [n for n in pubs if n not in allow and n != "nav_safety_gate"]
    if others:
        return {"ok": False, "reason": "/cmd_vel already has publishers %s" % others}
    if seen["cmd"]:
        return {"ok": False, "reason": "/cmd_vel active (%d msgs) — Nav2 goal or teleop running" % seen["cmd"]}
    t1 = time.monotonic()
    while time.monotonic() - t1 < 4.0 and seen["stopped"] is not False:
        stop_pub.publish(Bool(data=False))
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.05)
    if seen["stopped"] is not False:
        return {"ok": False, "reason": "gate software stop not released (stopped=%s)" % seen["stopped"]}
    return {"ok": True, "released_s": round(time.monotonic() - t1, 2), "total_s": round(time.monotonic() - t0, 2),
            "nav2_idle_publishers": pubs, "gate_ready": seen["gate"]}


def probe(node, rclpy, args, msgs):
    """움직이지 않고 센서·gate 상태를 잰다."""
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    seen = {"scan": [], "odom": [], "drive": None, "gate": None, "stopped": None, "status": None, "last_scan": None}
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_scan(m):
        seen["scan"].append(time.monotonic())
        seen["last_scan"] = m

    node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    node.create_subscription(msgs["Odometry"], "/odom", lambda m: seen["odom"].append(time.monotonic()), 20)
    node.create_subscription(Bool, "/drive/ready", lambda m: seen.__setitem__("drive", bool(m.data)), 10)
    node.create_subscription(Bool, "/nav_safety/ready", lambda m: seen.__setitem__("gate", bool(m.data)), latched)
    node.create_subscription(Bool, "/nav_safety/stopped", lambda m: seen.__setitem__("stopped", bool(m.data)), latched)
    node.create_subscription(String, "/nav_safety/status", lambda m: seen.__setitem__("status", m.data), latched)
    seen["cmd_vel_msgs"] = 0
    node.create_subscription(msgs["Twist"], "/cmd_vel", lambda m: seen.__setitem__("cmd_vel_msgs", seen["cmd_vel_msgs"] + 1), 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.probe_seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    now = time.monotonic()
    out = {"ok": True, "probe_s": round(now - t0, 2)}
    for key in ("scan", "odom"):
        ts = [t for t in seen[key] if t >= t0 + 0.5]   # 발견 지연 뒤 구간만
        span = (ts[-1] - ts[0]) if len(ts) > 1 else 0.0
        out[key + "_hz"] = round((len(ts) - 1) / span, 2) if span > 0 else 0.0
        out[key + "_age_s"] = round(now - seen[key][-1], 3) if seen[key] else None
    out.update(drive_ready=seen["drive"], gate_ready=seen["gate"], software_stop=seen["stopped"],
               gate_status=seen["status"])
    pubs = node.get_publishers_info_by_topic("/cmd_vel")
    out["cmd_vel_publishers"] = [p.node_name for p in pubs if p.node_name != node.get_name()]
    out["cmd_vel_msgs"] = seen["cmd_vel_msgs"]   # 점검 동안 남이 보낸 /cmd_vel 수(쉬는 Nav2 는 0)
    out["rear"] = rear_clearance(node, rclpy, seen["last_scan"])
    return out


def rear_clearance(node, rclpy, msg):
    """로봇 뒤(base_link x < -0.15, |y| < 0.35) 띠에서 가장 가까운 라이다 점까지 base_link 기준 거리."""
    if msg is None:
        return {"nearest_m": None, "reason": "no scan"}
    try:
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener
        buf = Buffer()
        TransformListener(buf, node)
        deadline = time.monotonic() + 3.0
        tr = None
        while time.monotonic() < deadline and tr is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                tr = buf.lookup_transform("base_link", msg.header.frame_id, Time()).transform
            except Exception:  # noqa: BLE001 — TF 가 아직 안 옴
                tr = None
        if tr is None:
            return {"nearest_m": None, "reason": "no base_link<-laser TF"}
        q = tr.rotation
        if abs(q.x) + abs(q.y) > 0.001:
            return {"nearest_m": None, "reason": "lidar not level"}
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        rear = []
        for i, d in enumerate(msg.ranges):
            if not math.isfinite(d) or not msg.range_min < d < msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment + yaw
            x = tr.translation.x + d * math.cos(a)
            y = tr.translation.y + d * math.sin(a)
            if x < -0.15 and abs(y) < 0.35:
                rear.append(-x)
        return {"nearest_m": round(min(rear), 3) if rear else None, "points": len(rear),
                "reason": None if rear else "no rear points (open or out of range)"}
    except Exception as exc:  # noqa: BLE001 — 점검 실패는 결과로 보고
        return {"nearest_m": None, "reason": "rear check error: %s" % exc}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--distance", type=float, default=0.0, help="+ 전진 / - 후진 (m, 시작 heading 방향 실제 odom)")
    ap.add_argument("--speed", type=float, default=0.03)
    ap.add_argument("--max-yaw-deg", type=float, default=1.5, help="이동 중 odom yaw 변화 한도")
    ap.add_argument("--max-lateral-m", type=float, default=0.02, help="이동 중 시작 heading 기준 좌우 이탈 한도")
    ap.add_argument("--settle-s", type=float, default=0.8, help="정지 명령 뒤 굴러간 거리까지 재는 시간")
    ap.add_argument("--timeout", type=float, default=0.0, help="0 이면 거리/속도 x2 + 6 s")
    ap.add_argument("--probe", action="store_true", help="움직이지 않고 센서·gate·후방 점검")
    ap.add_argument("--probe-seconds", type=float, default=4.0)
    ap.add_argument("--prepare", action="store_true", help="움직이지 않고 주행 준비 점검 + gate 소프트웨어 정지 해제")
    ap.add_argument("--allow-idle-nav2", action="store_true", help="쉬는 Nav2 의 /cmd_vel 발행자 허용")
    ap.add_argument("--hold-heading", action="store_true", help="운반 이동: 출발 방향 유지 회전 보정(삽입에는 쓰지 않음)")
    args = ap.parse_args(argv)
    try:
        validate(args)
    except ValueError as exc:
        emit(ok=False, reason=str(exc))
        return 2

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Bool

    rclpy.init()
    node = rclpy.create_node("packagu_pickup_straight_odom")
    if args.prepare:
        try:
            emit(**prepare_gate(node, rclpy, args, {"Twist": Twist}))
            return 0
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    if args.probe:
        try:
            emit(**probe(node, rclpy, args, {"Odometry": Odometry, "Twist": Twist}))
            return 0
        finally:
            node.destroy_node()
            rclpy.try_shutdown()

    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    latest = {"pose": None, "at": 0.0, "gate": None}

    def on_odom(m):
        latest["pose"] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m))
        latest["at"] = time.monotonic()

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Odometry, "/odom", on_odom, 20)
    node.create_subscription(Bool, "/nav_safety/ready", lambda m: latest.__setitem__("gate", bool(m.data)), latched)
    sign = 1.0 if args.distance > 0 else -1.0
    target = abs(args.distance)
    timeout = args.timeout or target / args.speed * 2.0 + 6.0
    res = {"ok": False, "target_m": round(args.distance, 4), "speed": args.speed}
    start, rel, peak = None, (0.0, 0.0, 0.0), {"lat": 0.0, "yaw": 0.0}
    stall = Stall()

    def send(v, w=0.0):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        pub.publish(t)

    began = time.monotonic()
    try:
        # Jetson 부하가 높으면 새 노드의 DDS 탐색이 4 초를 넘는다(192호 2026-11-01) → 10 초
        deadline = time.monotonic() + DISCOVERY_SEC
        while time.monotonic() < deadline and (latest["pose"] is None or latest["gate"] is None
                                               or pub.get_subscription_count() < 1):
            rclpy.spin_once(node, timeout_sec=0.05)
        res["discovery_s"] = round(DISCOVERY_SEC - max(0.0, deadline - time.monotonic()), 2)
        if latest["pose"] is None:
            res["reason"] = "/odom not received within %.0f s" % DISCOVERY_SEC
            return 1
        if pub.get_subscription_count() < 1:
            res["reason"] = "/cmd_vel has no subscriber (nav_safety_gate missing)"
            return 1
        # gate 가 잠깐 준비 안 됨(OpenCR drive not ready·odom stale 순간, Jetson 부하 높을 때)이면 0 명령을 보내며
        # 최대 GATE_WAIT_SEC 기다린다(192호 2026-11-01: 1.5 초 뒤 회복했는데 바로 포기함). 0 명령은 움직이지 않는다.
        wait_end = time.monotonic() + GATE_WAIT_SEC
        while latest["gate"] is not True and time.monotonic() < wait_end:
            send(0.0)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.02)
        res["gate_wait_s"] = round(GATE_WAIT_SEC - max(0.0, wait_end - time.monotonic()), 2) if latest["gate"] is True else None
        if latest["gate"] is not True:
            res["reason"] = "gate not ready (/nav_safety/ready=%s) after %.0f s" % (latest["gate"], GATE_WAIT_SEC)
            return 1
        start = latest["pose"]
        began = time.monotonic()
        drop_since = None
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now - latest["at"] > ODOM_STALE_SEC:
                res["reason"] = "/odom stale during move"
                return 1
            if latest["gate"] is False:
                # OpenCR 피드백 간격이 잠깐 벌어지면(0.3 초) gate 가 수십 ms 떨어진다(192호 2026-11-01). gate 가 출력을 0 으로
                # 막으니 0 명령으로 기다렸다가 GATE_DROP_TOL 안에 돌아오면 이어 간다. 그보다 길면 정지.
                drop_since = drop_since if drop_since is not None else now
                if now - drop_since > GATE_DROP_TOL:
                    res["reason"] = "gate dropped ready during move for > %.1f s (sensor stale or stop)" % GATE_DROP_TOL
                    return 1
                res["gate_drops"] = res.get("gate_drops", 0) + (1 if now - drop_since < 0.03 else 0)
                send(0.0)
                continue
            drop_since = None
            rel = relative(start, latest["pose"])
            fwd = rel[0] * sign
            peak["lat"] = max(peak["lat"], abs(rel[1]))
            peak["yaw"] = max(peak["yaw"], abs(math.degrees(rel[2])))
            if abs(math.degrees(rel[2])) > args.max_yaw_deg:
                res["reason"] = "yaw changed %.2f deg (> %.2f) while driving straight" % (math.degrees(rel[2]), args.max_yaw_deg)
                return 1
            if abs(rel[1]) > args.max_lateral_m:
                res["reason"] = "lateral drift %.3f m (> %.3f) while driving straight" % (rel[1], args.max_lateral_m)
                return 1
            remaining = target - fwd
            if remaining <= DONE_TOL_M:
                res.update(ok=True, reason="distance reached")
                return 0
            if now - began > timeout:
                res["reason"] = "timeout %.1f s (blocked or gate zeroed output)" % timeout
                return 1
            v = plan_speed(now - began, remaining, args.speed)
            if stall.update(now - began, fwd, v):
                res["reason"] = "stall: odom speed below 30%% of command for %.1f s (blocked or slipping)" % stall.window
                return 1
            w = 0.0
            if args.hold_heading:
                # 운반 이동 전용: 출발 방향 유지(팔레트를 들면 출발 때 왼쪽으로 틀어짐, 192호 2026-11-01). 삽입은 회전 0 그대로
                w = max(-HOLD_MAX_W, min(HOLD_MAX_W, -HOLD_K * rel[2]))
            send(sign * v, w)
    except KeyboardInterrupt:
        res["reason"] = "interrupted by operator"
        return 130
    finally:
        settle_end = time.monotonic() + max(0.3, args.settle_s)
        while time.monotonic() < settle_end:
            send(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        if start is not None and latest["pose"] is not None:
            rel = relative(start, latest["pose"])
            res.update(start={"x": round(start[0], 4), "y": round(start[1], 4), "yaw_deg": round(math.degrees(start[2]), 3)},
                       end={"x": round(latest["pose"][0], 4), "y": round(latest["pose"][1], 4),
                            "yaw_deg": round(math.degrees(latest["pose"][2]), 3)},
                       forward_m=round(rel[0], 4), lateral_m=round(rel[1], 4),
                       yaw_change_deg=round(math.degrees(rel[2]), 3),
                       peak_lateral_m=round(peak["lat"], 4), peak_yaw_deg=round(peak["yaw"], 3),
                       odom_age_s=round(time.monotonic() - latest["at"], 3))
        res["duration_s"] = round(time.monotonic() - began, 2)
        emit(**res)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
