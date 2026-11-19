#!/usr/bin/env python3
"""택배 집기 실기 시험용 제자리 회전 (ROS 컨테이너 안에서 실행).

/odom yaw 를 보며 /cmd_vel 로 제자리에서 돈다. 명령은 기존 경로
/cmd_vel → nav_safety_gate → /cmd_vel_safe → OpenCR 를 그대로 지난다(gate 각속도 상한 0.35 rad/s).
Nav2 goal 이 없을 때만 실행한다(호출하는 쪽이 보장). 끝나거나 실패하면 정지 명령을 여러 번 보낸다.

결과는 마지막 줄 `TURN_RESULT {json}` 으로 출력한다.
"""

import argparse
import json
import math
import sys
import time

MAX_ANGLE_DEG = 100.0
MAX_SPEED = 0.30      # rad/s, gate 상한 0.35 아래
MIN_SPEED = 0.10      # 이보다 느리면 정지 마찰로 안 도는 경우가 있다 [가정값]
ACCEL = 0.4           # rad/s^2
ODOM_STALE_SEC = 0.5
DONE_TOL = math.radians(0.4)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def plan_speed(elapsed, remaining, cruise):
    accel_limited = MIN_SPEED + ACCEL * elapsed
    decel_limited = max(MIN_SPEED, abs(remaining) * 1.5)
    return max(MIN_SPEED, min(cruise, accel_limited, decel_limited))


def emit(**fields):
    print("TURN_RESULT " + json.dumps(fields, ensure_ascii=False, sort_keys=True), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angle-deg", type=float, required=True, help="+ 왼쪽(반시계) / - 오른쪽")
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--timeout", type=float, default=0.0, help="0 이면 각도/속도 x2.5 + 5 s")
    args = parser.parse_args(argv)
    if not math.isfinite(args.angle_deg) or not 0.2 <= abs(args.angle_deg) <= MAX_ANGLE_DEG:
        emit(ok=False, reason="angle must be 0.2..%g deg in magnitude" % MAX_ANGLE_DEG)
        return 2
    if not math.isfinite(args.speed) or not MIN_SPEED <= args.speed <= MAX_SPEED:
        emit(ok=False, reason="speed must be %g..%g rad/s" % (MIN_SPEED, MAX_SPEED))
        return 2

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = rclpy.create_node("packagu_pickup_turn")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    latest = {"msg": None, "at": 0.0}

    def on_odom(msg):
        latest["msg"], latest["at"] = msg, time.monotonic()

    node.create_subscription(Odometry, "/odom", on_odom, 10)
    target = math.radians(args.angle_deg)
    timeout = args.timeout or abs(target) / args.speed * 2.5 + 5.0
    turned = 0.0
    last_yaw = None
    result = {"ok": False, "target_deg": round(args.angle_deg, 2)}

    def publish(w):
        twist = Twist()
        twist.angular.z = float(w)
        publisher.publish(twist)

    try:
        deadline = time.monotonic() + 10.0  # 부하 속 DDS 탐색(192호 2026-11-01)
        while time.monotonic() < deadline and (latest["msg"] is None or publisher.get_subscription_count() < 1):
            rclpy.spin_once(node, timeout_sec=0.05)
        if latest["msg"] is None:
            result["reason"] = "/odom not received within 10 s"
            return 1
        if publisher.get_subscription_count() < 1:
            result["reason"] = "/cmd_vel has no subscriber (nav_safety_gate missing)"
            return 1
        last_yaw = yaw_of(latest["msg"])
        began = time.monotonic()
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - latest["at"] > ODOM_STALE_SEC:
                result["reason"] = "/odom stale during turn"
                return 1
            yaw = yaw_of(latest["msg"])
            turned += wrap(yaw - last_yaw)
            last_yaw = yaw
            remaining = target - turned
            if abs(remaining) <= DONE_TOL or (remaining * target) < 0:
                result.update(ok=True, reason="angle reached")
                return 0
            if now - began > timeout:
                result["reason"] = "timeout %.1f s (gate stopped or wheels blocked?)" % timeout
                return 1
            publish(math.copysign(plan_speed(now - began, remaining, args.speed), remaining))
    except KeyboardInterrupt:
        result["reason"] = "interrupted by operator"
        return 130
    finally:
        for _ in range(10):
            publish(0.0)
            time.sleep(0.05)
        # 정지 후 관성으로 더 돈 양까지 포함한 최종 회전량
        settle_end = time.monotonic() + 0.4
        while time.monotonic() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if latest["msg"] is not None and last_yaw is not None:
                yaw = yaw_of(latest["msg"])
                turned += wrap(yaw - last_yaw)
                last_yaw = yaw
        result["turned_deg"] = round(math.degrees(turned), 2)
        emit(**result)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
