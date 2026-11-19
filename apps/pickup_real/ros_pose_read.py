#!/usr/bin/env python3
"""컨테이너에서 새 /odom 한 장만 읽는다. 발행자·서비스·시리얼 없음."""
import json
import math
import time


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node("packagu_room192_pose_read")
    latest = {}
    def receive(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        stamp = m.header.stamp.sec + m.header.stamp.nanosec/1e9
        age = node.get_clock().now().nanoseconds/1e9 - stamp
        if 0 <= age <= .5:
            latest.update(pose=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))], age_s=age)
    node.create_subscription(Odometry,"/odom",receive,20)
    try:
        end=time.monotonic()+10
        while not latest and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
        print("ROOM192_POSE "+json.dumps(dict(ok=bool(latest),**latest)),flush=True)
        return 0 if latest else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=="__main__":
    raise SystemExit(main())
