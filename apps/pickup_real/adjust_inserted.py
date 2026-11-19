#!/usr/bin/env python3
"""삽입된 고정 포크를 빼고 상대 좌우 보정 후 같은 깊이로 재삽입. 기본은 계획 출력."""
import argparse
import json
import math
from pathlib import Path
import shlex
import time

import yaml

from geometry import plan_moves, simulate_moves
from pickup_real import RealBase, STATE_FILE, Stop

REAR_PROBE = r'''
import rclpy,time,math,json
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener
rclpy.init();n=Node("pickup_adjust_rear_check");buf=Buffer();listener=TransformListener(buf,n);latest=[]
sub=n.create_subscription(LaserScan,"/scan",lambda m:latest.append((time.monotonic(),m)),qos_profile_sensor_data)
end=time.monotonic()+5
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
if not latest or time.monotonic()-latest[-1][0]>.5:raise RuntimeError("no fresh rear scan")
msg=latest[-1][1];tr=buf.lookup_transform("base_link",msg.header.frame_id,Time()).transform;q=tr.rotation
if abs(q.x)+abs(q.y)>.001:raise RuntimeError("non-level lidar transform")
yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));rear=[]
for i,d in enumerate(msg.ranges):
 if not math.isfinite(d) or not msg.range_min<d<msg.range_max:continue
 a=msg.angle_min+i*msg.angle_increment+yaw;x=tr.translation.x+d*math.cos(a);y=tr.translation.y+d*math.sin(a)
 if x<-.15 and abs(y)<.35:rear.append(-x)
print("REAR_RESULT "+json.dumps({"nearest_m":min(rear) if rear else None,"points":len(rear)}))
n.destroy_node();rclpy.shutdown()
'''


def adjust(base, cfg, right_m, log):
    if not math.isfinite(right_m) or not .005 <= abs(right_m) <= .10:
        raise Stop("relative shift must be 0.5..10 cm")
    withdraw = float(cfg["robot"]["fork_tip_forward_m"]) + .10
    beta = math.radians(12)
    required_rear = withdraw + abs(right_m) / math.tan(beta) + .35
    rc, text = base.stack.exec_capture("python3 -c " + shlex.quote(REAR_PROBE), 15)
    line = next((s.split("REAR_RESULT ", 1)[1] for s in text.splitlines() if "REAR_RESULT " in s), None)
    rear = json.loads(line) if rc == 0 and line else {}
    if rear.get("nearest_m") is None or rear["nearest_m"] < required_rear:
        raise Stop("rear corridor is not confirmed clear for %.2f m" % required_rear)
    log("rear_clear", required_m=required_rear, **rear)
    pose = (0., 0., 0.)
    clear_x = -(float(cfg["robot"]["fork_tip_forward_m"]) + .06)

    def move(kind, value):
        nonlocal pose
        if kind == "turn" and pose[0] > clear_x:
            raise Stop("forks are not fully withdrawn; turning blocked")
        log("adjust_move", action=kind, value=value)
        if kind == "turn":
            data = base.turn(value) or {}
            actual = math.radians(float(data.get("turned_deg", math.degrees(value))))
        else:
            data = base.straight(value, .03) or {}
            actual = math.copysign(float(data.get("traveled_m", abs(value))), value)
        pose = simulate_moves(pose, [(kind, actual)])
        log("adjust_pose", relative_pose=list(pose), feedback=data)

    move("straight", -withdraw)
    target = (pose[0], -right_m, 0.)
    tolerance = {"lateral": .008, "yaw": math.radians(1.5), "distance": .01}
    for _ in range(3):
        moves, why = plan_moves(pose, target, tolerance, beta, .30)
        if moves is None:
            raise Stop(why)
        if not moves:
            break
        for kind, value in moves:
            move(kind, value)
    else:
        raise Stop("relative side shift did not converge; staying withdrawn")
    forward = -pose[0] / math.cos(pose[2])
    if abs(pose[1] + forward * math.sin(pose[2]) + right_m) > .015 or not 0 < forward <= withdraw + .05:
        raise Stop("reinsert path uncertainty too large; staying withdrawn")
    move("straight", forward)
    return {"target_right_m": right_m, "estimated_right_m": -pose[1], "depth_change_m": pose[0],
            "heading_change_deg": math.degrees(pose[2])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--right-cm", required=True, type=float)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    cfg = yaml.safe_load(Path(__file__).with_name("config.yaml").read_text())
    if not args.execute:
        print(json.dumps({"right_cm": args.right_cm, "withdraw_m": cfg["robot"]["fork_tip_forward_m"] + .10}))
        return
    state = json.loads(STATE_FILE.read_text())
    if state.get("stage") != "inserted":
        raise Stop("operator-relative correction needs the inserted state")
    path = STATE_FILE.parent / ("adjust_inserted_%d.jsonl" % time.time())
    with path.open("w") as stream:
        def log(event, **fields):
            row = dict(fields, event=event, at=time.time())
            stream.write(json.dumps(row)+"\n");stream.flush();print(json.dumps(row), flush=True)
        base = RealBase(cfg, log)
        try:
            base.prepare()
            STATE_FILE.write_text(json.dumps(dict(state, stage="adjusting_inserted", prior_state=state)))
            result = adjust(base, cfg, args.right_cm / 100., log)
            STATE_FILE.write_text(json.dumps(dict(state, stage="inserted", at=time.time(), adjustment=result)))
            log("ADJUST_RESULT", ok=True, **result)
        finally:
            base.stop()


if __name__ == "__main__":
    main()
