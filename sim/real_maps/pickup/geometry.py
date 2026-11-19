"""Pure geometry shared by sensor simulation, controller and independent evaluator.

No Gazebo access. The controller calls only the observation-based transforms.
"""
import math
import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rotate(xy, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([c*xy[0]-s*xy[1], s*xy[0]+c*xy[1]])


def compose(p, q):
    xy = np.array(p[:2])+rotate(q[:2], p[2])
    return [float(xy[0]), float(xy[1]), wrap(p[2]+q[2])]


def relative(p, q):
    xy = rotate(np.array(q[:2])-p[:2], -p[2])
    return [float(xy[0]), float(xy[1]), wrap(q[2]-p[2])]


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def marker_points(c, facet):
    """Four ArUco corners in pocket coordinates; fixed noncoplanar target fixture."""
    s = c['pallet']['marker_size']/2
    corners = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
    axes = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)
    offset = np.array(c['pallet']['marker_offset_xyz'])+[0, facet['y'], 0]
    return (rpy_matrix([0, 0, facet['yaw']]) @ axes @ corners.T).T+offset


def stop_range(c):
    """Box front is entry plane. d = tip_x - required_insertion - sensor_x.

    At yaw=0 the desired entry plane is tip_x-insertion relative to base.
    Height sensor's 0.10 m threshold is deliberately absent.
    """
    return predicted_range(c, [c['robot']['fork_tip_x']-c['pallet']['insertion_depth'], 0, 0])


def predicted_range(c, entry_base):
    sensor = c['front_range']
    n = np.array([math.cos(entry_base[2]), math.sin(entry_base[2]), 0])
    direction = rpy_matrix(sensor['mount_rpy'])[:, 0]
    denominator = float(n @ direction)
    if denominator <= .2:
        return math.inf
    plane = np.array([entry_base[0], entry_base[1], 0])+n*c['pallet']['parcel_front_offset_m']
    delta = plane-sensor['mount_xyz']
    return float(n @ delta)/denominator


def engagement(c, base, entry, lift=0.0):
    """Evaluator only: fork segment in pocket frame, conservative swept envelope."""
    r, p = c['robot'], c['pallet']
    root = compose(base, [r['fork_root_x'], 0, 0])
    tip = compose(base, [r['fork_tip_x'], 0, 0])
    local_tip, local_root = relative(entry, tip), relative(entry, root)
    yaw = wrap(base[2]-entry[2])
    half = r['fork_width']/2*abs(math.cos(yaw))
    # Only the segment actually inside the pocket must fit between the stringers.
    y_entry = local_tip[1]-max(0, local_tip[0])*math.tan(yaw)
    lateral_envelope = max(abs(local_tip[1]), abs(y_entry))+half
    lateral_limit = p['pocket_width']/2-p['clearance_margin']
    z = r['fork_z']+lift
    vertical_ok = z-r['fork_thickness']/2 >= .005 and z+r['fork_thickness']/2 <= p['pocket_height']-.005
    ok = (abs(yaw) <= p['max_yaw'] and lateral_envelope <= lateral_limit and vertical_ok
          and abs(local_tip[0]-p['insertion_depth']) <= p['insertion_tolerance'])
    return dict(ok=bool(ok), lateral_error_m=local_tip[1], angle_error_rad=yaw,
                insertion_depth_m=local_tip[0], lateral_envelope_m=lateral_envelope,
                lateral_limit_m=lateral_limit, vertical_ok=bool(vertical_ok), fork_root_depth_m=local_root[0])
