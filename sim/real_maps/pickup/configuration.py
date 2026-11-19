"""Reject inconsistent simulation assumptions before starting any actuator."""
import math
import numpy as np
from geometry import marker_points, stop_range


def validate(c):
    r, p, u, k = c['robot'], c['pallet'], c['front_range'], c['control']
    # Preserve replay of snapshots made before these formerly fixed dimensions
    # were exposed in config.yaml. Current configurations list them explicitly.
    r.setdefault('tine_width', .035)
    p.setdefault('parcel_size_m', [p['depth'], .30, .40])
    p.setdefault('parcel_front_offset_m', 0.0)
    assert r['fork_root_x'] < r['fork_tip_x'], 'fork root must precede tip'
    assert 0 < r['tine_width'] < r['fork_width']/2
    length = r['fork_tip_x']-r['fork_root_x']
    assert len(p['parcel_size_m']) == 3 and min(p['parcel_size_m']) > 0
    centre = max(p['depth']/2, p['parcel_front_offset_m']+p['parcel_size_m'][0]/2)
    minimum, maximum = centre+p['insertion_end_margin'], length-p['insertion_end_margin']
    assert minimum <= p['insertion_depth']-p['insertion_tolerance']+1e-9, 'fork must reach past assumed load centre'
    assert p['insertion_depth']+p['insertion_tolerance'] <= maximum+1e-9, 'insertion exceeds usable fork length'
    assert r['fork_width']+2*p['clearance_margin'] < p['pocket_width'], 'pocket too narrow'
    assert .005 < r['fork_z']-r['fork_thickness']/2 < r['fork_z']+r['fork_thickness']/2 < p['pocket_height']-.005
    assert 0 < k['alignment_yaw'] < k['realign_yaw'] <= p['max_yaw']
    swept = k['realign_lateral']+p['insertion_depth']*math.tan(k['realign_yaw'])+r['fork_width']/2
    assert swept < p['pocket_width']/2-p['clearance_margin'], 'control tolerance exceeds pocket clearance'
    assert u['range_min'] < stop_range(c) < u['range_max'], 'stop range outside sensor valid range'
    assert r['fork_tip_x']-p['insertion_depth'] > r['body_front_x']+.02, 'body would hit pallet at insertion'
    for sensor in (c['front_range'], c['height_range']):
        assert 0 < sensor['range_min'] < sensor['range_max'] and sensor['rate_hz'] > 0
        assert sensor['noise_stddev'] >= 0 and 0 <= sensor['dropout_probability'] <= 1
        assert len(sensor['mount_xyz']) == len(sensor['mount_rpy']) == 3
    assert 0 < c['camera']['hfov'] < math.pi and c['camera']['rate_hz'] > 0
    assert c['camera']['noise_stddev'] >= 0 and 0 <= c['camera']['dropout_probability'] <= 1
    facets = p['marker_facets']
    assert len(facets) >= 2 and len({f['id'] for f in facets}) == len(facets), 'need distinct fixed marker faces'
    assert all(0 <= f['id'] < 50 for f in facets) and p['marker_size'] > 0
    corners = np.concatenate([marker_points(c, f) for f in facets])
    assert np.linalg.matrix_rank(corners-corners.mean(axis=0)) == 3, 'frontal yaw requires a noncoplanar fixture'
