#!/usr/bin/env python3
"""Build local-only pickup assets using the existing encoder-odom robot generator."""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as E

import cv2
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import generate_worlds as existing
from geometry import rpy_matrix


def sub(node, tag, value=None, **attrs):
    child = E.SubElement(node, tag, **attrs)
    if value is not None:
        child.text = str(value)
    return child


def box(link, name, xyz, size, color, collision=True, yaw=0):
    visual = sub(link, 'visual', name=name)
    sub(visual, 'pose', ' '.join(map(str, [*xyz, 0, 0, yaw])))
    sub(sub(visual, 'geometry'), 'box')
    sub(visual.find('geometry/box'), 'size', ' '.join(map(str, size)))
    mat = sub(visual, 'material')
    sub(mat, 'ambient', color)
    sub(mat, 'diffuse', color)
    if collision:
        col = sub(link, 'collision', name=name+'_collision')
        sub(col, 'pose', ' '.join(map(str, [*xyz, 0, 0, yaw])))
        sub(sub(sub(col, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))
    return name+'_collision'


def ray(tree, key, conf):
    name = key+'_sensor'
    link = sub(tree, 'link', name=name)
    inertial = sub(link, 'inertial')
    sub(inertial, 'mass', value='0.001')
    sub(inertial, 'inertia', ixx='1e-7', iyy='1e-7', izz='1e-7', ixy='0', ixz='0', iyz='0')
    vis = sub(link, 'visual')
    sub(sub(vis, 'geometry'), 'box', size='.012 .035 .02')
    joint = sub(tree, 'joint', name=name+'_joint', type='fixed')
    sub(joint, 'parent', link='base_link')
    sub(joint, 'child', link=name)
    sub(joint, 'origin', xyz=' '.join(map(str, conf['mount_xyz'])), rpy=' '.join(map(str, conf['mount_rpy'])))
    gazebo = sub(tree, 'gazebo', reference=name)
    sub(gazebo, 'material', 'Gazebo/Blue')
    sensor = sub(gazebo, 'sensor', name=name, type='ray')
    sub(sensor, 'always_on', 'true')
    sub(sensor, 'update_rate', conf['rate_hz'])
    scan = sub(sub(sensor, 'ray'), 'scan')
    for direction in ('horizontal', 'vertical'):
        s = sub(scan, direction)
        for k, v in dict(samples=5 if direction == 'horizontal' else 3, resolution=1,
                         min_angle=-conf['fov']/2, max_angle=conf['fov']/2).items():
            sub(s, k, v)
    limits = sub(sensor.find('ray'), 'range')
    for k, v in dict(min=conf['range_min'], max=conf['range_max'], resolution=.001).items():
        sub(limits, k, v)
    # Seeded noise/dropout is added by the independent relay, not Gazebo's unseeded RNG.
    plugin = sub(sensor, 'plugin', name=name+'_driver', filename='libgazebo_ros_ray_sensor.so')
    ros = sub(plugin, 'ros')
    sub(ros, 'remapping', '~/out:=/pickup/raw/'+key)
    sub(plugin, 'output_type', 'sensor_msgs/Range')
    sub(plugin, 'radiation_type', 'ultrasound')
    sub(plugin, 'frame_name', name)


def build(c, case, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    yaml.safe_dump(c, (out/'effective_config.yaml').open('w'), allow_unicode=True)
    existing.OUT = out
    existing.robot(False, 'robot.urdf', odometry='encoder')
    tree = E.parse(out/'robot.urdf').getroot()
    r, p = c['robot'], c['pallet']
    origin = tree.find("joint[@name='lead_screw_base_joint']/origin")
    origin.set('xyz', f"{r['mast_x']} 0 0")
    tree.find("joint[@name='lift_joint']/origin").set('xyz', f"0 0 {r['fork_z']}")
    carrier = tree.find("link[@name='lead_screw_carrier']")
    for sign, name in ((1, 'left'), (-1, 'right')):
        for kind in ('visual', 'collision'):
            part = sub(carrier, kind, name='fork_'+name+'_'+kind)
            x = (r['fork_tip_x']+r['fork_root_x'])/2-r['mast_x']
            y = sign*(r['fork_width']-r['tine_width'])/2
            sub(part, 'origin', xyz=f'{x} {y} 0')
            sub(sub(part, 'geometry'), 'box', size=f"{r['fork_tip_x']-r['fork_root_x']} {r['tine_width']} {r['fork_thickness']}")
    # Body fixed camera. Gazebo optical image is real rendering, never synthesized from truth poses.
    cam = c['camera']
    camera_link = sub(tree, 'link', name='pickup_body_camera')
    housing = sub(camera_link, 'visual')
    sub(housing, 'origin', xyz='-.02 0 0')
    sub(sub(housing, 'geometry'), 'box', size='.03 .06 .03')
    inertial = sub(camera_link, 'inertial')
    sub(inertial, 'mass', value='.001')
    sub(inertial, 'inertia', ixx='1e-7', iyy='1e-7', izz='1e-7', ixy='0', ixz='0', iyz='0')
    joint = sub(tree, 'joint', name='pickup_camera_joint', type='fixed')
    sub(joint, 'parent', link='base_link')
    sub(joint, 'child', link='pickup_body_camera')
    sub(joint, 'origin', xyz=' '.join(map(str, cam['mount_xyz'])), rpy=' '.join(map(str, cam['mount_rpy'])))
    camera_gazebo = sub(tree, 'gazebo', reference='pickup_body_camera')
    sub(camera_gazebo, 'material', 'Gazebo/Black')
    sensor = sub(camera_gazebo, 'sensor', name='pickup_camera', type='camera')
    sub(sensor, 'always_on', 'true')
    sub(sensor, 'update_rate', cam['rate_hz'])
    camera = sub(sensor, 'camera')
    sub(camera, 'horizontal_fov', cam['hfov'])
    im = sub(camera, 'image')
    for k, v in dict(width=cam['width'], height=cam['height'], format='R8G8B8').items():
        sub(im, k, v)
    clip = sub(camera, 'clip')
    sub(clip, 'near', cam['range_min'])
    sub(clip, 'far', cam['range_max'])
    noise = sub(camera, 'noise')
    sub(noise, 'type', 'gaussian')
    sub(noise, 'mean', 0)
    sub(noise, 'stddev', cam['noise_stddev'])
    plugin = sub(sensor, 'plugin', name='pickup_camera_driver', filename='libgazebo_ros_camera.so')
    sub(plugin, 'camera_name', 'pickup/body_camera')
    sub(plugin, 'frame_name', 'pickup_body_camera')
    ray(tree, 'approach', c['front_range'])
    ray(tree, 'lift_limit', c['height_range'])
    E.indent(tree)
    E.ElementTree(tree).write(out/'robot.urdf', encoding='unicode')

    sdf = E.Element('sdf', version='1.6')
    world = sub(sdf, 'world', name='pickup')
    plugin = sub(world, 'plugin', name='gazebo_state', filename='libgazebo_ros_state.so')
    sub(sub(plugin, 'ros'), 'namespace', '/gazebo')
    sub(plugin, 'update_rate', 30)
    physics = sub(world, 'physics', name='default', type='ode')
    sub(physics, 'max_step_size', .001)
    sub(physics, 'real_time_update_rate', 1000) # Required baseline: 1x
    sub(physics, 'real_time_factor', 1)
    sub(sub(physics, 'ode'), 'solver')
    sub(physics.find('ode/solver'), 'iters', 80)
    scene = sub(world, 'scene')
    sub(scene, 'ambient', '.7 .7 .7 1')
    sub(scene, 'background', '.65 .7 .75 1')
    sub(scene, 'shadows', 'false')
    light = sub(world, 'light', name='sun', type='directional')
    sub(light, 'pose', '0 0 5 0 0 0')
    sub(light, 'diffuse', '.8 .8 .8 1')
    sub(light, 'direction', '-.1 .1 -1')
    floor = sub(world, 'model', name='ground')
    sub(floor, 'static', 'true')
    fl = sub(floor, 'link', name='ground')
    box(fl, 'floor', [0, 0, -.025], [12, 10, .05], '.65 .65 .65 1')
    # Pallet frame origin is pocket entry centre on the floor; +x is insertion direction.
    model = sub(world, 'model', name='pickup_parcel')
    sub(model, 'static', 'true') # kinematic attachment validation, not payload dynamics
    x, y, yaw = case['pallet_pose']
    sub(model, 'pose', f'{x} {y} 0 0 0 {yaw}')
    link = sub(model, 'link', name='pallet')
    collisions = []
    collisions.append(box(link, 'deck', [p['depth']/2, 0, p['pocket_height']+p['deck_thickness']/2],
                          [p['depth'], p['width'], p['deck_thickness']], '.15 .45 .25 1'))
    sw = (p['width']-p['pocket_width'])/2
    for sign in (-1, 1):
        collisions.append(box(link, 'stringer_'+str(sign), [p['depth']/2, sign*(p['width']-sw)/2, p['pocket_height']/2],
                              [p['depth'], sw, p['pocket_height']], '.15 .45 .25 1'))
    pd, pw, ph = p['parcel_size_m']
    parcel_base = p['pocket_height']+p['deck_thickness']
    parcel_top = parcel_base+ph
    collisions.append(box(link, 'parcel', [p['parcel_front_offset_m']+pd/2, 0, parcel_base+ph/2],
                          [pd, pw, ph], '.66 .49 .30 1'))
    mx, my, mz = p['marker_offset_xyz']
    box(link, 'marker_support', [p['depth']/2, .12, (mz+parcel_top)/2], [.01, .01, max(.01, mz-parcel_top)], '.2 .2 .2 1', False)
    box(link, 'marker_bracket', [p['depth']/4, .12, mz], [p['depth']/2, .01, .01], '.2 .2 .2 1', False)
    cell = p['marker_size']/6
    for facet in p['marker_facets']:
        marker_id, fy, angle = facet['id'], facet['y'], facet['yaw']
        marker = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), marker_id, 6)
        cv2.imwrite(str(out/f'marker_{marker_id}.png'), cv2.resize(marker, (300, 300), interpolation=cv2.INTER_NEAREST))
        rot = rpy_matrix([0, 0, angle])
        origin = [mx, my+fy, mz]
        board = rot @ [.001, 0, 0]+origin
        box(link, f'marker_board_{marker_id}', board, [.001, p['marker_size']+.02, p['marker_size']+.02], '1 1 1 1', False, angle)
        for row in range(6):
            for col in range(6):
                if marker[row, col] == 0:
                    xyz = rot @ [0, (2.5-col)*cell, (2.5-row)*cell]+origin
                    box(link, f'marker_{marker_id}_{row}_{col}', xyz,
                        [.0005, cell, cell], '0 0 0 1', False, angle)
    contact = sub(link, 'sensor', name='pickup_contacts', type='contact')
    sub(contact, 'always_on', 'true')
    sub(contact, 'update_rate', 50)
    detect = sub(contact, 'contact')
    for col in collisions:
        sub(detect, 'collision', col)
    plugin = sub(contact, 'plugin', name='pickup_contact_driver', filename='libgazebo_ros_bumper.so')
    sub(sub(plugin, 'ros'), 'remapping', 'bumper_states:=/pickup/contacts')
    sub(plugin, 'frame_name', 'world')
    gui = sub(world, 'gui')
    camera = sub(gui, 'camera', name='user_camera')
    sub(camera, 'pose', '2.4 -2.8 2.2 0 .48 2.2')
    E.indent(sdf)
    E.ElementTree(sdf).write(out/'pickup.world', encoding='unicode')
    return out/'pickup.world', out/'robot.urdf'
