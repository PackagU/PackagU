#!/usr/bin/env python3
"""Build -> isolated Gazebo -> sensor-controlled pickup -> verify -> logs -> cleanup."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from isolation import assert_isolated
assert_isolated()

import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml

from build_scene import build
from configuration import validate
from controller import Controller
from vision import Vision
from world_io import WorldIO


def clean_json(value):
    if isinstance(value, float):
        import math
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: clean_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [clean_json(v) for v in value]
    return value


def run_case(config, name, out, gui=False, hold=0):
    out.mkdir(parents=True, exist_ok=False)
    c = copy.deepcopy(config)
    os.environ['QT_X11_NO_MITSHM'] = '1'
    case = c['cases'][name]
    world, robot = build(c, case, out/'assets')
    processes, handles = [], []
    nodes, executor, controller, io = [], None, None, None

    def start(tag, cmd):
        log = (out/(tag+'.log')).open('w')
        handles.append(log)
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc

    result = dict(passed=False, reason='STARTUP_INCOMPLETE', case=name)
    wall_start = time.monotonic()
    try:
        rclpy.init()
        io, vision = WorldIO(c, case, out), Vision(c, out/'camera', gui)
        nodes = [io, vision]
        executor = MultiThreadedExecutor(num_threads=4)
        for n in nodes:
            executor.add_node(n)
        server = start('gazebo', ['gzserver', '--verbose', '--seed', str(c['seed']), str(world),
                                  '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'])
        spawn = start('spawn', ['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity', 'packagu_sim',
                                '-file', str(robot), '-z', '.005', '-timeout', '60'])
        if gui:
            gui_ini = Path.home()/'.gazebo/gui.ini'
            gui_ini.parent.mkdir(parents=True, exist_ok=True)
            gui_ini.write_text('[geometry]\nx=20\ny=50\nwidth=1260\nheight=820\n')
            start('gui', ['gzclient', '--verbose'])
            start('camera_gui', [sys.executable, str(HERE/'viewer.py'), str(out/'camera/latest.png')])
        deadline = time.monotonic()+75
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if server.poll() is not None or spawn.poll() not in (None, 0):
                raise RuntimeError('GAZEBO_OR_SPAWN_EXITED: see gazebo.log/spawn.log')
            if io.truth and io.parcel and len(io.raw) == 2 and vision.latest:
                break
        else:
            raise RuntimeError(f'SENSOR_STARTUP_TIMEOUT: truth={io.truth is not None} parcel={io.parcel} raw={list(io.raw)} image_detected={vision.latest is not None}')
        controller = Controller(c, out)
        nodes.append(controller)
        executor.add_node(controller)
        start_sim = io.now()
        active_wall_start = time.monotonic()
        print(f'CASE_START {name} sim={start_sim:.3f}', flush=True)
        deadline = time.monotonic()+c['control']['total_timeout']*2+20
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if server.poll() is not None:
                raise RuntimeError('GAZEBO_EXITED')
            if controller.phase in ('DONE', 'FAILED'):
                # Observe actual standstill and load state for 1.2 seconds after the controller finishes.
                if io.now()-controller.terminal_t >= 1.2:
                    break
        else:
            controller.fail('WALL_CLOCK_TIMEOUT')
        result = io.result()
        result.update(case=name, duration_sim_s=io.now()-start_sim, duration_wall_s=time.monotonic()-wall_start,
                      achieved_real_time_factor=(io.now()-start_sim)/(time.monotonic()-active_wall_start),
                      retries=controller.retry, config_sha256=hashlib.sha256((out/'assets/effective_config.yaml').read_bytes()).hexdigest())
        print('CASE_RESULT '+json.dumps(clean_json(result)), flush=True)
        if gui and hold:
            until = time.monotonic()+hold
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.02)
    except Exception as exc:
        result.update(reason=str(exc))
        import traceback
        traceback.print_exc()
    finally:
        if controller:
            controller.move()
            controller.hold_lift()
        for node in nodes:
            for timer in node.timers:
                timer.cancel()
        if executor:
            # Humble's Executor.shutdown does not join its Python worker pool.
            # Drain queued callbacks before destroying the ROS handles they refer to.
            executor._executor.shutdown(wait=True)
            executor.shutdown(timeout_sec=10)
        if io and io.pending and not io.pending.done():
            io.set_client.remove_pending_request(io.pending)
            io.pending.cancel()
        for node in nodes:
            node.destroy_node()
        if controller:
            controller.trace.close()
        if io:
            io.rows_file.close()
        rclpy.try_shutdown()
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in reversed(processes):
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3)
        for log in handles:
            log.close()
        result = clean_json(result)
        (out/'summary.json').write_text(json.dumps(result, indent=2, ensure_ascii=False))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(HERE/'config.yaml'))
    parser.add_argument('--cases', default='all')
    parser.add_argument('--name', default=time.strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--hold-seconds', type=float, default=0)
    args = parser.parse_args()
    if '/' in args.name or args.name in ('.', '..') or args.repeats < 1:
        parser.error('name must be a directory name; repeats must be >=1')
    c = yaml.safe_load(Path(args.config).read_text())
    if args.seed is not None:
        c['seed'] = args.seed
    validate(c)
    cases = list(c['cases']) if args.cases == 'all' else args.cases.split(',')
    if any(k not in c['cases'] for k in cases):
        parser.error('unknown case')
    out = HERE.parents[2]/'logs/parcel_pickup'/args.name
    out.mkdir(parents=True, exist_ok=False)
    inputs = list(HERE.glob('*.py'))+[Path(args.config).resolve(), HERE.parent/'generate_worlds.py',
             HERE.parent/'world_params.yaml', HERE.parents[2]/'src/common_pkg/urdf/delivery_robot.urdf.xacro']
    (out/'provenance.json').write_text(json.dumps({str(p.relative_to(HERE.parents[2])) if p.is_relative_to(HERE.parents[2]) else p.name:
        hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}, indent=2))
    results = []
    try:
        for repeat in range(args.repeats):
            for case in cases:
                results.append(run_case(c, case, out/f'{repeat+1:02d}_{case}', args.gui, args.hold_seconds))
    finally:
        (out/'suite.json').write_text(json.dumps(results, indent=2, ensure_ascii=False))
        import csv
        with (out/'suite.csv').open('w') as f:
            writer = csv.writer(f)
            writer.writerow(['case', 'passed', 'phase', 'reason', 'lateral_error_m', 'angle_error_rad', 'stop_distance_m',
                             'insertion_depth_m', 'collision', 'max_lift_m', 'stop_latency_s'])
            for row in results:
                a = row.get('attach') or {}
                g = a.get('geometry') or row.get('final_geometry') or {}
                writer.writerow([row['case'], row['passed'], row.get('phase'), row.get('reason'), g.get('lateral_error_m'),
                                 g.get('angle_error_rad'), a.get('stop_range_m', row.get('stop_distance_m')), g.get('insertion_depth_m'), row.get('collision'),
                                 row.get('max_lift_m'), row.get('stop_latency_s')])
    print(f'SUITE_RESULT {sum(bool(r["passed"]) for r in results)}/{len(results)} {out}', flush=True)
    return 0 if len(results) == len(cases)*args.repeats and all(r['passed'] for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
