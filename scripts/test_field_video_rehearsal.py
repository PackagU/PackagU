#!/usr/bin/env python3
"""촬영 리허설 진행기 오프라인 시험 (ROS·장치 없이 가짜 연결로)."""

import json
import queue
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from field_video import backends, runner  # noqa: E402
from field_video import ros_map_switch, ros_nav_idle, ros_straight_move  # noqa: E402

SCENARIO = HERE / "field_video" / "scenario.json"
REGISTRY = HERE.parent / "src" / "slam_pkg" / "maps" / "field" / "waypoints.json"


def make(answers=None, failures=None, ultrasonic=None, nav_restart=False, auto=True, ros_arm=False, **field_kwargs):
    scenario = runner.load_scenario(SCENARIO)
    tmp = tempfile.mkdtemp(prefix="fv_test_")
    out = []
    field = backends.FakeField(failures, **field_kwargs)
    web = backends.FakeWeb(failures, ros_arm=ros_arm)
    lift = backends.FakeLift(failures)
    ui = runner.ConsoleUI(answers=answers, auto=auto, out=out.append)
    log = runner.RunLog(tmp, "test")
    us = ultrasonic if ultrasonic is not None else backends.FakeUltrasonic()
    rehearsal = runner.Rehearsal(scenario, field, web, lift, ui, log, nav_restart=nav_restart,
                                 allow_preview_arm=True, sleep=lambda _s: None, ultrasonic=us)
    return rehearsal, scenario, field, web, lift, out


def run(rehearsal, scenario, **select):
    steps = scenario["steps"]
    return rehearsal.run(runner.select_steps(steps, **select), steps)


def statuses(rehearsal):
    return {r["id"]: r["status"] for r in rehearsal.results}


class FullRunTest(unittest.TestCase):
    def test_full_dry_run_is_one_take(self):
        rehearsal, scenario, field, web, lift, _ = make()
        meta = run(rehearsal, scenario)
        self.assertTrue(meta["verdict"].startswith("한 번에"), meta["verdict"])
        self.assertEqual(len(rehearsal.results), len(scenario["steps"]))
        self.assertEqual(set(statuses(rehearsal).values()), {"OK"})
        presses = [c for c in web.calls if c[0] == "mission"]
        self.assertEqual([c[1:] for c in presses],
                         [("hall", "up"), ("car", "up"), ("hall", "down"), ("car", "down")])
        self.assertEqual([c for c in lift.calls if c[0] == "move"], [("move", "up"), ("move", "down")])
        self.assertIn(("load_map", "/ros2_ws/maps/field/f2/pinned.yaml"), field.calls)
        self.assertIn(("load_map", "/ros2_ws/maps/field/f1/pinned.yaml"), field.calls)
        self.assertIn(("record_start", "rehearsal"), field.calls)
        self.assertEqual(field.calls[-1], ("record_stop",))
        summary = (rehearsal.log.dir / "summary.md").read_text(encoding="utf-8")
        self.assertIn("S26", summary)

    def test_motion_is_always_wrapped_by_cancel_resume_stop(self):
        rehearsal, scenario, field, *_ = make()
        run(rehearsal, scenario)
        calls = field.calls
        for index, call in enumerate(calls):
            if call[0] in ("goal", "teleop", "straight", "straight_until"):
                self.assertEqual(calls[index - 1], ("resume",), call)
                self.assertEqual(calls[index - 2], ("idle",), call)
                self.assertEqual(calls[index + 1], ("stop",), call)
            if call[0] == "teleop":
                self.assertEqual(call[1], "released")

    def test_goal_never_sent_on_wrong_floor_map(self):
        rehearsal, scenario, field, *_ = make()
        run(rehearsal, scenario)
        loaded = "F1"
        for call in field.calls:
            if call[0] == "load_map":
                loaded = call[1].split("/")[-2].upper()
            if call[0] == "goal":
                self.assertEqual(call[2], loaded)


class FailurePathTest(unittest.TestCase):
    def test_goal_failure_is_retried_automatically_once(self):
        rehearsal, scenario, field, *_ = make(failures={"goal:f1_locker": 1})
        meta = run(rehearsal, scenario)
        self.assertEqual(meta["retries"], 1)  # 자동 재시도도 판정에 들어간다 (Codex #11)
        self.assertFalse(meta["verdict"].startswith("한 번에"))
        locker = next(r for r in rehearsal.results if r["id"] == "S02")
        self.assertIn("자동 재시도 1회", locker["detail"])
        self.assertIn(("clear",), field.calls)

    def test_goal_failing_twice_shows_menu_and_retry_works(self):
        rehearsal, scenario, field, *_ = make(failures={"goal:f1_locker": 2}, answers=["", "", "r"])
        meta = run(rehearsal, scenario, only=["S02"])
        result = rehearsal.results[0]
        self.assertEqual((result["status"], result["attempts"]), ("OK", 2))
        self.assertEqual(meta["retries"], 2)
        self.assertFalse(meta["verdict"].startswith("한 번에"))

    def test_lamp_not_lit_retries_press(self):
        rehearsal, scenario, _, web, *_ = make(answers=["", "r", "r", ""])
        run(rehearsal, scenario, only=["S10"])
        self.assertEqual(rehearsal.results[0]["attempts"], 2)
        self.assertEqual(len([c for c in web.calls if c[0] == "press"]), 2)

    def test_arm_error_then_person_does_it_is_manual(self):
        rehearsal, scenario, _, web, *_ = make(failures={"press:4": 1}, answers=["", "s"])
        meta = run(rehearsal, scenario, only=["S10"])
        self.assertEqual(rehearsal.results[0]["status"], "MANUAL")
        self.assertEqual(meta["manual"], 1)
        self.assertIn(("stop",), web.calls)

    def test_button_not_recognized_times_out(self):
        rehearsal, scenario, *_ = make(answers=["", "s"])
        rehearsal.settings = dict(rehearsal.settings, button_ready_timeout_sec=0.05)
        rehearsal.web.fail.failures["ready:UP"] = 10 ** 6
        run(rehearsal, scenario, only=["S07"])
        self.assertEqual(rehearsal.results[0]["status"], "MANUAL")
        self.assertIn("not stably recognized", rehearsal.results[0]["detail"])

    def test_quit_stops_robot_and_still_writes_summary(self):
        rehearsal, scenario, field, *_ = make(answers=["", "q"], auto=False)
        meta = run(rehearsal, scenario, only=["S02"])
        self.assertTrue(meta["verdict"].startswith("중단"))
        self.assertNotIn("goal", [c[0] for c in field.calls])
        self.assertEqual(field.calls[-1], ("record_stop",))
        self.assertTrue((rehearsal.log.dir / "steps.csv").exists())

    def test_optional_teleop_can_be_skipped(self):
        rehearsal, scenario, field, *_ = make(answers=["", "n"])
        run(rehearsal, scenario, only=["S15"])
        self.assertEqual(rehearsal.results[0]["status"], "SKIPPED")
        self.assertNotIn("teleop", [c[0] for c in field.calls])

    def test_adjust_then_start_counts_adjust(self):
        rehearsal, scenario, field, *_ = make(answers=["t", ""])
        run(rehearsal, scenario, only=["S07"])
        self.assertEqual(rehearsal.results[0]["adjusts"], 1)
        self.assertIn(("teleop", "released"), field.calls)


class CodexReviewRegressionTest(unittest.TestCase):
    """2026-10-21 Codex 검토 지적 재발 방지."""

    def test_idle_check_failure_blocks_motion(self):  # 1
        rehearsal, scenario, field, *_ = make(failures={"idle": 1}, answers=["", "", "q"])
        run(rehearsal, scenario, only=["S02"])
        names = [c[0] for c in field.calls]
        self.assertNotIn("goal", names)
        first_idle = names.index("idle")
        self.assertNotIn("resume", names[:first_idle + 1])

    def test_prep_stops_and_checks_idle_before_map_or_pose(self):  # 2
        rehearsal, scenario, field, *_ = make(loaded="F1")
        run(rehearsal, scenario, start="S14", end="S14")
        names = [c[0] for c in field.calls]
        self.assertLess(names.index("stop"), names.index("load_map"))
        self.assertLess(names.index("idle"), names.index("load_map"))
        self.assertLess(names.index("idle"), names.index("pose"))

    def test_failed_teleop_blocks_auto_map_switch(self):  # 4
        rehearsal, scenario, field, *_ = make(teleop_rc=127, answers=["", "q"])
        meta = run(rehearsal, scenario, start="S12", end="S13")
        self.assertTrue(meta["verdict"].startswith("중단"))
        self.assertNotIn("load_map", [c[0] for c in field.calls])

    def test_operator_must_confirm_exit_marker(self):  # 4
        rehearsal, scenario, field, *_ = make(answers=["", "", "r", "r", ""])
        run(rehearsal, scenario, start="S12", end="S13")
        teleops = [c for c in field.calls if c[0] == "teleop"]
        self.assertEqual(len(teleops), 2)
        self.assertEqual(rehearsal.results[0]["attempts"], 2)

    def test_lift_failure_waits_for_firmware_end(self):  # 5 — lift_uno 펌웨어는 정지 명령이 없다
        rehearsal, scenario, _, _, lift, _ = make(failures={"move:up": 1}, answers=["", "", "q"])
        run(rehearsal, scenario, only=["S04"])
        self.assertIn(("settle",), lift.calls)
        self.assertFalse(lift.moving)

    def test_ros_arm_with_nav_off_aborts_with_order(self):  # 8
        rehearsal, scenario, field, *_ = make(ros_arm=True, nav_running=False)
        meta = run(rehearsal, scenario)
        self.assertIn("--ros-arm", meta["verdict"])
        self.assertNotIn("start", [c[0] for c in field.calls])

    def test_nav_restart_with_ros_arm_is_refused(self):  # 8
        rehearsal, scenario, *_ = make(ros_arm=True, nav_restart=True)
        meta = run(rehearsal, scenario)
        self.assertIn("--nav-restart", meta["verdict"])

    def test_lift_reply_lines_kept_and_echo_lines_feed_sensor(self):  # 10
        sensor = backends.Ultrasonic()
        printed = []
        lift = backends.LiftUno("/dev/null", output=printed.append, line_hook=sensor.feed)
        for line in ("Up stop distance: 10.0 cm", "Up start", "Echo: 1740 us | Distance: 30.00 cm", "Up complete"):
            lift._append(line)
        self.assertEqual([l for _, l in lift.lines], ["Up stop distance: 10.0 cm", "Up start", "Up complete"])
        self.assertEqual(sensor.reading()[0], 30.0)  # 부팅 줄의 정지 기준 10.0 은 거리로 읽지 않는다
        self.assertEqual(lift.last_cm, 30.0)
        self.assertFalse(any("Echo" in line for line in printed))
        self.assertTrue(backends.is_ultrasonic_only("US distance_cm=3"))
        self.assertFalse(backends.is_ultrasonic_only("Up complete"))

    def test_unexpected_error_still_stops_and_writes_summary(self):
        rehearsal, scenario, field, *_ = make()

        def boom(*_a):
            raise ZeroDivisionError("bug")

        rehearsal.do_wait = boom
        meta = run(rehearsal, scenario, only=["S08"])
        self.assertIn("ZeroDivisionError", meta["verdict"])
        self.assertIn(("stop",), field.calls)
        self.assertTrue((rehearsal.log.dir / "summary.md").exists())

    def test_helper_pure_functions(self):
        def info(uuid):
            return type("I", (), {"goal_id": type("U", (), {"uuid": uuid})()})()

        class S:
            def __init__(self, uuid, status):
                self.status, self.goal_info = status, info(uuid)
        array = type("A", (), {"status_list": [S([1], 2), S([2], 5)]})()
        self.assertEqual(ros_nav_idle.unfinished(["01", "02"], array), ["01"])  # 01 still executing
        self.assertEqual(ros_nav_idle.unfinished(["01"], None), ["01"])  # no status = unconfirmed (2nd #1)
        canceling = type("A", (), {"status_list": [S([3], 3), S([4], 4)]})()
        self.assertEqual(ros_nav_idle.active_in(canceling), ["03"])  # already CANCELING is still waited for (3rd #1)
        self.assertEqual(ros_nav_idle.active_in(None), [])
        self.assertEqual(ros_nav_idle.foreign_publishers({"velocity_smoother", "packagu_field_web_ui"}),
                         ["packagu_field_web_ui"])
        self.assertTrue(ros_map_switch.same_size((10.0, 5.0), (10.1, 4.95)))

        def pose(x, y):
            return type("P", (), {"position": type("Q", (), {"x": x, "y": y})()})()
        grid = type("G", (), {"info": type("I", (), {"width": 4, "height": 2, "resolution": 0.05,
                                                     "origin": pose(-1.0, -2.0)})(),
                              "data": [0, 100, 0, 100, -1, 0, 100, 0]})()
        meta = type("M", (), {"size_x": 4, "size_y": 2, "resolution": 0.05, "origin": pose(-1.0, -2.0)})()
        fresh = [0, 254, 0, 254, 255, 0, 254, 0]
        stale = [0, 0, 254, 0, 255, 0, 0, 0]
        self.assertEqual(ros_map_switch.costmap_matches_map(meta, fresh, grid), (True, 1.0))
        self.assertEqual(ros_map_switch.costmap_matches_map(meta, stale, grid), (False, 0.0))
        other = type("M", (), {"size_x": 8, "size_y": 2, "resolution": 0.05, "origin": pose(-1.0, -2.0)})()
        self.assertEqual(ros_map_switch.costmap_matches_map(other, fresh, grid), (False, None))
        self.assertFalse(ros_map_switch.same_size((10.0, 5.0), (20.0, 5.0)))


class CodexSecondReviewTest(unittest.TestCase):
    """2026-10-21 Codex 2차 검토 지적 재발 방지."""

    def test_lift_only_run_stops_before_opening_lift(self):  # 2
        rehearsal, scenario, field, _, lift, _ = make()
        original = lift.open
        lift.open = lambda: (field.calls.append(("lift_open",)), original())
        run(rehearsal, scenario, only=["S04"])
        names = [c[0] for c in field.calls]
        self.assertLess(names.index("stop"), names.index("lift_open"))
        self.assertLess(names.index("idle"), names.index("lift_open"))

    def test_unsettled_lift_blocks_next_lift_command(self):  # 3
        rehearsal, scenario, _, _, lift, _ = make(failures={"move:up": 1, "settle": 10}, answers=["", "", "r", "q"])
        meta = run(rehearsal, scenario, only=["S04"])
        self.assertEqual([c for c in lift.calls if c[0] == "move"], [("move", "up")])  # 끝 줄 확인 전 재전송 없음
        self.assertGreaterEqual(lift.calls.count(("settle",)), 2)
        self.assertTrue(lift.moving)
        self.assertTrue(meta["verdict"].startswith("중단"))

    def test_clear_failure_after_pose_fails_map_step(self):  # 5
        rehearsal, scenario, field, *_ = make(failures={"clear": 1}, answers=["q"])
        meta = run(rehearsal, scenario, only=["S13"])
        self.assertTrue(meta["verdict"].startswith("중단"))
        self.assertEqual(rehearsal.results, [])

    def test_confirm_quit_aborts_before_map_switch(self):  # 8
        rehearsal, scenario, field, *_ = make(answers=["", "", "q"])
        meta = run(rehearsal, scenario, start="S12", end="S13")
        self.assertTrue(meta["verdict"].startswith("중단"))
        self.assertNotIn("load_map", [c[0] for c in field.calls])

    def test_confirm_other_input_asks_again(self):  # 8
        rehearsal, scenario, field, *_ = make(answers=["", "", "x", ""])
        run(rehearsal, scenario, only=["S12"])
        self.assertEqual(rehearsal.results[0]["status"], "OK")

    def test_nav_restart_refused_when_arm_node_runs_without_button_steps(self):  # 9
        rehearsal, scenario, field, *_ = make(nav_restart=True, arm_node=True)
        meta = run(rehearsal, scenario, only=["S13"])
        self.assertIn("--nav-restart", meta["verdict"])
        self.assertNotIn("nav_stop", [c[0] for c in field.calls])

    def test_cleanup_continues_when_record_stop_fails(self):  # 10
        rehearsal, scenario, field, _, lift, _ = make()

        def broken():
            raise RuntimeError("recorder hung")

        field.record_stop = broken
        meta = run(rehearsal, scenario, only=["S04"])
        self.assertIn(("close",), lift.calls)
        self.assertTrue((rehearsal.log.dir / "summary.md").exists())
        self.assertTrue(meta["verdict"].startswith("정리 실패"), meta["verdict"])  # 3차 #3
        self.assertEqual(len(meta["cleanup_errors"]), 1)

    def test_real_lift_blocks_next_command_until_end_line(self):  # 3
        lift = backends.LiftUno("/dev/null", output=lambda _l: None)
        lift.conn = FakeUnoConn(complete=False)
        threading.Thread(target=lift._reader, daemon=True).start()
        with self.assertRaises(backends.BackendError) as ctx:
            lift.move("up", 0.3)
        self.assertIn("물리 비상정지", str(ctx.exception))
        self.assertTrue(lift.moving)
        self.assertFalse(lift.wait_until_stopped(0.2))
        with self.assertRaises(backends.BackendError):
            lift.move("down", 1.0)  # 펌웨어 버퍼에 쌓이면 이동이 끝난 뒤 실행되므로 보내지 않는다
        self.assertEqual(lift.conn.written, ["u"])
        lift.close()


class UltrasonicTest(unittest.TestCase):
    def test_parse_formats(self):
        cases = {"US distance_cm=23.4": 23.4, "DIST:12": 12.0, "ultrasonic 7.5": 7.5,
                 "STATUS moving=0 position_pulses=166400 move_size=166400": None,
                 "STATUS moving=0 position_pulses=0 us_cm=15": 15.0, "UP complete": None, "US distance_cm=0": None,
                 # lift_uno 펌웨어 (2026-10-29)
                 "Echo: 1740 us | Distance: 30.00 cm": 30.0, "Up stop distance: 10.0 cm": None,
                 "Up stopped: distance below stop threshold": None, "Up complete": None,
                 "Sensor: Up only; No echo does not stop Up": None}
        for line, expected in cases.items():
            self.assertEqual(backends.parse_ultrasonic(line), expected, line)

    def test_freshness_uses_clock(self):
        now = [100.0]
        sensor = backends.Ultrasonic(stale_sec=0.5, clock=lambda: now[0])
        self.assertFalse(sensor.fresh())
        self.assertTrue(sensor.feed("US distance_cm=30"))
        self.assertTrue(sensor.fresh())
        now[0] += 0.6
        self.assertFalse(sensor.fresh())
        self.assertFalse(sensor.feed("UP start"))

    def test_approach_stops_at_threshold(self):
        rehearsal, scenario, field, *_ = make(ultrasonic=backends.FakeUltrasonic([40, 30, 20, 9, 7.5]))
        run(rehearsal, scenario, only=["S03"])
        result = rehearsal.results[0]
        self.assertEqual(result["status"], "OK")
        self.assertIn("7.5", result["detail"])
        self.assertIn(("straight_until", 0.6), field.calls)

    def test_stale_sensor_blocks_approach(self):
        rehearsal, scenario, field, *_ = make(ultrasonic=backends.FakeUltrasonic(stale=True))
        meta = run(rehearsal, scenario, only=["S03"])
        self.assertTrue(meta["verdict"].startswith("중단"))
        self.assertNotIn("straight_until", [c[0] for c in field.calls])

    def test_never_reaching_threshold_fails(self):
        rehearsal, scenario, field, *_ = make(ultrasonic=backends.FakeUltrasonic([50]), answers=["", "", "", "s"])
        run(rehearsal, scenario, only=["S03"])
        self.assertEqual(rehearsal.results[0]["status"], "MANUAL")
        self.assertIn("max distance", rehearsal.results[0]["detail"])

    def test_lift_uno_hands_ultrasonic_lines_to_sensor(self):
        sensor = backends.Ultrasonic()
        printed = []
        lift = backends.LiftUno("/dev/null", output=printed.append, line_hook=sensor.feed)
        lift._append("US distance_cm=11")
        lift._append("Up complete")
        self.assertEqual(sensor.reading()[0], 11.0)
        self.assertEqual([line for _, line in lift.lines], ["Up complete"])

    def test_banner_stop_distance_is_not_a_reading(self):
        sensor = backends.Ultrasonic()
        lift = backends.LiftUno("/dev/null", output=lambda _l: None, line_hook=sensor.feed)
        lift._append("Up stop distance: 10.0 cm")
        self.assertIsNone(sensor.reading()[0])
        self.assertEqual(lift.lines[-1][1], "Up stop distance: 10.0 cm")


class FakeUnoConn:
    """lift_uno 펌웨어 응답을 흉내 내는 가짜 시리얼 (부팅 안내 줄, u/d 한 글자 명령)."""

    BANNER = ("Motor ready", "u: Up (ultrasonic stop)", "d: Down (no ultrasonic stop)",
              "Sensor: Up only; No echo does not stop Up", "Up stop distance: 10.0 cm")

    def __init__(self, complete=True, height_limit=False, ack=True, banner=BANNER):
        self.out = queue.Queue()
        self.complete, self.height_limit, self.ack = complete, height_limit, ack
        self.written = []
        for line in banner:
            self.out.put(line)

    def write(self, data):
        command = data.decode()
        self.written.append(command)
        if command in ("u", "d") and self.ack:
            up = command == "u"
            self.out.put("Up start" if up else "Down start")
            if up:
                self.out.put("Echo: 1740 us | Distance: 30.00 cm")
            if up and self.height_limit:
                self.out.put("Echo: 551 us | Distance: 9.50 cm")
                self.out.put("Up stopped: distance below stop threshold")
            elif self.complete:
                self.out.put("Up complete" if up else "Down complete")
        return len(data)

    def flush(self):
        pass

    def readline(self):
        try:
            return (self.out.get(timeout=0.05) + "\n").encode()
        except queue.Empty:
            return b""

    def close(self):
        pass


class LiftUnoTest(unittest.TestCase):
    def make(self, **conn_kwargs):
        lift = backends.LiftUno("/dev/null", output=lambda _l: None)
        lift.conn = FakeUnoConn(**conn_kwargs)
        since = time.monotonic()
        threading.Thread(target=lift._reader, daemon=True).start()
        return lift, since

    def test_banner_up_down(self):
        lift, since = self.make()
        self.assertEqual(lift.check_banner(since, 2.0), 10.0)
        up = lift.move("up", 2.0)
        self.assertEqual((up["result"], up["last_cm"]), ("complete", 30.0))
        self.assertEqual(lift.move("down", 2.0)["result"], "complete")
        self.assertEqual(lift.conn.written, ["u", "d"])  # 한 글자씩, 줄바꿈 없음
        self.assertFalse(lift.moving)
        lift.close()

    def test_height_limit_stop(self):
        lift, since = self.make(height_limit=True)
        lift.check_banner(since, 2.0)
        result = lift.move("up", 2.0)
        self.assertEqual((result["result"], result["last_cm"]), ("height_limit", 9.5))
        self.assertFalse(lift.moving)
        lift.close()

    def test_wrong_firmware_banner_is_rejected(self):
        lift, since = self.make(banner=("READY UNO_TB6600_POSITION_GUARD_V1",))
        with self.assertRaises(backends.BackendError):
            lift.check_banner(since, 0.5)
        lift.close()

    def test_missing_ack_keeps_lift_blocked(self):
        lift, _ = self.make(ack=False)
        with self.assertRaises(backends.BackendError):
            lift.move("up", 3.0)
        self.assertTrue(lift.moving)
        lift.close()

    def test_late_end_line_unblocks(self):
        lift, _ = self.make(complete=False)
        with self.assertRaises(backends.BackendError):
            lift.move("down", 0.3)
        self.assertTrue(lift.moving)
        threading.Timer(0.1, lambda: lift.conn.out.put("Down complete")).start()
        self.assertTrue(lift.wait_until_stopped(2.0))
        self.assertFalse(lift.moving)
        lift.close()


class LiftSerialErrorTest(unittest.TestCase):
    def test_serial_error_during_move_fails_and_keeps_lift_blocked(self):  # 5
        lift = backends.LiftUno("/dev/null", output=lambda _l: None)
        lift.conn = FakeUnoConn(complete=False)
        threading.Thread(target=lift._reader, daemon=True).start()
        threading.Timer(0.2, lambda: lift._append("SERIAL_ERROR device disconnected")).start()
        with self.assertRaises(backends.BackendError):
            lift.move("up", 5.0)
        self.assertTrue(lift.moving)
        self.assertEqual(lift.conn.written, ["u"])  # 펌웨어에 정지 명령이 없어 '!' 를 보내지 않는다
        lift.close()


class LiftStepTest(unittest.TestCase):
    """2026-10-29 lift_uno 펌웨어(초음파 박스 높이 제한) 반영."""

    def test_height_limit_up_is_ok_and_down_asks_first(self):
        rehearsal, scenario, _, _, lift, out = make(failures={"height": 1})
        run(rehearsal, scenario, only=["S04", "S16"])
        self.assertEqual(statuses(rehearsal), {"S04": "OK", "S16": "OK"})
        self.assertIn("height limit", rehearsal.results[0]["detail"])
        self.assertTrue(any("덜 올라갔습니다" in line for line in out))
        self.assertEqual(rehearsal.lift_pos, "bottom")

    def test_operator_can_cancel_down_after_height_limit(self):
        rehearsal, scenario, _, _, lift, _ = make(failures={"height": 1}, answers=["", "", "", "x", "s"])
        run(rehearsal, scenario, only=["S04", "S16"])
        self.assertEqual(statuses(rehearsal), {"S04": "OK", "S16": "MANUAL"})
        self.assertEqual([c for c in lift.calls if c[0] == "move"], [("move", "up")])

    def test_lift_refuses_moves_past_the_ends(self):
        rehearsal, *_ = make()
        for pos in ("top", "partial"):
            rehearsal.lift_pos = pos
            with self.assertRaises(runner.StepFailed):
                rehearsal.do_lift({"id": "S04", "action": "up"})
        rehearsal.lift_pos = "bottom"
        with self.assertRaises(runner.StepFailed):
            rehearsal.do_lift({"id": "S16", "action": "down"})
        self.assertEqual([c for c in rehearsal.lift.calls if c[0] == "move"], [])

    def test_unknown_position_after_failure_is_asked_before_retry(self):
        rehearsal, scenario, _, _, lift, out = make(failures={"move:up": 1}, answers=["", "", "r", ""])
        run(rehearsal, scenario, only=["S04"])
        self.assertEqual((rehearsal.results[0]["status"], rehearsal.results[0]["attempts"]), ("OK", 2))
        self.assertTrue(any("포크 위치를 진행기가 모릅니다" in line for line in out))
        self.assertEqual(rehearsal.lift_pos, "top")


class ScenarioTest(unittest.TestCase):
    def test_scenario_matches_registered_waypoints_and_floors(self):
        scenario = runner.load_scenario(SCENARIO)
        registry = json.loads(REGISTRY.read_text(encoding="utf-8"))["waypoints"]
        floor = "F1"
        for step in scenario["steps"]:
            if step["type"] == "map":
                floor = step["floor"]
            if "waypoint" in step:
                self.assertIn(step["waypoint"], registry, step["id"])
                self.assertEqual(registry[step["waypoint"]]["floor"], floor, step["id"])

    def test_target_floor_is_4_going_up_and_1_going_down(self):
        scenario = runner.load_scenario(SCENARIO)
        car = [runner.BUTTON_LABELS[(s["stage"], s["direction"])] for s in scenario["steps"]
               if s["type"] == "button" and s["stage"] == "car"]
        self.assertEqual(car, ["4", "1"])

    def test_state_before(self):
        steps = runner.load_scenario(SCENARIO)["steps"]
        self.assertEqual(runner.state_before(steps, "S14"),
                         {"floor": "F2", "lift": "top", "waypoint": "f2_elevator_exit"})
        self.assertEqual(runner.state_before(steps, "S01"), {"floor": "F1", "lift": "bottom", "waypoint": None})

    def test_mid_run_start_sets_pose_and_loads_floor_map(self):
        rehearsal, scenario, field, _, lift, _ = make(loaded="F1")
        run(rehearsal, scenario, start="S14", end="S14")
        self.assertIn(("load_map", "/ros2_ws/maps/field/f2/pinned.yaml"), field.calls)
        self.assertIn(("pose", -11.375, -2.525, -0.829), field.calls)
        self.assertEqual(rehearsal.results[0]["status"], "OK")

    def test_only_lift_down_expects_fork_on_top(self):
        rehearsal, scenario, _, _, lift, out = make()
        run(rehearsal, scenario, only=["S16"])
        self.assertTrue(any("올라간 위치" in line for line in out))
        self.assertEqual(rehearsal.results[0]["status"], "OK")
        self.assertEqual(rehearsal.lift_pos, "bottom")

    def test_nav2_start_is_retried_once_when_base_is_slow(self):
        rehearsal, scenario, field, *_ = make(nav_running=False)
        original = field.start_background
        calls = []

        def flaky(key, args, pattern, timeout, log_dir):
            calls.append(key)
            if len(calls) == 1:
                raise backends.BackendError("nav exited early")
            return original(key, args, pattern, timeout, log_dir)

        field.start_background = flaky
        run(rehearsal, scenario, only=["S02"])
        self.assertEqual(calls, ["nav_F1", "nav_F1"])
        self.assertEqual(rehearsal.results[0]["status"], "OK")

    def test_nav_restart_mode_restarts_instead_of_load_map(self):
        rehearsal, scenario, field, *_ = make(nav_restart=True)
        run(rehearsal, scenario, only=["S13"])
        names = [c[0] for c in field.calls]
        self.assertIn("nav_stop", names)
        self.assertNotIn("load_map", names)
        self.assertIn(("start", "nav_F2", "nav", "start", "F2"), field.calls)

    def test_bad_scenario_is_rejected(self):
        bad = {"steps": [{"id": "X", "type": "button", "stage": "car", "direction": "sideways", "title": "x"}]}
        path = Path(tempfile.mkdtemp()) / "bad.json"
        path.write_text(json.dumps(bad), encoding="utf-8")
        with self.assertRaises(ValueError):
            runner.load_scenario(path)


class StraightMoveTest(unittest.TestCase):
    def test_limits(self):
        with self.assertRaises(ValueError):
            ros_straight_move.validate(1.5, 0.05)
        with self.assertRaises(ValueError):
            ros_straight_move.validate(0.4, 0.2)
        ros_straight_move.validate(-0.4, 0.05)

    def test_speed_profile_stays_in_bounds(self):
        for elapsed in (0.0, 0.5, 3.0):
            for remaining in (0.001, 0.05, 0.5):
                v = ros_straight_move.plan_speed(elapsed, remaining, 0.05)
                self.assertGreaterEqual(v, ros_straight_move.MIN_SPEED_MPS)
                self.assertLessEqual(v, 0.05)


if __name__ == "__main__":
    unittest.main(verbosity=1)
