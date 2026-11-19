#!/usr/bin/env python3
"""장치 없이 촬영/인식 분리, HTTP 새 프레임, 신선도 및 예외 복구를 검증한다."""
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
import urllib.request
from unittest.mock import patch

import numpy as np
import yaml

import camera_server as camera


class FakeCapture:
    def __init__(self, *args):
        self.n = 0

    def isOpened(self):
        return True

    def set(self, *args):
        return True

    def read(self):
        time.sleep(1 / 30.0)
        self.n += 1
        return True, np.full((48, 64, 3), self.n % 255, np.uint8)

    def release(self):
        pass


class Estimator:
    k_source = "test"

    def estimate(self, frame):
        return {"ok": False, "reason": "test: no target"}

    def annotate(self, frame, est, lines):
        return frame.copy()


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("timed out waiting for camera")


class CameraStreamTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cfg = yaml.safe_load((camera.HERE / "config.yaml").read_text())
        self.patch = patch.object(camera, "make_estimator", return_value=Estimator())
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.addCleanup(self.tmp.cleanup)

    def service(self, source):
        return camera.CameraService(self.cfg, source, Path(self.tmp.name) / "teach.json", self.tmp.name)

    def test_slow_recognition_does_not_block_live_http(self):
        entered, release = threading.Event(), threading.Event()
        with patch.object(camera.cv2, "VideoCapture", FakeCapture):
            source = camera.V4L2Source(0, 64, 48)
        service = self.service(source)

        def estimate(frame):
            entered.set()
            release.wait(5)
            return {"ok": False, "reason": "slow test"}

        service.estimator.estimate = estimate
        worker = threading.Thread(target=service.run_forever, daemon=True)
        server = camera.ThreadingServer(("127.0.0.1", 0), camera.make_handler(service))
        http = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        http.start()
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        url = "http://127.0.0.1:%d" % server.server_port
        try:
            self.assertTrue(entered.wait(2))
            wait_for(lambda: source.preview()[0] >= 2)
            with opener.open(url + "/frame.jpg?view=live", timeout=2) as response:
                seq = int(response.headers["X-Frame-Sequence"])
                body = response.read()
                self.assertEqual(response.headers["Cache-Control"], "no-store")
                self.assertTrue(body.startswith(b"\xff\xd8"))
            wait_for(lambda: source.preview()[0] >= seq + 3)
            with opener.open(url + "/frame.jpg?view=live", timeout=2) as response:
                self.assertGreater(int(response.headers["X-Frame-Sequence"]), seq)
                self.assertNotEqual(body, response.read())
                self.assertLess(float(response.headers["X-Frame-Age"]), 0.5)
            with opener.open(url + "/api/state", timeout=2) as response:
                state = json.load(response)
                self.assertGreater(state["capture_frames"], 3)
                self.assertEqual(state["frames"], 0)  # 인식이 아직 막혀 있어도 영상은 새로 온다.
            # 끊긴 클라이언트 다음 요청도 같은 최신 촬영 소스를 쓴다.
            with opener.open(url + "/live.mjpg", timeout=2) as response:
                self.assertIn(b"Content-Type: image/jpeg", response.read(100))
        finally:
            service.stop()
            release.set()
            worker.join(2)
            source.thread.join(2)
            server.shutdown()
            server.server_close()

    def test_capture_timestamp_is_not_replaced_by_recognition_time(self):
        class OldFrame:
            stamp = time.monotonic() - 2.0

            def __call__(self):
                return np.zeros((48, 64, 3), np.uint8)

        source = OldFrame()
        service = self.service(source)
        service.estimator.estimate = lambda frame: {"ok": True, "cam": {"x": -1, "y": 0, "heading": 0},
                                                    "reprojection_px": 0}
        self.assertTrue(service.step())
        self.assertEqual(service.latest["stamp"], source.stamp)
        self.assertGreater(service.state()["age_s"], 1.9)
        self.assertFalse(service.state()["ok"])
        result = service.look(n=1, timeout=0.06)
        self.assertFalse(result["ok"])
        self.assertEqual(result["frames_seen"], 0)

    def test_look_without_target_clears_previous_target(self):
        # 2026-11-03 강의실 1d: 앞선 look 의 송장 지정이 남아 매 프레임 숫자 판독 → 시작 점검 4.5 fps < 5 로 정지
        service = self.service(lambda: None)
        if service.mode != "tape":
            self.skipTest("scene evidence is tape mode only")
        service.preview_target_hint = {"label": "402"}
        self.assertTrue(service.scene_wanted())
        service.look(n=1, timeout=0.05)
        self.assertIsNone(service.preview_target_hint)
        self.assertIsNone(service.scene_wanted())

    def test_recognition_exception_recovers(self):
        service = self.service(lambda: np.zeros((48, 64, 3), np.uint8))
        original = service.estimator.estimate
        called = threading.Event()

        def estimate(frame):
            if not called.is_set():
                called.set()
                raise ValueError("one bad frame")
            return original(frame)

        service.estimator.estimate = estimate
        worker = threading.Thread(target=service.run_forever, daemon=True)
        worker.start()
        try:
            wait_for(lambda: called.is_set() and service.frames > 0)
            self.assertIsNone(service.camera_error)
        finally:
            service.stop()
            worker.join(2)

    def test_capture_consumer_waits_for_a_new_frame(self):
        with patch.object(camera.cv2, "VideoCapture", FakeCapture):
            source = camera.V4L2Source(0, 64, 48)
        try:
            source()
            stamp = source.stamp
            source()
            self.assertGreater(source.stamp, stamp)
            self.assertGreater(source.frames, 1)
        finally:
            source.stop()
            source.thread.join(2)


if __name__ == "__main__":
    unittest.main()
