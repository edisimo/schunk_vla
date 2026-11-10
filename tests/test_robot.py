"""Lwa4p robot against a fake bridge speaking the same JSON protocol."""

import json
import socketserver
import threading

import pytest

from lerobot_robot_lwa4p import Lwa4p, Lwa4pConfig


class _FakeBridge(socketserver.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            req = json.loads(line)
            self.server.requests.append(req)
            if req["op"] == "state":
                reply = {"q": [0.1] * 6, "gripper": 0.05, "stamp": 0.0, "error": ""}
            elif req["op"] == "act":
                reply = {"ok": True, "q": [v * 0.5 for v in req["q"]], "error": ""}
            else:
                reply = {"ok": True}
            self.wfile.write((json.dumps(reply) + "\n").encode())


@pytest.fixture
def robot():
    server = socketserver.ThreadingTCPServer(("127.0.0.1", 0), _FakeBridge)
    server.daemon_threads, server.requests = True, []
    threading.Thread(target=server.serve_forever, daemon=True).start()
    r = Lwa4p(Lwa4pConfig(port=server.server_address[1], cameras={}))
    r.connect()
    yield r, server.requests
    r.disconnect()
    server.shutdown()


def test_observation(robot):
    r, _ = robot
    obs = r.get_observation()
    assert set(obs) == set(r.observation_features)
    assert obs["arm_3.pos"] == 0.1 and obs["gripper.pos"] == 0.05


def test_action_returns_what_the_bridge_sent(robot):
    r, requests = robot
    action = {f"arm_{i}.pos": 1.0 for i in range(1, 7)} | {"gripper.pos": 0.5}
    sent = r.send_action(action)
    assert sent["arm_1.pos"] == 0.5
    assert sent["gripper.pos"] == r.config.gripper_open_m
    assert requests[-1]["gripper"] == r.config.gripper_open_m


def test_disconnect_holds(robot):
    r, requests = robot
    r.disconnect()
    assert requests[-1]["op"] == "hold"
    r.connect()


def test_depth_to_rgb():
    import numpy as np

    from lerobot_robot_lwa4p.robot import depth_to_rgb

    img = depth_to_rgb(np.array([[[0], [100], [550], [2000]]], dtype=np.uint16), 0.1, 1.0)
    assert img.shape == (1, 4, 3) and img.dtype == np.uint8
    assert list(img[0, :, 0]) == [0, 255, 127, 0]
