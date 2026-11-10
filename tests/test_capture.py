# Copyright (c) 2026 Airvolute s.r.o.
# Licensed under the Airvolute DroneCore Platform License (ADPL), Version 1.0.
# Use permitted only within Airvolute Hardware, the DroneCore Platform,
# or Derived Systems. IP remains with Airvolute s.r.o.
# Contact: info@airvolute.com

"""Passive recording and clean teleop shutdown against local fake sockets and devices."""

import io
import json
import socket
import struct
import threading
from types import SimpleNamespace

import pytest

from lerobot_robot_lwa4p.bridge_client import ArmFault
from lerobot_robot_lwa4p.capture import analyze, record


def test_capture_only_requests_read_operations_and_retains_can(tmp_path):
    """Interleave CAN frames and a fault snapshot without sending actuator commands."""
    operations = []
    bus, writer = socket.socketpair(type=socket.SOCK_DGRAM)
    client, server = socket.socketpair()

    def serve():
        """Respond to read-only bridge operations and expose the final diagnostic trace."""
        with server, server.makefile("rwb") as stream:
            for line in stream:
                request = json.loads(line)
                operations.append(request["op"])
                if request["op"] == "state":
                    reply = {
                        "q": [0.0] * 6,
                        "fault": "arm_5_joint emergency 0x3222",
                        "controller": {"error": [0.001] * 6},
                    }
                else:
                    reply = {
                        "events": [{"validation_ms": 12.0, "accepted": False, "error": "0x3222"}]
                    }
                stream.write((json.dumps(reply) + "\n").encode())
                stream.flush()

    worker = threading.Thread(target=serve, daemon=True)
    worker.start()

    def frame(can_id, payload):
        """Encode a Linux classic CAN frame for the fake datagram channel."""
        return struct.pack("=IB3x8s", can_id, len(payload), payload.ljust(8, b"\0"))

    writer.send(frame(0x80, b""))
    writer.send(frame(0x207, struct.pack("<Hhi", 0x1F, 0, 12345)))
    writer.send(frame(0x207, struct.pack("<Hhi", 0x1F, 0, 12346)))
    writer.send(frame(0x87, bytes.fromhex("2232257400000000")))
    output = io.StringIO()
    try:
        counts = record(bus, client, output, 0.2)
    finally:
        client.close()
        bus.close()
        writer.close()
        worker.join(timeout=1.0)
    assert counts["can"] == 4
    assert set(operations) == {"state", "trace"}
    assert operations.count("trace") == 1
    path = tmp_path / "capture.jsonl"
    path.write_text(output.getvalue())
    report = analyze(path)
    assert report["nodes"][7]["max_step_deg"] == pytest.approx(0.001)
    assert report["emergencies"][0]["code"] == "0x3222"
    assert report["validation_max_ms"] == 12.0
    assert report["voltage_v"] is None


def test_teleop_fault_has_clear_exit_and_always_disconnects(monkeypatch, capsys):
    """The recommended CLI reports the drive fault without a Python traceback."""
    import lerobot_robot_lwa4p.robot as robot_module
    import lerobot_robot_lwa4p.teleop as teleop_module
    from lerobot_robot_lwa4p.main import _teleop

    closed = []

    class Robot:
        """Provide a drive that trips on its first command."""

        def __init__(self, config):
            """Accept the same CLI configuration as the real robot."""
            pass

        def connect(self):
            """Simulate an available local bridge."""
            pass

        def send_action(self, action):
            """Expose the actual drive reason to the CLI."""
            raise ArmFault("arm_5_joint emergency 0x3222")

        def disconnect(self):
            """Verify cleanup executes on a drive fault."""
            closed.append("robot")

    class Pad:
        """Supply a fake gamepad without touching any input device."""

        def __init__(self, config):
            """Accept the usual gamepad configuration."""
            pass

        def connect(self):
            """Simulate a connected controller."""
            pass

        def get_action(self):
            """Supply a single placeholder command."""
            return {}

        def disconnect(self):
            """Verify the gamepad closes too."""
            closed.append("pad")

    monkeypatch.setattr(robot_module, "Lwa4p", Robot)
    monkeypatch.setattr(teleop_module, "Lwa4pGamepad", Pad)
    args = SimpleNamespace(
        host="127.0.0.1",
        port=5555,
        joint_speed=0.3,
        acceleration=0.3,
        jerk=2.0,
        speed=0.03,
        grasp_gap=0.017,
    )
    with pytest.raises(SystemExit) as result:
        _teleop(args)
    assert result.value.code == 2
    text = capsys.readouterr().out
    assert "0x3222" in text
    assert "Traceback" not in text
    assert closed == ["robot", "pad"]
