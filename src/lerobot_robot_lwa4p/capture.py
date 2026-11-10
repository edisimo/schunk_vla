# Copyright (c) 2026 Airvolute s.r.o.
# Licensed under the Airvolute DroneCore Platform License (ADPL), Version 1.0.
# Use permitted only within Airvolute Hardware, the DroneCore Platform,
# or Derived Systems. IP remains with Airvolute s.r.o.
# Contact: info@airvolute.com

"""Passive SocketCAN and bridge recorder; never send CAN frames or motion requests."""

import json
import selectors
import socket
import statistics
import struct
import time


def record(bus, bridge, output, duration):
    """Multiplex CAN reception and read-only bridge snapshots into a JSON-lines stream."""
    if duration <= 0:
        raise ValueError("capture duration must be positive")
    bus.setblocking(False)
    bridge.setblocking(False)
    deadline = time.monotonic() + duration
    next_poll, pending_since = 0.0, 0.0
    pending, outgoing, incoming = None, b"", b""
    trace_requested = False
    trace_saved = False
    counts = {"can": 0, "bridge": 0}

    def emit(source, **values):
        """Timestamp every record without assuming undocumented drive units."""
        output.write(json.dumps({"source": source, "received_ns": time.time_ns(), **values}) + "\n")

    emit("metadata", format_version=1, passive=True, duration_s=duration)
    with selectors.DefaultSelector() as selector:
        selector.register(bus, selectors.EVENT_READ, "can")
        selector.register(bridge, selectors.EVENT_READ, "bridge")
        bridge_alive = True
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if bridge_alive and pending is None and now >= next_poll:
                    pending = "trace" if trace_requested else "state"
                    outgoing = (json.dumps({"op": pending}) + "\n").encode()
                    pending_since = now
                if bridge_alive and outgoing:
                    try:
                        outgoing = outgoing[bridge.send(outgoing) :]
                    except BlockingIOError:
                        pass
                    except OSError as error:
                        emit("warning", error=str(error))
                        selector.unregister(bridge)
                        bridge_alive = False
                if bridge_alive and pending and now - pending_since > 2.0:
                    emit("warning", error="bridge capture request timed out; CAN capture continues")
                    selector.unregister(bridge)
                    bridge_alive = False
                for key, _ in selector.select(min(0.02, max(0.0, deadline - now))):
                    if key.data == "can":
                        for _ in range(256):
                            try:
                                data, ancillary, _, _ = bus.recvmsg(16, 128)
                            except BlockingIOError:
                                break
                            except OSError as error:
                                emit("warning", error=f"CAN capture stopped: {error}")
                                selector.unregister(bus)
                                break
                            if len(data) != 16:
                                emit("warning", error="short CAN frame")
                                break
                            can_id, length, payload = struct.unpack("=IB3x8s", data)
                            stamp, dropped = time.time_ns(), 0
                            for level, kind, value in ancillary:
                                if level == socket.SOL_SOCKET and kind == 35:
                                    sec, nsec = struct.unpack("@ll", value)
                                    stamp = sec * 1_000_000_000 + nsec
                                elif level == socket.SOL_SOCKET and kind == 40:
                                    dropped = struct.unpack("=I", value)[0]
                            emit(
                                "can",
                                stamp_ns=stamp,
                                can_id=can_id,
                                data=payload[:length].hex(),
                                dropped=dropped,
                            )
                            counts["can"] += 1
                    else:
                        try:
                            block = bridge.recv(65536)
                        except BlockingIOError:
                            continue
                        if not block:
                            emit("warning", error="bridge disconnected; CAN capture continues")
                            selector.unregister(bridge)
                            bridge_alive = False
                            continue
                        incoming += block
                        if len(incoming) > 16 * 1024 * 1024:
                            raise ValueError("bridge capture reply exceeded 16 MiB")
                        while b"\n" in incoming:
                            line, incoming = incoming.split(b"\n", 1)
                            result = json.loads(line)
                            emit("bridge", operation=pending, state=result)
                            counts["bridge"] += 1
                            trace_requested = (
                                pending == "state" and bool(result.get("fault")) and not trace_saved
                            )
                            if pending == "trace":
                                trace_saved = True
                                trace_requested = False
                                next_poll = time.monotonic() + 1.0
                            else:
                                if not result.get("fault"):
                                    trace_saved = False
                                next_poll = time.monotonic() + (0.0 if trace_requested else 0.05)
                            pending = None
                output.flush()
        except KeyboardInterrupt:
            emit("warning", error="capture interrupted by operator")
    emit("summary", **counts)
    output.flush()
    return counts


def capture(interface, host, port, output_path, duration):
    """Open a passive CAN socket and a bridge connection; never enable or move a drive."""
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        bus.setsockopt(socket.SOL_SOCKET, 35, 1)  # SO_TIMESTAMPNS: kernel arrival time.
        bus.setsockopt(socket.SOL_SOCKET, 40, 1)  # SO_RXQ_OVFL: cumulative receive losses.
        bus.bind((interface,))
        with socket.create_connection((host, port), timeout=2.0) as bridge:
            with open(output_path, "x", encoding="utf-8") as output:
                return record(bus, bridge, output, duration)


def analyze(input_path):
    """Summarize timing, fault frames and tracking errors without claiming voltage telemetry."""
    sync, latencies, emergencies = [], [], []
    previous, nodes = {}, {}
    dropped, tracking_error, count, bus_errors = 0, 0.0, 0, 0
    with open(input_path, encoding="utf-8") as source:
        for line in source:
            event = json.loads(line)
            if event["source"] == "can":
                count += 1
                can_id = event["can_id"]
                data = bytes.fromhex(event["data"])
                stamp = event["stamp_ns"] * 1e-9
                dropped = max(dropped, event.get("dropped", 0))
                if can_id & 0x20000000:
                    bus_errors += 1
                if can_id == 0x80:
                    sync.append(stamp)
                elif 0x83 <= can_id <= 0x88 and len(data) >= 2:
                    emergencies.append(
                        {
                            "node": can_id - 0x80,
                            "code": hex(int.from_bytes(data[:2], "little")),
                            "data": data.hex(),
                            "stamp": stamp,
                        }
                    )
                elif 0x203 <= can_id <= 0x208 and len(data) == 8:
                    node = can_id - 0x200
                    controlword, _, position = struct.unpack("<Hhi", data)
                    if controlword != 0x1F:
                        previous.pop(node, None)
                        continue
                    row = nodes.setdefault(
                        node, {"setpoints": 0, "max_step_deg": 0.0, "max_sampled_speed_deg_s": 0.0}
                    )
                    row["setpoints"] += 1
                    if node in previous:
                        old_stamp, old_position = previous[node]
                        step = abs(position - old_position) * 0.001
                        row["max_step_deg"] = max(row["max_step_deg"], step)
                        if stamp > old_stamp:
                            row["max_sampled_speed_deg_s"] = max(
                                row["max_sampled_speed_deg_s"], step / (stamp - old_stamp)
                            )
                    previous[node] = (stamp, position)
            elif event["source"] == "bridge":
                state = event["state"]
                if event["operation"] == "trace":
                    latencies.extend(item["validation_ms"] for item in state.get("events", []))
                controller = state.get("controller")
                if controller:
                    tracking_error = max(
                        tracking_error, max((abs(v) for v in controller["error"]), default=0.0)
                    )
    intervals = [(b - a) * 1000 for a, b in zip(sync, sync[1:])]
    return {
        "can_frames": count,
        "can_dropped": dropped,
        "can_error_frames": bus_errors,
        "nodes": nodes,
        "emergencies": emergencies,
        "sync_median_ms": statistics.median(intervals) if intervals else None,
        "sync_max_ms": max(intervals) if intervals else None,
        "validation_max_ms": max(latencies) if latencies else None,
        "max_sampled_tracking_error_rad": tracking_error,
        "voltage_v": None,
        "voltage_note": "No calibrated voltage object has been identified",
    }
