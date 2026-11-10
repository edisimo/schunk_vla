"""Client for the lwa4p_vla_bridge ROS 2 node (newline-delimited JSON over TCP)."""

import json
import socket
import threading


class BridgeError(ConnectionError):
    """The bridge is unreachable or closed the connection."""


class BridgeClient:
    """Thread-safe request/reply connection to the bridge."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5555, timeout_s: float = 2.0):
        self.host, self.port, self.timeout_s = host, port, timeout_s
        self._lock = threading.Lock()
        self._sock = self._file = None

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        except OSError as e:
            raise BridgeError(
                f"no bridge at {self.host}:{self.port} - start it with "
                f"'ros2 launch lwa4p_vla_bridge robot.launch.py' ({e})"
            ) from e
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._file = self._sock.makefile("rw", encoding="utf-8")

    def close(self) -> None:
        if self._sock is not None:
            self._file.close()
            self._sock.close()
            self._sock = self._file = None

    def call(self, op: str, **kwargs) -> dict:
        """Send one request and return the bridge's reply."""
        if self._sock is None:
            raise BridgeError("bridge client is not connected")
        with self._lock:
            try:
                self._file.write(json.dumps({"op": op, **kwargs}) + "\n")
                self._file.flush()
                line = self._file.readline()
            except OSError as e:
                raise BridgeError(f"bridge connection lost: {e}") from e
        if not line:
            raise BridgeError("bridge closed the connection")
        return json.loads(line)
