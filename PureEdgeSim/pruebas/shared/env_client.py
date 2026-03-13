from __future__ import annotations

import json
import socket
from typing import Any, Dict, Optional


class MAPPOClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 5006, connect_timeout_s: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self._sock: Optional[socket.socket] = None
        self._file = None

    def connect(self) -> None:
        if self._sock is not None:
            return
        self._sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout_s)
        self._sock.settimeout(None)
        self._file = self._sock.makefile("r")

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
        if self._sock is not None:
            self._sock.close()
        self._file = None
        self._sock = None

    def _send_json(self, payload: Dict[str, Any]) -> None:
        self.connect()
        msg = json.dumps(payload, separators=(",", ":"))
        assert self._sock is not None
        self._sock.sendall((msg + "\n").encode("utf-8"))

    def recv_message(self) -> Dict[str, Any]:
        self.connect()
        assert self._file is not None
        line = self._file.readline()
        if not line:
            raise RuntimeError("Disconnected from MAPPOEnvServer.")
        return json.loads(line)

    def send_action(self, step_id: Any, dest_action: int, prb_action: int) -> None:
        payload = {
            "type": "marl_action",
            "step_id": step_id,
            "dest_action": int(dest_action),
            "prb_action": int(prb_action),
        }
        self._send_json(payload)

    def request_termination(self) -> None:
        self._send_json({"type": "control", "command": "terminate"})
