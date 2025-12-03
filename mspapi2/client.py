from __future__ import annotations

import base64
import json
import socket
import uuid
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

from .lib import InavMSP
from .mspcodec import MSPCodec

__all__ = ["MSPClientAPI"]


class MSPClientAPI:
    """TCP client for the JSON/line protocol exposed by msp_server.py."""

    def __init__(self, host: str, port: int, *, client_id: Optional[str] = None) -> None:
        if not host:
            raise ValueError("host is required")
        if not isinstance(port, int) or port <= 0:
            raise ValueError("port must be a positive integer")
        self.host = host
        self.port = port
        self.client_id = client_id
        self._sock: Optional[socket.socket] = None
        self._reader: Optional[Any] = None
        self.last_diag: Optional[Dict[str, Any]] = None
        self._codec = MSPCodec.from_json_file(str(Path(__file__).with_name("lib") / "msp_messages.json"))

    def open(self) -> None:
        if self._sock:
            return
        self._sock = socket.create_connection((self.host, self.port))
        self._reader = self._sock.makefile("r", encoding="utf-8")

    def close(self) -> None:
        if self._reader:
            try:
                self._reader.close()
            finally:
                self._reader = None
        if self._sock:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def _ensure_open(self) -> None:
        if not self._sock or not self._reader:
            self.open()

    def _send(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        self._ensure_open()
        payload.setdefault("id", uuid.uuid4().hex)
        if self.client_id:
            payload.setdefault("client_id", self.client_id)
        data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8") + b"\n"
        assert self._sock and self._reader
        self._sock.sendall(data)
        line = self._reader.readline()
        if not line:
            raise RuntimeError("MSP server closed the connection")
        resp = json.loads(line)
        if resp.get("id") != payload["id"]:
            raise RuntimeError("Mismatched response ID from server")
        return resp

    def request(
        self,
        code: int,
        payload: bytes = b"",
        timeout: float = 1.0,
        force_version: Optional[int] = None,
        cacheable: bool = True,
    ) -> Tuple[int, bytes]:
        message: Dict[str, Any] = {
            "code": int(code),
            "timeout_ms": max(100, int(timeout * 1000)),
            "raw": base64.b64encode(payload or b"").decode("ascii"),
        }
        if not cacheable:
            message["no_cache"] = True
        resp = self._send(message)
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "MSP server error")
        payload_b64 = resp.get("payload_b64", "")
        decoded = base64.b64decode(payload_b64) if payload_b64 else b""
        self.last_diag = resp.get("diag")
        return resp.get("code", int(code)), decoded

    def sched_set(
        self,
        code: int,
        delay: float,
        payload: Optional[Mapping[str, Any]] = None,
        timeout: float = 1.0,
    ) -> Dict[str, Any]:
        message: Dict[str, Any] = {
            "action": "sched_set",
            "code": int(code),
            "delay": float(delay),
            "timeout_ms": max(100, int(timeout * 1000)),
        }
        if payload is not None:
            message["payload"] = payload
        resp = self._send(message)
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Scheduler error")
        self.last_diag = resp.get("diag")
        return resp.get("schedule", {})

    def sched_get(self) -> Dict[str, Any]:
        resp = self._send({"action": "sched_get"})
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Scheduler error")
        self.last_diag = resp.get("diag")
        return resp.get("schedules", {})

    def sched_remove(self, code: int) -> Dict[str, Any]:
        resp = self._send({"action": "sched_remove", "code": int(code)})
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Scheduler error")
        self.last_diag = resp.get("diag")
        return resp

    def sched_data(self, *codes: InavMSP) -> Dict[int, Dict[str, Any]]:
        message: Dict[str, Any] = {"action": "sched_data"}
        if codes:
            resolved = []
            for code in codes:
                resolved.append(int(code.value) if isinstance(code, InavMSP) else int(code))
            message["codes"] = resolved
        resp = self._send(message)
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Scheduler data error")
        data = resp.get("data")
        if data is None:
            return {}
        out: Dict[int, Dict[str, Any]] = {}
        for raw_code, entry in data.items():
            code_int = int(raw_code)
            payload_b64 = entry.get("payload_b64")
            if not payload_b64:
                continue
            payload = base64.b64decode(payload_b64)
            decoded = self._codec.unpack_reply(InavMSP(code_int), payload)
            out[code_int] = {
                "time": entry.get("time"),
                "interval": entry.get("interval"),
                "data": decoded,
            }
        self.last_diag = resp.get("diag")
        return out

    def health(self) -> Dict[str, Any]:
        resp = self._send({"action": "health"})
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Health query failed")
        self.last_diag = resp.get("health")
        return resp.get("health", {})

    def utilization(self) -> Dict[str, Any]:
        resp = self._send({"action": "utilization"})
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Utilization query failed")
        self.last_diag = resp.get("utilization")
        return resp.get("utilization", {})

    def clients(self) -> Dict[str, Any]:
        resp = self._send({"action": "clients"})
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Clients query failed")
        self.last_diag = resp.get("diag") or {}
        return {"clients": resp.get("clients", []), "disconnects": resp.get("disconnects"), "errors": resp.get("errors")}

    def stats(self) -> Dict[str, Any]:
        resp = self._send({"action": "stats"})
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Stats query failed")
        self.last_diag = resp.get("diag") or {}
        return {"code_stats": resp.get("code_stats", {}), "errors": resp.get("errors"), "disconnects": resp.get("disconnects"), "last_error": resp.get("last_error")}

    def reset(self) -> None:
        resp = self._send({"action": "reset"})
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Reset failed")
        self.last_diag = resp.get("reset")
        return resp.get("reset", {})

    def shutdown(self) -> None:
        resp = self._send({"action": "shutdown"})
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error") or "Shutdown failed")
        self.last_diag = resp.get("shutdown")
