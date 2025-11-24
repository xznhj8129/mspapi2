from __future__ import annotations

import argparse
import base64
import json
import queue
import socket
import struct
import threading
import time
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, List, Mapping, Optional, Tuple
from collections import deque
import sys
import os

from .lib import InavMSP
from .msp_serial import MSPSerial
from .mspcodec import MSPCodec

CACHE_TTL_S = 0.1
GLOBAL_REQ_LIMIT_PER_SEC = 200
RATE_WINDOW_S = 1.0
SEND_QUEUE_LIMIT = 512
SERVER_BACKLOG = 16
REQUEST_QUEUE_POLL_S = 0.005
SCHEDULER_TICK_S = 0.02
MIN_TIMEOUT_MS = 50
MAX_PENDING_WAITERS = 512
ENCODING = "utf-8"
DEFAULT_FC_TIMEOUT = 1.0
DEFAULT_LOG_PATH = "server.log"


def _now_s() -> float:
    return time.monotonic()


def _crc_key(code: int, payload: bytes) -> str:
    crc = zlib.crc32(struct.pack("<H", int(code)) + payload) & 0xFFFFFFFF
    return f"{crc:08x}"

def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, bytes):
        try:
            return value.decode("latin1")
        except Exception as exc:
            raise TypeError(f"non-decodable bytes payload: {exc}") from exc
    if isinstance(value, bytearray):
        return _json_safe(bytes(value))
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    try:
        import enum
        if isinstance(value, enum.Enum):
            return value.name
    except Exception:
        pass
    raise TypeError(f"non-serializable type {type(value).__name__}")

def _redirect_output(log_path: Path) -> None:
    log_file = log_path.open("a", encoding="utf-8")
    orig_out = sys.stdout
    orig_err = sys.stderr

    class _Tee:
        def __init__(self, primary, mirror) -> None:
            self.primary = primary
            self.mirror = mirror

        def write(self, data: str) -> int:
            n1 = self.primary.write(data)
            self.primary.flush()
            self.mirror.write(data)
            self.mirror.flush()
            return n1

        def flush(self) -> None:
            self.primary.flush()
            self.mirror.flush()

    sys.stdout = _Tee(log_file, orig_out)
    sys.stderr = _Tee(log_file, orig_err)


def _pack_code(value: Any) -> int:
    if isinstance(value, InavMSP):
        return int(value.value)
    if isinstance(value, str):
        return int(InavMSP[value].value)
    return int(value)


@dataclass
class CacheEntry:
    payload: bytes
    diag: Dict[str, Any]
    created: float


@dataclass
class PendingRequest:
    code: int
    payload: bytes
    timeout_s: float
    key: str
    scheduled: bool
    schedule_delay: Optional[float]
    created: float
    force_version: Optional[int] = None
    throttle_ms: float = 0.0
    waiters: List[Tuple["ClientSession", str]] = field(default_factory=list)


@dataclass
class ScheduleEntry:
    code: int
    delay: float
    payload: Optional[Mapping[str, Any]]
    next_fire: float


@dataclass
class ClientSession:
    conn: socket.socket
    reader: Any
    addr: Tuple[str, int]
    client_id: str
    rate_window: Deque[float] = field(default_factory=deque)
    send_lock: threading.Lock = field(default_factory=threading.Lock)
    closed: bool = False

    def send_json(self, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode(ENCODING) + b"\n"
        with self.send_lock:
            self.conn.sendall(data)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.reader.close()
        finally:
            try:
                self.conn.close()
            finally:
                return


class MSPRequestServer:
    def __init__(
        self,
        host: str,
        port: int,
        *,
        serial_port: Optional[str] = None,
        baudrate: Optional[int] = None,
        tcp_endpoint: Optional[str] = None,
        fc_timeout: float = 1.0,
    ) -> None:
        if not host:
            raise ValueError("host is required")
        if not isinstance(port, int) or port <= 0:
            raise ValueError("port must be a positive integer")
        if serial_port and tcp_endpoint:
            raise ValueError("choose either serial_port or tcp_endpoint, not both")
        if not serial_port and not tcp_endpoint:
            raise ValueError("serial_port or tcp_endpoint must be provided")
        if serial_port and baudrate is None:
            raise ValueError("baudrate is required for serial operation")
        if tcp_endpoint and ":" not in tcp_endpoint:
            raise ValueError("tcp_endpoint must be HOST:PORT")
        if fc_timeout <= 0:
            raise ValueError("fc_timeout must be positive")

        self.host = host
        self.port = port
        self.codec = MSPCodec.from_json_file(str(Path(__file__).with_name("lib") / "msp_messages.json"))
        read_timeout = 0.05
        write_timeout = 0.25
        if serial_port:
            self.transport = MSPSerial(
                serial_port,
                baudrate,
                read_timeout=read_timeout,
                write_timeout=write_timeout,
                max_retries=1,
                keepalive_code=None,
                keepalive_interval=0.0,
                keepalive_timeout=0.0,
            )
        else:
            self.transport = MSPSerial(
                tcp_endpoint or "",
                baudrate or 0,
                read_timeout=read_timeout,
                write_timeout=write_timeout,
                max_retries=1,
                tcp=True,
                keepalive_code=None,
                keepalive_interval=0.0,
                keepalive_timeout=0.0,
            )
        self.server_sock: Optional[socket.socket] = None

        self.stop_evt = threading.Event()
        self.state_lock = threading.Lock()
        self.pending: Dict[str, PendingRequest] = {}
        self.cache: Dict[str, CacheEntry] = {}
        self.schedules: Dict[int, ScheduleEntry] = {}
        self.clients: Dict[str, ClientSession] = {}
        self.code_stats: Dict[int, Dict[str, float]] = {}
        self.start_time = _now_s()

        self.send_queue: queue.Queue[PendingRequest] = queue.Queue(maxsize=SEND_QUEUE_LIMIT)

        self.sender_thread = threading.Thread(target=self._sender_loop, name="msp_sender", daemon=True)
        self.scheduler_thread = threading.Thread(target=self._scheduler_loop, name="msp_scheduler", daemon=True)

    def serve_forever(self) -> None:
        self._log(f"start host={self.host} port={self.port} serial_port={self.transport.port} tcp={self.transport._use_tcp}")  # type: ignore[attr-defined]
        self.transport.open()
        self.sender_thread.start()
        self.scheduler_thread.start()

        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(SERVER_BACKLOG)
        self._log("listening")

        while not self.stop_evt.is_set():
            conn, addr = self.server_sock.accept()
            reader = conn.makefile("r", encoding=ENCODING)
            session = ClientSession(conn=conn, reader=reader, addr=addr, client_id="")
            threading.Thread(
                target=self._client_loop, args=(session,), name=f"msp_client_{addr[0]}_{addr[1]}", daemon=True
            ).start()

    def shutdown(self) -> None:
        self.stop_evt.set()
        if self.server_sock:
            try:
                self.server_sock.close()
            finally:
                self.server_sock = None
        self.transport.close()

    def _client_loop(self, session: ClientSession) -> None:
        try:
            while not self.stop_evt.is_set():
                line = session.reader.readline()
                if not line:
                    break
                self._log(f"recv client={session.client_id or 'unknown'} addr={session.addr} line_len={len(line.strip())}")
                self._handle_line(session, line.rstrip("\r\n"))
        finally:
            session.close()
            if session.client_id in self.clients:
                self.clients.pop(session.client_id, None)

    def _handle_line(self, session: ClientSession, line: str) -> None:
        if not line:
            self._send_error(session, None, "empty line")
            return
        try:
            req = json.loads(line)
        except json.JSONDecodeError as exc:
            self._send_error(session, None, f"invalid json: {exc}")
            return

        client_id = req.get("client_id")
        if not client_id or not isinstance(client_id, str):
            self._send_error(session, req.get("id"), "client_id is required")
            return
        if not session.client_id:
            session.client_id = client_id
            self.clients[client_id] = session
        elif session.client_id != client_id:
            self._send_error(session, req.get("id"), "client_id cannot change mid-connection")
            return
        self._log(f"parsed client_id={session.client_id} action={req.get('action')} code={req.get('code')} id={req.get('id')}")

        action = req.get("action")
        if action:
            if action == "sched_set":
                self._handle_sched_set(session, req)
            elif action == "sched_get":
                self._send_sched_get(session, req.get("id"))
            elif action == "sched_remove":
                self._handle_sched_remove(session, req)
            else:
                self._send_error(session, req.get("id"), f"unknown action {action}")
            return

        if "code" not in req:
            self._send_error(session, req.get("id"), "code is required")
            return
        if "timeout_ms" not in req:
            self._send_error(session, req.get("id"), "timeout_ms is required")
            return

        try:
            code = _pack_code(req["code"])
        except Exception as exc:
            self._send_error(session, req.get("id"), f"invalid code: {exc}")
            return
        timeout_ms = int(req["timeout_ms"])
        if timeout_ms < MIN_TIMEOUT_MS:
            self._send_error(session, req.get("id"), f"timeout_ms must be >= {MIN_TIMEOUT_MS}")
            return
        timeout_s = timeout_ms / 1000.0

        raw_payload = req.get("raw")
        payload_struct = req.get("payload")
        force_version = req.get("force_version")
        no_cache = bool(req.get("no_cache"))

        try:
            payload_bytes = self._build_payload(code, raw_payload, payload_struct)
        except Exception as exc:
            self._send_error(session, req.get("id"), str(exc))
            return
        key = _crc_key(code, payload_bytes)

        now = _now_s()
        window = session.rate_window
        while window and now - window[0] > RATE_WINDOW_S:
            window.popleft()
        active_clients = max(1, len(self.clients))
        limit = max(1, int(GLOBAL_REQ_LIMIT_PER_SEC / active_clients))
        if len(window) < limit:
            window.append(now)
            throttle_ms = 0.0
        else:
            earliest = window[0]
            wait = RATE_WINDOW_S - (now - earliest)
            time.sleep(wait)
            window.append(_now_s())
            throttle_ms = wait * 1000.0

        if not no_cache:
            with self.state_lock:
                entry = self.cache.get(key)
            if entry:
                age = _now_s() - entry.created
                if age <= CACHE_TTL_S:
                    diag = dict(entry.diag)
                    diag["cached"] = True
                    diag["cache_age_ms"] = age * 1000.0
                    session.send_json(
                        {
                            "id": req.get("id"),
                            "ok": True,
                            "code": code,
                            "name": InavMSP(code).name,
                            "payload_b64": base64.b64encode(entry.payload).decode("ascii"),
                            "payload_len": len(entry.payload),
                            "data": self.codec.unpack_reply(InavMSP(code), entry.payload),
                            "cached": True,
                            "diag": diag,
                        }
                    )
                    return

        self._queue_request(
            session,
            req_id=req.get("id"),
            code=code,
            payload=payload_bytes,
            timeout_s=timeout_s,
            key=key,
            force_version=force_version,
            scheduled=False,
            schedule_delay=None,
            throttle_ms=throttle_ms,
        )
        self._log(f"queued client_id={session.client_id} code={code} key={key} throttle_ms={throttle_ms}")

    def _build_payload(self, code: int, raw: Any, payload_struct: Any) -> bytes:
        if raw is not None:
            if not isinstance(raw, str):
                raise ValueError("raw payload must be base64 string")
            return base64.b64decode(raw.encode("ascii"))
        if payload_struct is None:
            spec = self.codec._get_spec(InavMSP(code))
            if spec.request.struct_fmt is not None:
                raise ValueError(f"{spec.name}: payload required")
            return b""
        if not isinstance(payload_struct, Mapping):
            raise ValueError("payload must be an object")
        return self.codec.pack_request(InavMSP(code), payload_struct)

    def _queue_request(
        self,
        session: Optional[ClientSession],
        *,
        req_id: Any,
        code: int,
        payload: bytes,
        timeout_s: float,
        key: str,
        force_version: Optional[int],
        scheduled: bool,
        schedule_delay: Optional[float],
        throttle_ms: float,
    ) -> None:

        with self.state_lock:
            pending = self.pending.get(key)
            if pending:
                if session:
                    if len(pending.waiters) >= MAX_PENDING_WAITERS:
                        self._send_error(session, req_id, "too many waiters for this request")
                        return
                    pending.waiters.append((session, req_id))
                return

            pending = PendingRequest(
                code=code,
                payload=payload,
                timeout_s=timeout_s,
                key=key,
                scheduled=scheduled,
                schedule_delay=schedule_delay,
                created=_now_s(),
                force_version=force_version,
                throttle_ms=throttle_ms,
            )
            if session:
                pending.waiters.append((session, req_id))
            self.pending[key] = pending

        try:
            self.send_queue.put_nowait(pending)
        except queue.Full:
            with self.state_lock:
                self.pending.pop(key, None)
            if session:
                self._send_error(session, req_id, "send queue full")
                return
            raise RuntimeError("send queue full")

    def _sender_loop(self) -> None:
        try:
            while not self.stop_evt.is_set():
                try:
                    pending = self.send_queue.get(timeout=REQUEST_QUEUE_POLL_S)
                except queue.Empty:
                    continue
                self._log(f"dequeue code={pending.code} key={pending.key} waiters={len(pending.waiters)}")
                start = _now_s()
                error: Optional[str] = None
                payload = b""
                try:
                    rsp_code, payload = self.transport.request(
                        pending.code,
                        pending.payload,
                        timeout=pending.timeout_s,
                        force_version=getattr(pending, "force_version", None),
                    )
                    if rsp_code != pending.code:
                        raise RuntimeError(f"unexpected response code {rsp_code} for {pending.code}")
                except Exception as exc:
                    error = str(exc)
                    self._log(f"request_error code={pending.code} key={pending.key} error={error}")
                    try:
                        diag_extra = getattr(self.transport, "last_diag", None)
                        if diag_extra:
                            self._log(f"request_error_diag code={pending.code} diag={diag_extra}")
                        reader_err = getattr(self.transport, "_reader_error", None)
                        if reader_err:
                            self._log(f"reader_error code={pending.code} err={reader_err!r}")
                        thread = getattr(self.transport, "_rx_thread", None)
                        if thread:
                            self._log(f"reader_alive={thread.is_alive()} name={thread.name}")
                    except Exception:
                        pass
                duration_ms = (_now_s() - start) * 1000.0
                with self.state_lock:
                    inflight_total = len(self.pending)
                    queue_depth = self.send_queue.qsize()
                server_info = {
                    "requests_per_min": self._requests_per_min(),
                    "inflight_total": inflight_total,
                    "queue_depth": queue_depth,
                    "reconnections": self.transport._reconnects,  # type: ignore[attr-defined]
                    "scheduler_failed": False,
                }
                if pending.waiters:
                    session = pending.waiters[0][0]
                    active_clients = max(1, len(self.clients))
                    limit = max(1, int(GLOBAL_REQ_LIMIT_PER_SEC / active_clients))
                    now = _now_s()
                    window = session.rate_window
                    while window and now - window[0] > RATE_WINDOW_S:
                        window.popleft()
                    rate_info = {
                        "current_per_sec": len(window) / RATE_WINDOW_S,
                        "limit_per_sec": float(limit),
                        "utilization": (len(window) / RATE_WINDOW_S) / float(limit),
                        "throttle_ms": pending.throttle_ms,
                    }
                else:
                    rate_info = {}
                with self.state_lock:
                    stats = self.code_stats.get(pending.code, {"count": 0.0, "avg_ms": 0.0})
                    count = stats["count"] + 1.0
                    avg = stats["avg_ms"]
                    avg = ((avg * stats["count"]) + duration_ms) / count
                    stats["count"] = count
                    stats["avg_ms"] = avg
                    self.code_stats[pending.code] = stats
                uptime = max(1.0, _now_s() - self.start_time)
                code_stat = {"avg_ms": avg, "requests_per_min": count * (60.0 / uptime)}
                diag = {
                    "code": pending.code,
                    "duration_ms": duration_ms,
                    "cached": False,
                    "cache_age_ms": None,
                    "scheduled": pending.scheduled,
                    "schedule_delay": pending.schedule_delay,
                    "attempt": 1,
                    "transport": "serial",
                    "timestamp": time.time(),
                    "server": server_info,
                    "client": {"id": [w[0].client_id for w in pending.waiters]},
                    "rate": rate_info,
                    "code_stats": code_stat,
                    "ok": error is None,
                }
                if error:
                    self._fail_pending(pending, error, diag)
                else:
                    self._complete_pending(pending, payload, diag)
                self._log(f"done code={pending.code} key={pending.key} ok={error is None} waiters={len(pending.waiters)} duration_ms={duration_ms:.2f}")
        except Exception as exc:
            self._log(f"fatal sender loop error: {exc!r}")
            self._fatal()

    def _requests_per_min(self) -> float:
        with self.state_lock:
            total = sum(stats.get("count", 0.0) for stats in self.code_stats.values())
        uptime = max(1.0, _now_s() - self.start_time)
        return total * (60.0 / uptime)

    def _complete_pending(self, pending: PendingRequest, payload: bytes, diag: Dict[str, Any]) -> None:
        cache_entry = CacheEntry(payload=payload, diag=diag, created=_now_s())
        with self.state_lock:
            self.cache[pending.key] = cache_entry
            self.pending.pop(pending.key, None)
        data = self.codec.unpack_reply(InavMSP(pending.code), payload)
        safe_data = _json_safe(data)
        safe_diag = _json_safe(diag)
        for session, req_id in pending.waiters:
            if session.closed:
                continue
            try:
                self._log(
                    f"send_ok client_id={session.client_id} id={req_id} code={pending.code} "
                    f"len={len(payload)} cached=False duration_ms={diag.get('duration_ms')}"
                )
                session.send_json(
                    {
                        "id": req_id,
                        "ok": True,
                        "code": pending.code,
                        "name": InavMSP(pending.code).name,
                        "payload_b64": base64.b64encode(payload).decode("ascii"),
                        "payload_len": len(payload),
                        "data": safe_data,
                        "cached": False,
                        "diag": safe_diag,
                    }
                )
            except OSError as exc:
                self._log(f"send_failure client_id={session.client_id} error={exc}")
                session.close()

    def _fail_pending(self, pending: PendingRequest, error: str, diag: Dict[str, Any]) -> None:
        with self.state_lock:
            self.pending.pop(pending.key, None)
        for session, req_id in pending.waiters:
            if session.closed:
                continue
            try:
                self._log(f"send_err client_id={session.client_id} id={req_id} code={pending.code} error={error}")
                session.send_json({"id": req_id, "ok": False, "error": error, "diag": _json_safe(diag)})
            except OSError as exc:
                self._log(f"send_failure client_id={session.client_id} error={exc}")
                session.close()

    def _send_error(self, session: ClientSession, req_id: Any, message: str) -> None:
        try:
            session.send_json({"id": req_id, "ok": False, "error": _json_safe(message)})
        except OSError as exc:
            self._log(f"send_failure client_id={session.client_id} error={exc}")
            session.close()

    def _scheduler_loop(self) -> None:
        try:
            while not self.stop_evt.is_set():
                now = _now_s()
                with self.state_lock:
                    entries = list(self.schedules.values())
                for entry in entries:
                    if now + SCHEDULER_TICK_S < entry.next_fire:
                        continue
                    with self.state_lock:
                        current = self.schedules.get(entry.code)
                        if current is None:
                            continue
                        current.next_fire = now + current.delay
                        delay = current.delay
                        payload_struct = current.payload
                        code = current.code
                    payload = self._build_payload(code, None, payload_struct)
                    key = _crc_key(code, payload)
                    self._queue_request(
                        None,
                        req_id=f"sched_{code}_{int(now*1000)}",
                        code=code,
                        payload=payload,
                        timeout_s=delay,
                        key=key,
                        force_version=None,
                        scheduled=True,
                        schedule_delay=delay,
                        throttle_ms=0.0,
                    )
                time.sleep(SCHEDULER_TICK_S)
        except Exception as exc:
            self._log(f"fatal scheduler loop error: {exc!r}")
            self._fatal()

    def _handle_sched_set(self, session: ClientSession, req: Dict[str, Any]) -> None:
        req_id = req.get("id")
        if "code" not in req or "delay" not in req:
            self._send_error(session, req_id, "code and delay are required for sched_set")
            return
        try:
            code = _pack_code(req["code"])
        except Exception as exc:
            self._send_error(session, req_id, f"invalid code: {exc}")
            return
        delay = float(req["delay"])
        if delay <= 0:
            self._send_error(session, req_id, "delay must be positive")
            return
        payload = req.get("payload")
        with self.state_lock:
            self.schedules[code] = ScheduleEntry(code=code, delay=delay, payload=payload, next_fire=_now_s() + delay)
        self._send_sched_get(session, req_id)

    def _handle_sched_remove(self, session: ClientSession, req: Dict[str, Any]) -> None:
        req_id = req.get("id")
        if "code" not in req:
            self._send_error(session, req_id, "code is required for sched_remove")
            return
        try:
            code = _pack_code(req["code"])
        except Exception as exc:
            self._send_error(session, req_id, f"invalid code: {exc}")
            return
        with self.state_lock:
            self.schedules.pop(code, None)
        self._send_sched_get(session, req_id)

    def _send_sched_get(self, session: ClientSession, req_id: Any) -> None:
        with self.state_lock:
            schedules = {InavMSP(code).name: {"delay": entry.delay} for code, entry in self.schedules.items()}
        session.send_json({"id": req_id, "ok": True, "schedules": schedules, "diag": {"server": {"requests_per_min": self._requests_per_min()}}})

    def _log(self, msg: str) -> None:
        sys.stdout.write(f"[msp_server] {msg}\n")
        sys.stdout.flush()

    def _fatal(self) -> None:
        sys.stdout.write("[msp_server] fatal error, exiting\n")
        sys.stdout.flush()
        os._exit(1)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MSP TCP broker")
    parser.add_argument("--host", required=True, help="TCP host to bind")
    parser.add_argument("--port", required=True, type=int, help="TCP port to bind")
    parser.add_argument("--serial", dest="serial_port", help="Serial device for FC")
    parser.add_argument("--baudrate", type=int, help="Baudrate for serial")
    parser.add_argument("--tcp-endpoint", help="Existing TCP endpoint for FC (HOST:PORT)")
    parser.add_argument("--fc-timeout", type=float, default=DEFAULT_FC_TIMEOUT, help="FC request timeout seconds")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> None:
    args = parse_args(argv)
    _redirect_output(Path(DEFAULT_LOG_PATH))
    server = MSPRequestServer(
        host=args.host,
        port=args.port,
        serial_port=args.serial_port,
        baudrate=args.baudrate,
        tcp_endpoint=args.tcp_endpoint,
        fc_timeout=args.fc_timeout,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
