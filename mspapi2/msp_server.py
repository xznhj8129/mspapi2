#!/usr/bin/env python3
"""
Multi-client MSP API server.

Listens on a TCP socket, accepts JSON-per-line requests, and serializes access
to the MSPSerial transport so that multiple independent clients can share a
single FC connection safely.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import logging
import signal
import sys
import time
import uuid
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .lib import InavMSP
from .mspcodec import MSPCodec
from .msp_serial import MSPSerial


RATE_LIMIT_TOTAL = 200


def _now() -> float:
    return time.monotonic()


def _json_dumps(obj: Dict[str, Any]) -> bytes:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8") + b"\n"


def _json_safe(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, (bytes, bytearray)):
        data = bytes(value)
        try:
            text = data.decode("utf-8").rstrip("\x00")
            if text and all(32 <= ord(ch) < 127 or ch in "\r\n\t" for ch in text):
                return text
        except UnicodeDecodeError:
            pass
        return base64.b64encode(data).decode("ascii")
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return str(value)


@dataclass
class SerialResponse:
    code: int
    payload: bytes
    duration_ms: float
    cached: bool
    cache_age_ms: Optional[float] = None
    scheduled: bool = False
    schedule_delay: Optional[float] = None


class MSPRequestServer:
    # Requests that should never be cached/deduped, even if payload repeats.
    NO_CACHE_CODES = {
        int(InavMSP.MSP_SET_RAW_RC),
    }

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 9000,
        serial_port: Optional[str] = "/dev/ttyACM0",
        tcp_endpoint: Optional[str] = None,
        baudrate: int = 115200,
        read_timeout: float = 0.05,
        write_timeout: float = 0.25,
        codec_path: Optional[str] = None,
        cache_ttl: float = 1.0,
        default_timeout: float = 1.0,
    ) -> None:
        if not serial_port and not tcp_endpoint:
            raise ValueError("Either serial_port or tcp_endpoint must be provided")
        if serial_port and tcp_endpoint:
            raise ValueError("Provide serial_port or tcp_endpoint, not both")

        self.host = host
        self.port = port
        self.cache_ttl = max(0.0, cache_ttl)
        self.default_timeout = max(0.1, default_timeout)
        schema_path = Path(codec_path) if codec_path else Path(__file__).with_name("lib") / "msp_messages.json"
        self._codec = MSPCodec.from_json_file(str(schema_path))

        serial_args = dict(
            baudrate=baudrate,
            read_timeout=read_timeout,
            write_timeout=write_timeout,
            keepalive_code=int(InavMSP.MSP_API_VERSION),
            keepalive_interval=5.0,
            keepalive_timeout=0.5,
        )
        if tcp_endpoint:
            self._serial = MSPSerial(tcp_endpoint, tcp=True, **serial_args)
        else:
            self._serial = MSPSerial(serial_port or "", **serial_args)

        self._server: Optional[asyncio.AbstractServer] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._sessions: List["ClientSession"] = []
        self._inflight: Dict[Tuple[int, bytes], List[asyncio.Future[SerialResponse]]] = {}
        self._cache: Dict[Tuple[int, bytes], Tuple[float, SerialResponse, Dict[str, Any]]] = {}
        self._lock = asyncio.Lock()
        self._code_stats: Dict[int, Dict[str, Any]] = {}
        self._total_requests = 0
        self._request_times = deque()
        self._clients: Dict[str, Dict[str, Any]] = {}
        self._client_lock = asyncio.Lock()
        self._total_inflight = 0
        self._schedulers: Dict[int, Dict[str, Any]] = {}
        self._sched_lock = asyncio.Lock()
        self._scheduler_task: Optional[asyncio.Task] = None

    @property
    def codec(self) -> MSPCodec:
        return self._codec

    async def start(self) -> None:
        if self._server:
            return
        loop = asyncio.get_running_loop()
        self._loop = loop
        await loop.run_in_executor(None, self._serial.open)
        self._server = await asyncio.start_server(self._handle_client, self.host, self.port)
        logging.info("MSP server listening on %s:%s", self.host, self.port)
        self._scheduler_task = asyncio.create_task(self._scheduler_loop())

    async def serve_until_cancelled(self) -> None:
        if not self._server:
            raise RuntimeError("Server not started")
        async with self._server:
            await self._server.serve_forever()

    async def close(self) -> None:
        logging.info("Shutting down MSP server")
        if self._scheduler_task:
            self._scheduler_task.cancel()
            try:
                await self._scheduler_task
            except Exception:
                pass
            self._scheduler_task = None
        if self._server:
            self._server.close()
            await self._server.wait_closed()
            self._server = None

        for session in list(self._sessions):
            await session.close()
        self._sessions.clear()

        await asyncio.get_running_loop().run_in_executor(None, self._serial.close)

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        session = ClientSession(self, reader, writer)
        self._sessions.append(session)
        try:
            await session.run()
        finally:
            self._sessions.remove(session)

    async def dispatch_request(
        self,
        code: InavMSP,
        payload: bytes,
        *,
        timeout: float,
        cacheable: bool,
        scheduled: bool = False,
        schedule_delay: Optional[float] = None,
    ) -> SerialResponse:
        if int(code) in self.NO_CACHE_CODES:
            cacheable = False

        key = (int(code), payload)
        now = _now()

        if cacheable:
            cached = self._cache.get(key)
            if cached:
                cached_ts, stored_resp, meta = cached
                fresh = self.cache_ttl <= 0 or (now - cached_ts) <= self.cache_ttl
                if fresh:
                    age_ms = (_now() - cached_ts) * 1000.0
                    resp = SerialResponse(
                        stored_resp.code,
                        stored_resp.payload,
                        stored_resp.duration_ms,
                        cached=True,
                        cache_age_ms=age_ms,
                        scheduled=meta.get("scheduled", False),
                        schedule_delay=meta.get("delay"),
                    )
                    self._record_latency(int(code), stored_resp.duration_ms)
                    return resp

        inflight_key = key
        loop = asyncio.get_running_loop()
        fut: asyncio.Future[SerialResponse] = loop.create_future()
        start_request = False

        async with self._lock:
            waiters = self._inflight.get(inflight_key)
            if waiters is None:
                self._inflight[inflight_key] = [fut]
                start_request = True
            else:
                waiters.append(fut)

        if start_request:
            asyncio.create_task(
                self._fulfill_request(
                    inflight_key,
                    code,
                    payload,
                    timeout,
                    cacheable,
                    scheduled=scheduled,
                    schedule_delay=schedule_delay,
                )
            )

        return await fut

    async def _fulfill_request(
        self,
        inflight_key: Tuple[int, bytes],
        code: InavMSP,
        payload: bytes,
        timeout: float,
        cacheable: bool,
        *,
        scheduled: bool = False,
        schedule_delay: Optional[float] = None,
    ) -> None:
        try:
            loop = asyncio.get_running_loop()
            start = _now()
            rsp_code, rsp_payload = await loop.run_in_executor(
                None, lambda: self._serial.request(int(code), payload, timeout=timeout)
            )
            duration_ms = (_now() - start) * 1000.0
            resp = SerialResponse(
                rsp_code,
                rsp_payload,
                duration_ms,
                cached=False,
                scheduled=scheduled,
                schedule_delay=schedule_delay,
            )
            self._record_latency(int(code), duration_ms)
            if cacheable:
                self._cache[(int(code), payload)] = (
                    _now(),
                    resp,
                    {"scheduled": scheduled, "delay": schedule_delay},
                )
            waiters = await self._pop_inflight(inflight_key)
            for fut in waiters:
                if not fut.done():
                    fut.set_result(resp)
        except Exception as exc:
            logging.exception("Internal dispatch error for code %s", code)
            waiters = await self._pop_inflight(inflight_key)
            for fut in waiters:
                if not fut.done():
                    fut.set_exception(exc)

    async def _pop_inflight(
        self, inflight_key: Tuple[int, bytes]
    ) -> List[asyncio.Future[SerialResponse]]:
        async with self._lock:
            return self._inflight.pop(inflight_key, [])

    def _prune_cache(self, now: float) -> None:
        # Cache entries are preserved to provide the last known response even if stale.
        return

    def requests_per_min(self) -> int:
        cutoff = _now() - 60.0
        while self._request_times and self._request_times[0] < cutoff:
            self._request_times.popleft()
        return len(self._request_times)

    def _record_latency(self, code: int, duration_ms: float) -> None:
        stats = self._code_stats.setdefault(code, {
            "count": 0,
            "total_ms": 0.0,
            "last_ms": 0.0,
            "times": deque(),
        })
        stats["count"] += 1
        stats["total_ms"] += duration_ms
        stats["last_ms"] = duration_ms
        now = _now()
        times = stats["times"]
        times.append(now)
        cutoff = now - 60.0
        while times and times[0] < cutoff:
            times.popleft()

    def _code_stats_snapshot(self, code: int) -> Dict[str, Any]:
        stats = self._code_stats.get(code)
        if not stats:
            return {
                "count": 0,
                "avg_ms": None,
                "requests_per_min": 0,
                "last_ms": None,
            }
        times = stats["times"]
        cutoff = _now() - 60.0
        while times and times[0] < cutoff:
            times.popleft()
        rpm = len(times)
        avg_ms = stats["total_ms"] / stats["count"] if stats["count"] else None
        return {
            "count": stats["count"],
            "avg_ms": avg_ms,
            "requests_per_min": rpm,
            "last_ms": stats.get("last_ms"),
        }

    async def set_schedule(self, code: int, payload: bytes, delay: float, timeout: float) -> Dict[str, Any]:
        if delay <= 0:
            async with self._sched_lock:
                self._schedulers.pop(code, None)
            return {}
        entry = {
            "delay": delay,
            "payload": payload,
            "timeout": timeout,
            "last_time": None,
            "next_time": _now() + delay,
        }
        async with self._sched_lock:
            self._schedulers[code] = entry
        return {
            "delay": delay,
            "payload_b64": base64.b64encode(payload).decode("ascii"),
            "last_time": entry["last_time"],
        }

    async def get_schedules(self) -> Dict[str, Any]:
        async with self._sched_lock:
            result: Dict[str, Any] = {}
            for code, entry in self._schedulers.items():
                try:
                    name = InavMSP(code).name
                except Exception:
                    name = None
                result[name] = {
                    "delay": entry["delay"],
                    "payload_b64": base64.b64encode(entry["payload"]).decode("ascii"),
                    "last_time": entry["last_time"]
                }
            return result

    async def remove_schedule(self, code: int) -> None:
        async with self._sched_lock:
            self._schedulers.pop(code, None)

    async def _scheduler_loop(self) -> None:
        try:
            while True:
                await asyncio.sleep(0.05)
                now = _now()
                due: List[Tuple[int, Dict[str, Any]]] = []
                async with self._sched_lock:
                    for code, entry in self._schedulers.items():
                        if now >= entry["next_time"]:
                            entry["next_time"] = now + max(0.01, entry["delay"])
                            entry["last_time"] = now
                            due.append((code, dict(entry)))
                for code, entry in due:
                    payload = entry["payload"]
                    timeout = entry.get("timeout", self.default_timeout)
                    asyncio.create_task(
                        self.dispatch_request(
                            InavMSP(code),
                            payload,
                            timeout=timeout,
                            cacheable=True,
                            scheduled=True,
                            schedule_delay=entry.get("delay"),
                        )
                    )
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            logging.exception("Scheduler loop crashed")

    def encode_payload(self, code: InavMSP, body: Dict[str, Any]) -> bytes:
        if "raw" in body and body["raw"] is not None:
            try:
                return base64.b64decode(body["raw"])
            except Exception as exc:
                raise ValueError("raw payload must be base64-encoded") from exc
        payload = body.get("payload")
        if payload is None:
            return b""
        try:
            return self._codec.pack_request(code, payload)
        except Exception as exc:
            raise ValueError(f"pack_request failed: {exc}") from exc

    def decode_payload(self, code: InavMSP, payload: bytes) -> Optional[Any]:
        try:
            return self._codec.unpack_reply(code, payload)
        except Exception:
            return None

    @staticmethod
    def resolve_code(value: Any) -> InavMSP:
        if isinstance(value, str):
            try:
                return InavMSP[value]
            except KeyError as exc:
                raise ValueError(f"Unknown MSP code name '{value}'") from exc
        try:
            return InavMSP(int(value))
        except Exception as exc:
            raise ValueError(f"Invalid MSP code '{value}'") from exc

    async def register_client(self, session_id: str, peer: Optional[Tuple[str, int]]) -> None:
        address, port = (peer or ("?", 0)) if peer else ("?", 0)
        info = {
            "session_id": session_id,
            "client_id": session_id,
            "address": address,
            "port": port,
            "connected_at": _now(),
            "last_seen": _now(),
            "public_key": None,
        }
        async with self._client_lock:
            self._clients[session_id] = info

    async def update_client(self, session_id: str, *, client_id: Optional[str] = None) -> None:
        async with self._client_lock:
            info = self._clients.get(session_id)
            if not info:
                return
            if client_id:
                info["client_id"] = str(client_id)
            info["last_seen"] = _now()

    async def unregister_client(self, session_id: str) -> None:
        async with self._client_lock:
            self._clients.pop(session_id, None)

    def get_client_info(self, session_id: str) -> Dict[str, Any]:
        info = self._clients.get(session_id)
        if not info:
            return {"session_id": session_id}
        return dict(info)

    def per_client_rate_limit(self) -> int:
        with_clients = len(self._clients)
        limit = int(RATE_LIMIT_TOTAL / max(1, with_clients))
        return max(1, limit)


class ClientSession:
    def __init__(self, server: MSPRequestServer, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.server = server
        self.reader = reader
        self.writer = writer
        self.addr = writer.get_extra_info("peername")
        self._closed = False
        self.session_id = uuid.uuid4().hex
        self.client_id: Optional[str] = None
        self._rate_window = deque()
        self._last_rate: Optional[Dict[str, Any]] = None
        self._pending = 0

    async def run(self) -> None:
        logging.info("Client connected: %s", self.addr)
        await self.server.register_client(self.session_id, self.addr)
        logging.info("Client connected %s session=%s", self.addr, self.session_id)
        try:
            while True:
                line = await self.reader.readline()
                if not line:
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    req = json.loads(line)
                except json.JSONDecodeError:
                    await self._send({"ok": False, "error": "invalid json"})
                    continue
                await self._handle_request(req)
        except (asyncio.IncompleteReadError, ConnectionResetError, BrokenPipeError):
            pass
        finally:
            await self.close()
            await self.server.unregister_client(self.session_id)
            logging.info(
                "Client disconnected %s session=%s client_id=%s",
                self.addr,
                self.session_id,
                self.client_id,
            )

    async def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.writer.close()
            await self.writer.wait_closed()
        except Exception:
            pass

    def _basic_diag(self) -> Dict[str, Any]:
        return {
            "server": {
                "requests_total": self.server._total_requests,
                "reconnections": getattr(self.server._serial, "reconnects", 0),
                "requests_per_min": self.server.requests_per_min(),
                "inflight_total": self.server._total_inflight,
                "rate_limit_per_client": self.server.per_client_rate_limit(),
            },
            "client": self._client_diag_meta(),
        }

    def _client_diag_meta(self) -> Dict[str, Any]:
        info = dict(self.server.get_client_info(self.session_id))
        info["pending"] = self._pending
        if self._last_rate:
            info["rate"] = self._last_rate
        return info

    async def _respect_rate_limit(self) -> None:
        window = self._rate_window
        while True:
            now = _now()
            window.append(now)
            cutoff = now - 1.0
            while window and window[0] < cutoff:
                window.popleft()
            current = len(window)
            limit = self.server.per_client_rate_limit()
            if current <= limit:
                self._last_rate = {
                    "limit_per_sec": limit,
                    "current_per_sec": current,
                    "utilization": round(current / limit, 3) if limit else None,
                    "pending": self._pending,
                    "throttle_ms": 0,
                }
                return
            window.pop()
            wait = (window[0] + 1.0 - now) if window else 0.05
            wait = max(wait, 0.01)
            self._last_rate = {
                "limit_per_sec": limit,
                "current_per_sec": len(window),
                "throttle_ms": wait * 1000.0,
                "utilization": round(min(len(window), limit) / limit, 3) if limit else None,
                "pending": self._pending,
            }
            await asyncio.sleep(wait)

    async def _handle_request(self, req: Dict[str, Any]) -> None:
        await self._respect_rate_limit()
        self._pending += 1
        self.server._total_inflight += 1
        try:
            req_id = req.get("id")
            client_label = req.get("client_id")
            if client_label:
                new_id = str(client_label)
                if new_id != self.client_id:
                    self.client_id = new_id
                    logging.info(
                        "Client session=%s labeled as %s",
                        self.session_id,
                        self.client_id,
                    )
            await self.server.update_client(self.session_id, client_id=self.client_id)

            action = req.get("action")

            timeout_ms = max(100.0, float(req.get("timeout_ms", self.server.default_timeout * 1000.0)))
            timeout = timeout_ms / 1000.0
            cacheable = not bool(req.get("no_cache"))

            if action:
                handled = await self._handle_action(str(action).lower(), req, req_id, timeout_ms)
                if handled:
                    return

            try:
                code = MSPRequestServer.resolve_code(req.get("code"))
            except Exception as exc:
                await self._send({"id": req_id, "ok": False, "error": str(exc)})
                return

            try:
                payload = self.server.encode_payload(code, req)
            except Exception as exc:
                await self._send({"id": req_id, "ok": False, "error": str(exc)})
                return

            try:
                resp = await self.server.dispatch_request(
                    code,
                    payload,
                    timeout=timeout,
                    cacheable=cacheable,
                )
            except Exception as exc:
                logging.exception(
                    "Dispatch failure for client %s request %s", self.session_id, req
                )
                await self._send({"id": req_id, "ok": False, "error": str(exc)})
                return

            decoded = self.server.decode_payload(code, resp.payload)
            self.server._total_requests += 1
            self.server._request_times.append(_now())
            requests_per_min = self.server.requests_per_min()

            try:
                name = InavMSP(resp.code).name
            except Exception:
                name = f"MSP_{resp.code}"

            diag = {
                "code": resp.code,
                "duration_ms": resp.duration_ms,
                "cached": resp.cached,
                "cache_age_ms": resp.cache_age_ms,
                "scheduled": resp.scheduled,
                "schedule_delay": resp.schedule_delay,
                "code_stats": self.server._code_stats_snapshot(resp.code),
                "server": {
                    "requests_total": self.server._total_requests,
                    "reconnections": getattr(self.server._serial, "reconnects", 0),
                    "requests_per_min": requests_per_min,
                    "inflight_total": self.server._total_inflight,
                    "rate_limit_per_client": self.server.per_client_rate_limit(),
                },
            }
            diag["client"] = self._client_diag_meta()

            reply: Dict[str, Any] = {
                "id": req_id,
                "ok": True,
                "code": resp.code,
                "name": name,
                "payload_b64": base64.b64encode(resp.payload).decode("ascii"),
                "payload_len": len(resp.payload),
                "duration_ms": round(resp.duration_ms, 3),
                "cached": resp.cached,
                "diag": diag,
            }
            if decoded is not None:
                reply["data"] = _json_safe(decoded)
            await self._send(reply)
        finally:
            self._pending = max(self._pending - 1, 0)
            self.server._total_inflight = max(self.server._total_inflight - 1, 0)

    async def _handle_action(self, action: str, req: Dict[str, Any], req_id: Any, timeout_ms: float) -> bool:
        if action == "sched_get":
            schedules = await self.server.get_schedules()
            await self._send({"id": req_id, "ok": True, "schedules": schedules, "diag": self._basic_diag()})
            return True
        if action == "sched_set":
            try:
                code = MSPRequestServer.resolve_code(req.get("code"))
            except Exception as exc:
                await self._send({"id": req_id, "ok": False, "error": str(exc)})
                return True
            delay = float(req.get("delay", 0.0))
            try:
                payload = self.server.encode_payload(code, req)
            except Exception as exc:
                await self._send({"id": req_id, "ok": False, "error": str(exc)})
                return True
            schedule = await self.server.set_schedule(int(code), payload, delay, timeout_ms / 1000.0)
            reply = {"id": req_id, "ok": True, "schedule": schedule, "diag": self._basic_diag()}
            await self._send(reply)
            return True
        if action == "sched_remove":
            try:
                code = MSPRequestServer.resolve_code(req.get("code"))
            except Exception as exc:
                await self._send({"id": req_id, "ok": False, "error": str(exc)})
                return True
            await self.server.remove_schedule(int(code))
            await self._send({"id": req_id, "ok": True, "removed": int(code), "diag": self._basic_diag()})
            return True
        return False

    async def _send(self, payload: Dict[str, Any]) -> None:
        try:
            self.writer.write(_json_dumps(payload))
            await self.writer.drain()
        except Exception:
            await self.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Multi-client MSP API server")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9000, help="Bind port (default: 9000)")
    parser.add_argument("--serial", default="/dev/ttyACM0", help="Serial device path")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--tcp-endpoint", help="Alternative FC endpoint in HOST:PORT form")
    parser.add_argument("--cache-ttl", type=float, default=1.0, help="Seconds to reuse identical replies")
    parser.add_argument("--timeout", type=float, default=1.0, help="Default MSP timeout in seconds")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args()


async def run_server(args: argparse.Namespace) -> None:
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="[%(asctime)s] %(levelname)s: %(message)s",
        stream=sys.stdout,
        force=True,
    )
    server = MSPRequestServer(
        host=args.host,
        port=args.port,
        serial_port=None if args.tcp_endpoint else args.serial,
        tcp_endpoint=args.tcp_endpoint,
        baudrate=args.baudrate,
        cache_ttl=args.cache_ttl,
        default_timeout=args.timeout,
    )
    await server.start()

    stop_event = asyncio.Event()

    def _stop(*_: Any) -> None:
        if not stop_event.is_set():
            stop_event.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _stop)
        except NotImplementedError:
            signal.signal(sig, lambda *_: _stop())

    await stop_event.wait()
    await server.close()


def main() -> None:
    args = parse_args()
    asyncio.run(run_server(args))


if __name__ == "__main__":
    main()
