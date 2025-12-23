# msp_serial.py
# Minimal, robust MSP (V1 + V2) serial transport.
# - Handles framing/parsing in a background reader thread
# - Provides send() and request() (send + await matching response)
# - Thread-safe; no asyncio; serial-only.
#
# Usage:
#   msp = MSPSerial("/dev/ttyACM0", 115200, read_timeout=0.05)
#   msp.open()
#   # fire-and-forget
#   msp.send(101, b"\x01\x02")
#   # request/response (waits for same code back)
#   code, payload = msp.request(101, b"", timeout=1.0)
#   msp.close()

from __future__ import annotations
import threading
import time
import serial
import sys
from dataclasses import dataclass
from queue import Queue, Empty, Full
from typing import Optional, Tuple, Callable, Dict, Deque, List, Any
from collections import deque
import enum
from .lib import *

# ------------------------
# Utilities / constants
# ------------------------

MSP_V1_SYNC_1 = 0x24  # '$'
MSP_V1_SYNC_2 = 0x4D  # 'M'
MSP_V2_SYNC_2 = 0x58  # 'X'
DIR_TO_FC        = 0x3C  # '<'
DIR_FROM_FC      = 0x3E  # '>'
UNSUPPORTED_FLAG = 0x21  # '!'

# Jumbo frame sentinel/limit per V1 semantics
JUMBO_LIMIT_V1 = 255
MAX_PAYLOAD_LEN = 8192

# ------------------------
# CRC8 DVB-S2 for MSP v2
# ------------------------
def crc8_dvb_s2(crc: int, byte: int) -> int:
    crc ^= (byte & 0xFF)
    for _ in range(8):
        if (crc & 0x80) != 0:
            crc = ((crc << 1) & 0xFF) ^ 0xD5
        else:
            crc = (crc << 1) & 0xFF
    return crc & 0xFF

# ------------------------
# Public message type
# ------------------------
@dataclass
class MSPMessage:
    version: int     # 1 or 2
    code: int        # 0..65535
    payload: bytes
    unsupported: bool = False

class MSPResult(enum.IntEnum):
    MSP_RESULT_ACK = 1
    MSP_RESULT_ERROR = -1
    MSP_RESULT_NO_REPLY = 0


class MSPUnsupportedError(RuntimeError):
    pass


# ------------------------
# Parser state machine
# ------------------------
class _MSPParser:
    """
    Incremental MSP V1/V2 parser.
    Feed bytes via feed(data) repeatedly; it yields zero or more MSPMessage.
    Designed to be re-entrant and resync quickly on garbage.
    """
    def __init__(self) -> None:
        self.buf = deque()  # type: Deque[int]
        self.reset()

    def reset(self):
        self.state = 0
        self.version = 0
        self.direction = -1
        self.flags = 0
        self.code = 0
        self.length_expected = 0
        self.length_received = 0
        self.checksum = 0
        self.payload = bytearray()
        self.is_jumbo = False
        self.unsupported = False

    def feed(self, data: bytes) -> List[MSPMessage]:
        for b in data:
            self.buf.append(b)
        out: List[MSPMessage] = []
        while True:
            msg = self._try_parse_one()
            if msg is None:
                break
            out.append(msg)
        return out

    def _pop(self) -> Optional[int]:
        try:
            return self.buf.popleft()
        except IndexError:
            return None

    def _peek(self) -> Optional[int]:
        try:
            return self.buf[0]
        except IndexError:
            return None

    def _try_parse_one(self) -> Optional[MSPMessage]:
        # This is essentially the same logical flow as your original,
        # but kept tight, iterative, and resilient to desync.
        while True:
            if self.state == 0:
                b = self._pop()
                if b is None:
                    return None
                if b == MSP_V1_SYNC_1:  # '$'
                    self.state = 1
                # else keep scanning for '$'

            elif self.state == 1:
                b = self._pop()
                if b is None:
                    return None
                if b == MSP_V1_SYNC_2:      # 'M' -> V1
                    self.version = 1
                    self.state = 2
                elif b == MSP_V2_SYNC_2:    # 'X' -> V2
                    self.version = 2
                    self.state = 2
                else:
                    # wrong second sync; resync
                    self.state = 0

            elif self.state == 2:
                # direction or error flag in both V1/V2
                b = self._pop()
                if b is None:
                    return None

                if b == UNSUPPORTED_FLAG:
                    self.direction = 1
                    self.unsupported = True
                elif b == DIR_FROM_FC:
                    self.direction = 1
                elif b == DIR_TO_FC:
                    self.direction = 0
                else:
                    # invalid; resync
                    self.state = 0
                    continue

                if self.version == 1:
                    self.state = 3  # length next
                else:
                    self.state = 2_1  # flags next

            elif self.state == 2_1:
                # V2 flags (currently ignored)
                b = self._pop()
                if b is None:
                    return None
                self.flags = b & 0xFF
                self.state = 2_2  # code L

            elif self.state == 2_2:
                b = self._pop()
                if b is None:
                    return None
                self.code = b & 0xFF
                self.state = 2_3  # code H

            elif self.state == 2_3:
                b = self._pop()
                if b is None:
                    return None
                self.code |= (b & 0xFF) << 8
                self.state = 3_1  # len L

            elif self.state == 3:
                # V1: length byte; also start checksum with len ^ code
                b = self._pop()
                if b is None:
                    return None
                self.length_expected = b & 0xFF
                self.checksum = b & 0xFF
                self.is_jumbo = (self.length_expected == JUMBO_LIMIT_V1)
                self.state = 4  # code next

            elif self.state == 3_1:
                # V2: length L
                b = self._pop()
                if b is None:
                    return None
                self.length_expected = b & 0xFF
                self.state = 3_2  # len H

            elif self.state == 3_2:
                # V2: length H
                b = self._pop()
                if b is None:
                    return None
                self.length_expected |= (b & 0xFF) << 8
                if self.length_expected > MAX_PAYLOAD_LEN:
                    raise ValueError(f"MSP V2 payload length {self.length_expected} exceeds limit {MAX_PAYLOAD_LEN}")
                self.payload = bytearray()
                self.length_received = 0
                # go to payload or checksum
                if self.length_expected > 0:
                    self.state = 7
                else:
                    self.state = 9  # checksum immediately

            elif self.state == 4:
                # V1: code, update XOR checksum
                b = self._pop()
                if b is None:
                    return None
                self.code = b & 0xFF
                self.checksum ^= self.code
                if self.length_expected > 0:
                    if self.is_jumbo:
                        self.state = 5  # jumbo size L
                    else:
                        self.payload = bytearray()
                        self.length_received = 0
                        self.state = 7   # payload
                else:
                    self.state = 9  # checksum next

            elif self.state == 5:
                # V1: jumbo size L (true payload length)
                b = self._pop()
                if b is None:
                    return None
                self.length_expected = b & 0xFF
                self.checksum ^= (b & 0xFF)
                self.state = 6

            elif self.state == 6:
                # V1: jumbo size H
                b = self._pop()
                if b is None:
                    return None
                self.length_expected += (b & 0xFF) << 8
                self.checksum ^= (b & 0xFF)
                if self.length_expected > MAX_PAYLOAD_LEN:
                    raise ValueError(f"MSP V1 payload length {self.length_expected} exceeds limit {MAX_PAYLOAD_LEN}")
                self.payload = bytearray()
                self.length_received = 0
                self.state = 7

            elif self.state == 7:
                # Payload byte
                b = self._pop()
                if b is None:
                    return None
                self.payload.append(b & 0xFF)
                if self.version == 1:
                    self.checksum ^= (b & 0xFF)
                else:
                    # v2 checksum handled in state 9 using crc8 over fields
                    pass
                self.length_received += 1
                if self.length_received == self.length_expected:
                    self.state = 9  # checksum next

            elif self.state == 9:
                # Final checksum byte
                b = self._pop()
                if b is None:
                    return None
                chksum = b & 0xFF
                ok = False
                if self.version == 1:
                    ok = (self.checksum & 0xFF) == chksum
                else:
                    # v2 CRC8 over: flags, code L, code H, len L, len H, payload...
                    c = 0
                    c = crc8_dvb_s2(c, self.flags & 0xFF)
                    c = crc8_dvb_s2(c, self.code & 0xFF)
                    c = crc8_dvb_s2(c, (self.code >> 8) & 0xFF)
                    c = crc8_dvb_s2(c, self.length_expected & 0xFF)
                    c = crc8_dvb_s2(c, (self.length_expected >> 8) & 0xFF)
                    for pb in self.payload:
                        c = crc8_dvb_s2(c, pb & 0xFF)
                    ok = (c & 0xFF) == chksum

                if ok:
                    msg = MSPMessage(
                        version=self.version,
                        code=self.code,
                        payload=bytes(self.payload),
                        unsupported=self.unsupported,
                    )
                    self.reset()
                    return msg
                else:
                    # bad frame -> resync hard
                    self.reset()
                    # continue to look for next '$'
            else:
                # Unknown state -> reset
                self.reset()


# ------------------------
# Serial / socket transport
# ------------------------
class MSPSerial:
    """
    MSP transport over a serial port or TCP socket:
      - open()/close()
      - send(code, payload=b'', force_version=None)
      - request(code, payload=b'', timeout=1.0, force_version=None) -> MSPReply

    Background reader thread parses V1/V2 frames and pushes them
    into a per-code queue (and an "all" queue) for matching.
    """
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        read_timeout: float = 0.05,
        write_timeout: float = 0.2,
        min_gap_s: float = 0.005,
        rx_queue_size: int = 1024,
        tcp: bool = False,
        udp: bool = False,
        *,
        keepalive_code: Optional[int] = None,
        keepalive_payload: bytes = b"",
        keepalive_interval: float = 5.0,
        keepalive_timeout: float = 1.0,
        max_retries: int = 3,
        reconnect_delay: float = 0.5,
        log_path: Optional[str] = "msp.log",
    ) -> None:
        """
        port: serial device path (e.g. '/dev/ttyACM0') or host:port for TCP when tcp=True
        min_gap_s: minimum spacing between writes to avoid FC overload
        """
        self.port = port
        self.baudrate = baudrate
        self.read_timeout = read_timeout
        self.write_timeout = write_timeout
        self.min_gap_s = max(0.0, float(min_gap_s))
        self._use_tcp = tcp
        self._use_udp = udp
        if tcp:
            self.transport = "tcp"
        elif udp:
            self.transport = "udp"
        else:
            self.transport = "serial"

        self._keepalive_code = int(keepalive_code) if keepalive_code is not None else None
        self._keepalive_payload = bytes(keepalive_payload)
        self._keepalive_interval = max(0.0, float(keepalive_interval)) if keepalive_code is not None else 0.0
        self._keepalive_timeout = max(0.1, float(keepalive_timeout))
        self._max_retries = max(1, int(max_retries))
        self._reconnect_delay = max(0.1, float(reconnect_delay))
        self._log_path = log_path
        self._log_lock = threading.Lock()
        self._log_fp = None
        self._log_enabled = bool(log_path)

        self._ser: Optional[serial.SerialBase] = None  # type: ignore[attr-defined]
        self._rx_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._reader_error: Optional[BaseException] = None

        self._parser = _MSPParser()

        # Queues for received messages:
        self._rx_all = Queue(maxsize=rx_queue_size)
        self._rx_by_code: Dict[int, Queue] = {}

        # Optional unsolicited handler: Callable[[MSPMessage], None]
        self.on_message: Optional[Callable[[MSPMessage], None]] = None

        # Locks
        self._wlock = threading.Lock()
        self._q_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._running_keepalive = False

        self._last_write_ts = 0.0
        self._last_activity = time.monotonic()
        self.last_diag: Optional[Dict[str, Any]] = None
        self._reconnects = 0

    # ---------- lifecycle ----------

    def open(self) -> None:
        if self._ser and self._ser.is_open and self._rx_thread and self._rx_thread.is_alive():
            return
        self._reader_error = None
        if self._use_tcp:
            url = self.port
            if "://" not in url:
                url = f"socket://{url}"
            self._ser = serial.serial_for_url(  # type: ignore[attr-defined]
                url,
                timeout=self.read_timeout,
                write_timeout=self.write_timeout,
            )
        else:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.read_timeout,
                write_timeout=self.write_timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
        # Clear buffers
        if hasattr(self._ser, "reset_input_buffer"):
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
        if hasattr(self._ser, "reset_output_buffer"):
            try:
                self._ser.reset_output_buffer()
            except Exception:
                pass

        # Start reader thread
        self._stop_evt.clear()
        self._rx_thread = threading.Thread(target=self._reader_loop, name="MSPSerialReader", daemon=True)
        self._rx_thread.start()
        self._last_activity = time.monotonic()

    def close(self) -> None:
        self._stop_evt.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        self._rx_thread = None
        self._running_keepalive = False
        self._reader_error = None

        if self._ser:
            try:
                if self._ser.is_open:
                    self._ser.close()
            finally:
                self._ser = None

        # Drain queues
        with self._q_lock:
            self._rx_all = Queue(maxsize=self._rx_all.maxsize)
            self._rx_by_code.clear()
        if self._log_fp:
            try:
                self._log_fp.flush()
                self._log_fp.close()
            finally:
                self._log_fp = None

    # ---------- public I/O ----------

    def send(self, code: int, payload: bytes = b"", force_version: Optional[int] = None) -> int:
        """
        Encode and write a single MSP frame to the port.
        Returns number of bytes written.
        """
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial port is not open")

        if payload is None:
            payload = b""
        if not isinstance(payload, (bytes, bytearray)):
            raise TypeError("payload must be bytes or bytearray")
        if len(payload) > MAX_PAYLOAD_LEN:
            raise ValueError(f"payload length {len(payload)} exceeds limit {MAX_PAYLOAD_LEN}")

        ver = self._choose_version(code, payload, force_version)
        frame = self._encode(ver, code, payload)
        self._log_io("OUT", frame)

        # Respect min gap between writes to avoid overwhelming FC
        with self._wlock:
            now = time.monotonic()
            dt = now - self._last_write_ts
            if dt < self.min_gap_s:
                time.sleep(self.min_gap_s - dt)
            n = self._ser.write(frame)
            self._ser.flush()  # push to driver
            self._last_write_ts = time.monotonic()
            self._last_activity = self._last_write_ts
            return n

    def request(
        self,
        code: int,
        payload: bytes = b"",
        timeout: float = 1.0,
        force_version: Optional[int] = None,
    ) -> Tuple[int, bytes]:
        """
        Send a frame and wait for the matching response (same code coming FROM FC).
        Returns (code, payload) and raises on transport failures.
        """
        if payload is None:
            payload = b""
        if not isinstance(payload, (bytes, bytearray)):
            raise TypeError("payload must be bytes or bytearray")
        payload_bytes = bytes(payload)

        with self._request_lock:
            last_error: Optional[Exception] = None
            for attempt in range(1, self._max_retries + 1):
                start_time = time.monotonic()
                try:
                    self._ensure_open()
                    self._ensure_reader_ok()
                    rsp_code, rsp_payload = self._request_core(
                        int(code), payload_bytes, timeout, force_version
                    )
                    self._last_activity = time.monotonic()
                    duration_ms = (time.monotonic() - start_time) * 1000.0
                    self.last_diag = {
                        "code": rsp_code,
                        "duration_ms": duration_ms,
                        "attempt": attempt,
                        "transport": self.transport,
                        "timestamp": time.time(),
                    }
                    return rsp_code, rsp_payload
                except TimeoutError as exc:
                    last_error = exc
                    self.last_diag = {
                        "code": int(code),
                        "error": str(exc),
                        "duration_ms": None,
                        "attempt": attempt,
                        "transport": self.transport,
                        "timestamp": time.time(),
                    }
                    if attempt >= self._max_retries or not self._retry_connection(attempt):
                        raise TimeoutError(f"{exc} after {attempt} attempt(s)") from exc

                except MSPUnsupportedError as exc:
                    last_error = exc
                    self.last_diag = {
                        "code": int(code),
                        "error": str(exc),
                        "duration_ms": None,
                        "attempt": attempt,
                        "transport": "serial",
                        "timestamp": time.time(),
                    }
                    break
                except (serial.SerialException, OSError, RuntimeError) as exc:
                    last_error = exc
                    self.last_diag = {
                        "code": int(code),
                        "error": str(exc),
                        "duration_ms": None,
                        "attempt": attempt,
                        "transport": "serial",
                        "timestamp": time.time(),
                    }
                    if attempt >= self._max_retries or not self._retry_connection(attempt):
                        break
                except Exception as exc:
                    last_error = exc
                    break

            if last_error:
                raise last_error
            raise RuntimeError("MSP transport failure")

    # ---------- internals ----------

    def _reader_loop(self):
        ser = self._ser
        if not ser:
            return
        try:
            while not self._stop_evt.is_set():
                try:
                    chunk = ser.read(4096)
                    if not chunk:
                        continue
                    self._last_activity = time.monotonic()
                    self._log_io("IN", chunk)
                    messages = self._parser.feed(chunk)
                    for msg in messages:
                        self._dispatch(msg)
                except (serial.SerialException, OSError):
                    self._record_reader_error()
                    break
                except Exception as exc:
                    self._record_reader_error(exc)
                    break
        finally:
            try:
                if ser.is_open:
                    ser.close()
            except Exception:
                self._log_io("ERR", b"failed closing serial on reader exit")
            self._ser = None

    def _dispatch(self, msg: MSPMessage) -> None:
        try:
            self._rx_all.put_nowait(msg)
        except Full:
            self._log_io("ERR", f"rx_all queue full dropping code {msg.code}".encode("ascii"))

        with self._q_lock:
            q = self._rx_by_code.get(msg.code)
        if q:
            try:
                q.put_nowait(msg)
            except Full:
                self._log_io("ERR", f"rx code queue full dropping code {msg.code}".encode("ascii"))

        if self.on_message:
            try:
                self.on_message(msg)
            except Exception:
                self._log_io("ERR", f"on_message exception for code {msg.code}".encode("ascii"))

    def _get_queue_for_code(self, code: int) -> Queue:
        with self._q_lock:
            q = self._rx_by_code.get(code)
            if q is None:
                q = Queue(maxsize=64)
                self._rx_by_code[code] = q
            return q

    @property
    def reconnects(self) -> int:
        return self._reconnects

    def _ensure_open(self) -> None:
        if not self._ser or not self._ser.is_open:
            self.open()
        elif not self._rx_thread or not self._rx_thread.is_alive():
            self._stop_evt.set()
            self._rx_thread = threading.Thread(target=self._reader_loop, name="MSPSerialReader", daemon=True)
            self._stop_evt.clear()
            self._rx_thread.start()
        self._ensure_reader_ok()

    def _retry_connection(self, attempt: int) -> bool:
        try:
            self.close()
        except Exception:
            pass
        time.sleep(self._reconnect_delay * attempt)
        try:
            self.open()
            self._reconnects += 1
            return True
        except Exception:
            return False

    def _request_core(
        self,
        code: int,
        payload: bytes,
        timeout: float,
        force_version: Optional[int],
        *,
        suppress_keepalive: bool = False,
    ) -> Tuple[int, bytes]:
        code_int = int(code)
        code_label = self._code_label(code_int)
        self._ensure_reader_ok()
        if not suppress_keepalive:
            self._run_keepalive_if_needed()

        q = self._get_queue_for_code(code)
        while True:
            try:
                q.get_nowait()
            except Empty:
                break
        self.send(code, payload, force_version=force_version)

        end = time.monotonic() + max(0.0, float(timeout))
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"MSP request timeout waiting for {code_label}({code_int})")
            try:
                msg: MSPMessage = q.get(timeout=remaining)
            except Empty:
                raise TimeoutError(f"MSP request timeout waiting for {code_label}({code_int})")
            if msg.unsupported:
                raise MSPUnsupportedError(f"MSP code {code_label}({code_int}) unsupported (! response)")
            return (msg.code, msg.payload)

    def _run_keepalive_if_needed(self) -> None:
        if (
            self._keepalive_code is None
            or self._keepalive_interval <= 0
            or self._running_keepalive
        ):
            return
        self._ensure_reader_ok()
        try:
            self._ensure_open()
        except Exception:
            return
        now = time.monotonic()
        if now - self._last_activity < self._keepalive_interval:
            return

        self._running_keepalive = True
        try:
            self._request_core(
                self._keepalive_code,
                self._keepalive_payload,
                self._keepalive_timeout,
                None,
                suppress_keepalive=True,
            )
        except Exception:
            # Let the caller trigger reconnect logic on failure.
            pass
        finally:
            self._last_activity = time.monotonic()
            self._running_keepalive = False

    @staticmethod
    def _choose_version(code: int, payload: bytes, force_version: Optional[int]) -> int:
        if force_version in (1, 2):
            return int(force_version)
        # MSP v2 required for codes >255 or payload >256 (incl. header constraint)
        if code > 0xFF or len(payload) > 0xFF:
            return 2
        return 1

    @staticmethod
    def _encode(version: int, code: int, payload: bytes) -> bytes:
        if version == 1:
            # V1: "$" "M" "<" len(1) code(1) payload len bytes xor_checksum(1)
            if len(payload) <= 0xFF:
                length_byte = len(payload) & 0xFF
                header = bytes([MSP_V1_SYNC_1, MSP_V1_SYNC_2, DIR_TO_FC, length_byte, code & 0xFF])
                checksum = length_byte ^ (code & 0xFF)
                for b in payload:
                    checksum ^= (b & 0xFF)
                return header + payload + bytes([checksum & 0xFF])
            else:
                # Jumbo (legacy): len == 255 then 16-bit true length
                true_len = len(payload)
                header = bytes([
                    MSP_V1_SYNC_1, MSP_V1_SYNC_2, DIR_TO_FC,
                    JUMBO_LIMIT_V1,                # sentinel 255
                    code & 0xFF
                ])
                checksum = (JUMBO_LIMIT_V1 ^ (code & 0xFF))
                lL = true_len & 0xFF
                lH = (true_len >> 8) & 0xFF
                checksum ^= lL
                checksum ^= lH
                body = bytes([lL, lH]) + payload
                for b in payload:
                    checksum ^= (b & 0xFF)
                return header + body + bytes([checksum & 0xFF])

        elif version == 2:
            # V2: "$" "X" "<" flags(1=0) codeL codeH lenL lenH payload crc8
            flags = 0
            codeL = code & 0xFF
            codeH = (code >> 8) & 0xFF
            lenL  = len(payload) & 0xFF
            lenH  = (len(payload) >> 8) & 0xFF
            header = bytes([MSP_V1_SYNC_1, MSP_V2_SYNC_2, DIR_TO_FC, flags, codeL, codeH, lenL, lenH])
            c = 0
            for b in header[3:]:   # start at flags
                c = crc8_dvb_s2(c, b)
            for b in payload:
                c = crc8_dvb_s2(c, b & 0xFF)
            return header + payload + bytes([c & 0xFF])

        else:
            raise ValueError("version must be 1 or 2")

    def _ensure_reader_ok(self) -> None:
        if self._reader_error is not None:
            raise RuntimeError(f"MSP reader stopped: {self._reader_error}")

    def _record_reader_error(self, exc: Optional[BaseException] = None) -> None:
        err = exc or sys.exc_info()[1] or RuntimeError("unknown reader error")
        self._reader_error = err
        self._stop_evt.set()
        self._log_io("ERR", f"reader error: {err}".encode("ascii", errors="ignore"))

    def _code_label(self, code: int) -> str:
        try:
            return f"{code} ({InavMSP(code).name})"
        except Exception:
            return str(code)

    def _log_io(self, direction: str, data: Any) -> None:
        if not self._log_enabled or not self._log_path:
            return
        ts = f"{time.time():.6f}"
        if isinstance(data, bytes):
            body = data.hex()
        elif isinstance(data, str):
            body = data
        else:
            body = repr(data)
        line = f"{ts} {direction} {body}\n"
        with self._log_lock:
            if self._log_fp is None:
                self._log_fp = open(self._log_path, "a", encoding="ascii", errors="ignore")
            self._log_fp.write(line)
            self._log_fp.flush()
