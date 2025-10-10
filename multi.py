# framework_core_v2.py
# Process-based, JSON messaging. Clean shutdown. Visible B activity.

import base64
import json
import logging
import multiprocessing as mp
import queue
import signal
import sys
import time
import uuid
from dataclasses import dataclass, asdict
from typing import Any, Dict, Optional

try:
    import serial
except Exception:
    serial = None


# ---------- util ----------

def _now_ms() -> int:
    return int(time.time() * 1000)

def _id() -> str:
    return uuid.uuid4().hex

def _reset_logging(name: str, level: int):
    root = logging.getLogger()
    for h in list(root.handlers):
        root.removeHandler(h)
    logging.basicConfig(level=level, format=f"[{name}] [%(levelname)s] %(message)s", stream=sys.stdout)


# ---------- message ----------

@dataclass
class Env:
    id: str
    src: str
    dst: str
    type: str           # REQUEST | RESPONSE | EVENT | ERROR | SHUTDOWN | REGISTER
    ts: int
    ttl_ms: int
    payload: Dict[str, Any]

    def dumps(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"), ensure_ascii=False)

    @staticmethod
    def loads(s: str) -> "Env":
        return Env(**json.loads(s))

def msg(src: str, dst: str, mtype: str, payload: Dict[str, Any], ttl_ms: int = 2000) -> Env:
    return Env(id=_id(), src=src, dst=dst, type=mtype, ts=_now_ms(), ttl_ms=ttl_ms, payload=payload)

def expired(e: Env) -> bool:
    return (_now_ms() - e.ts) > e.ttl_ms


# ---------- base actor ----------

class Actor(mp.Process):
    def __init__(self, name: str, inbox: mp.Queue, broker_q: mp.Queue, loglevel=logging.INFO):
        super().__init__(name=name)
        self.name = name
        self.inbox = inbox
        self.broker_q = broker_q
        self._run = mp.Event()
        self._run.clear()
        self.loglevel = loglevel

    def run(self):
        _reset_logging(self.name, self.loglevel)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, self._sig_term)
        self._run.set()
        try:
            self.on_start()
            self._register()
            while self._run.is_set():
                try:
                    raw = self.inbox.get(timeout=0.02)
                except queue.Empty:
                    self.on_idle()
                    continue
                env = Env.loads(raw)
                if expired(env):
                    continue
                if env.type == "SHUTDOWN":
                    break
                self.on_msg(env)
        except Exception:
            logging.exception("fatal")
            try:
                self._send("broker", "ERROR", {"actor": self.name})
            except Exception:
                pass
        finally:
            try:
                self.on_stop()
            except Exception:
                logging.exception("on_stop")
            logging.info("stopped")

    def _sig_term(self, *_):
        self._run.clear()

    def stop(self):
        self._run.clear()

    def _send(self, dst: str, mtype: str, payload: Dict[str, Any], ttl_ms: int = 2000):
        try:
            self.broker_q.put_nowait(msg(self.name, dst, mtype, payload, ttl_ms).dumps())
        except queue.Full:
            logging.warning("broker_q full")

    def _register(self):
        self._send("broker", "REGISTER", {"name": self.name})

    # hooks
    def on_start(self): ...
    def on_stop(self): ...
    def on_idle(self): ...
    def on_msg(self, env: Env): ...


# ---------- broker ----------

class Broker(Actor):
    def __init__(self, inbox: mp.Queue, routing: Dict[str, mp.Queue], loglevel=logging.INFO):
        super().__init__("broker", inbox, inbox, loglevel)
        self.routing = routing

    def on_start(self):
        logging.info("started")

    def on_msg(self, env: Env):
        if env.type == "REGISTER":
            who = env.payload.get("name", "?")
            logging.info(f"registered {who}")
            return
        if env.type == "ERROR":
            logging.error(f"actor error from {env.src}: {env.payload}")
            self._broadcast_shutdown()
            return
        if env.type == "SHUTDOWN":
            self._broadcast_shutdown()
            return

        q = self.routing.get(env.dst)
        if not q:
            return
        try:
            q.put_nowait(env.dumps())
        except queue.Full:
            try:
                _ = q.get_nowait()
                q.put_nowait(env.dumps())
            except Exception:
                pass

    def _broadcast_shutdown(self):
        for name, q in self.routing.items():
            try:
                q.put_nowait(msg("broker", name, "SHUTDOWN", {}, 500).dumps())
            except Exception:
                pass
        self.stop()


# ---------- serial handler (agnostic) ----------

class SerialHandler(Actor):
    """
    REQUEST payload: {"op":"serial.txrx","tx_b64":"..","timeout_ms":500}
    RESPONSE payload: {"ok":true,"rx_b64":"..","written":N,"rx_len":M} or {"ok":false,"error":".."}
    """
    def __init__(self, port: str, baud: int, inbox: mp.Queue, broker_q: mp.Queue, loglevel=logging.INFO):
        super().__init__("serial", inbox, broker_q, loglevel)
        self.port = port
        self.baud = baud
        self.ser = None

    def on_start(self):
        self._open()

    def on_stop(self):
        self._close()

    def _open(self):
        if serial is None:
            raise RuntimeError("pyserial not installed")
        if self.ser and self.ser.is_open:
            return
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=0.5)
        logging.info(f"open {self.port} @ {self.baud}")

    def _close(self):
        if self.ser:
            try: self.ser.close()
            except Exception: pass
            self.ser = None

    def on_msg(self, env: Env):
        if env.type != "REQUEST":
            return
        if env.payload.get("op") != "serial.txrx":
            self._reply(env, ok=False, error="unsupported op")
            return
        try:
            tx = base64.b64decode(env.payload.get("tx_b64", ""))
        except Exception:
            self._reply(env, ok=False, error="bad base64")
            return
        timeout_ms = int(env.payload.get("timeout_ms", 500))
        try:
            self._txrx(env, tx, timeout_ms)
        except Exception as e:
            logging.exception("txrx")
            try:
                self._close()
                time.sleep(0.05)
                self._open()
            except Exception:
                pass
            self._reply(env, ok=False, error=str(e))

    def _txrx(self, env: Env, tx: bytes, timeout_ms: int):
        if not (self.ser and self.ser.is_open):
            self._open()

        self.ser.reset_input_buffer()
        written = self.ser.write(tx)
        self.ser.flush()

        deadline = time.time() + (timeout_ms / 1000.0)
        chunks = []
        while time.time() < deadline and self._run.is_set():
            b = self.ser.read(4096)
            if b:
                chunks.append(b)
            else:
                time.sleep(0.001)
        rx = b"".join(chunks)
        self._reply(env, ok=True, rx_b64=base64.b64encode(rx).decode("ascii"), written=written, rx_len=len(rx))

    def _reply(self, req: Env, ok: bool, **kw):
        out = Env(id=req.id, src=self.name, dst=req.src, type="RESPONSE", ts=_now_ms(), ttl_ms=1000, payload={"ok": ok, **kw})
        try:
            self.broker_q.put_nowait(out.dumps())
        except queue.Full:
            pass


# ---------- client API ----------

class ClientAPI:
    def __init__(self, name: str, inbox: mp.Queue, broker_q: mp.Queue, default_timeout_ms: int = 1000):
        self.name = name
        self.inbox = inbox
        self.broker_q = broker_q
        self.default_timeout_ms = default_timeout_ms
        self.broker_q.put_nowait(msg(self.name, "broker", "REGISTER", {"name": self.name}).dumps())

    def request(self, dst: str, op: str, payload: Dict[str, Any], timeout_ms: Optional[int] = None) -> Dict[str, Any]:
        tms = timeout_ms or self.default_timeout_ms
        e = msg(self.name, dst, "REQUEST", {"op": op, **payload}, ttl_ms=tms)
        self.broker_q.put_nowait(e.dumps())
        deadline = time.time() + (tms / 1000.0)
        while time.time() < deadline:
            try:
                raw = self.inbox.get(timeout=0.02)
            except queue.Empty:
                continue
            env = Env.loads(raw)
            if env.type == "SHUTDOWN":
                raise RuntimeError("shutdown")
            if env.id != e.id:
                continue
            return env.payload
        raise TimeoutError(f"request timeout: {op}")

    def serial_txrx(self, tx: bytes, timeout_ms: int = 500) -> Dict[str, Any]:
        return self.request("serial", "serial.txrx",
                            {"tx_b64": base64.b64encode(tx).decode("ascii"), "timeout_ms": timeout_ms},
                            timeout_ms=timeout_ms + 500)


# ---------- clients ----------

class ClientBase(Actor):
    def __init__(self, name: str, cadence_hz: float, inbox: mp.Queue, broker_q: mp.Queue, loglevel=logging.INFO):
        super().__init__(name, inbox, broker_q, loglevel)
        self.api = ClientAPI(name, inbox, broker_q)
        self.period = 1.0 / cadence_hz
        self._tlast = 0.0
        self._ticks = 0

    def on_idle(self):
        now = time.time()
        if now - self._tlast >= self.period:
            self._tlast = now
            self._ticks += 1
            self.tick(self._ticks)

    def tick(self, n: int): ...
    def on_msg(self, env: Env): ...


class ClientA(ClientBase):
    def __init__(self, inbox, broker_q):
        super().__init__("A", 10.0, inbox, broker_q)

    def tick(self, n: int):
        try:
            payload = b"\x24\x4d\x3c\x00\x64\x64"  # placeholder
            r = self.api.serial_txrx(payload, timeout_ms=200)
            logging.info(f"A ok={r.get('ok')} rx_len={r.get('rx_len')}")
        except Exception as e:
            logging.error(f"{e}")


class ClientB(ClientBase):
    def __init__(self, inbox, broker_q):
        super().__init__("B", 100.0, inbox, broker_q)

    def tick(self, n: int):
        # Visible at INFO every 10 ticks (10 Hz equivalent), still running at 100 Hz internally
        if n % 10 == 0:
            logging.info(f"B tick {n}")


# ---------- supervisor ----------
# Replace your Supervisor with this version

class Supervisor:
    def __init__(self):
        _reset_logging("supervisor", logging.INFO)
        self.QN = 512
        self.q_broker = mp.Queue(maxsize=self.QN)
        self.q_serial = mp.Queue(maxsize=self.QN)
        self.q_A = mp.Queue(maxsize=self.QN)
        self.q_B = mp.Queue(maxsize=self.QN)

        self.routing = {
            "broker": self.q_broker,
            "serial": self.q_serial,
            "A": self.q_A,
            "B": self.q_B,
        }

        self.broker = Broker(inbox=self.q_broker, routing=self.routing)
        self.serial = SerialHandler(port="/dev/ttyACM0", baud=115200, inbox=self.q_serial, broker_q=self.q_broker)
        self.A = ClientA(inbox=self.q_A, broker_q=self.q_broker)
        self.B = ClientB(inbox=self.q_B, broker_q=self.q_broker)

        self.procs = [self.broker, self.serial, self.A, self.B]

    def start_all(self):
        for p in self.procs:
            p.daemon = False
            p.start()

    def _broadcast_direct(self):
        # Send SHUTDOWN directly to every inbox, not only via broker.
        shut = msg("supervisor", "any", "SHUTDOWN", {}, 500).dumps()
        for name, q in self.routing.items():
            try:
                q.put_nowait(shut)
            except Exception:
                pass

    def shutdown(self):
        # 1) Direct broadcast so every actor sees SHUTDOWN even if broker is dead.
        self._broadcast_direct()
        time.sleep(0.05)

        # 2) Nudge any blocking waits: SIGTERM to all children.
        for p in self.procs:
            if p.is_alive():
                try:
                    os.kill(p.pid, signal.SIGTERM)
                except Exception:
                    pass

        # 3) Short grace period.
        deadline = time.time() + 0.8
        while time.time() < deadline:
            if all(not p.is_alive() for p in self.procs):
                break
            time.sleep(0.05)

        # 4) Hard kill any stragglers.
        for p in self.procs:
            if p.is_alive():
                p.terminate()

        # 5) Join.
        for p in self.procs:
            try:
                p.join(timeout=0.8)
            except Exception:
                pass

    def run(self):
        self.start_all()

        def _sigint(_sig, _frm):
            logging.info("Ctrl+C")
            self.shutdown()
        signal.signal(signal.SIGINT, _sigint)

        try:
            while True:
                dead = [p.name for p in self.procs if not p.is_alive()]
                if dead:
                    logging.error(f"dead: {dead}")
                    self.shutdown()
                    break
                time.sleep(0.25)
        finally:
            logging.info("done")



if __name__ == "__main__":
    try:
        mp.set_start_method("fork")
    except RuntimeError:
        pass
    Supervisor().run()
