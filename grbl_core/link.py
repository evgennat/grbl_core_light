"""Поток чтения serial, поток статуса и отправка строк.

Realtime-байты (?, !, 0x85, ctrl-x) GRBL вырезает из потока сразу,
поэтому их можно писать, не дожидаясь `ok`.
"""

from __future__ import annotations

import queue
import re
import threading
import time
from dataclasses import dataclass, field

_STATUS_AXES = ("X", "Y", "Z", "A", "B", "C")
_WORD = re.compile(r"([A-Za-z])\s*([-+]?\d*\.?\d+)")


@dataclass
class Reply:
    ok: bool
    text: str


@dataclass
class Status:
    state: str
    mpos: dict[str, float]
    wpos: dict[str, float]
    feed: float
    spindle: float
    raw: str
    t: float


def _numbers(text: str) -> list[float]:
    out = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            out.append(float(part))
        except ValueError:
            pass
    return out


def _map_axes(nums: list[float], names: list[str]) -> dict[str, float]:
    return {names[i]: nums[i] for i in range(min(len(nums), len(names)))}


def parse_status(line: str, axis_names: list[str]) -> Status | None:
    if not (line.startswith("<") and line.endswith(">")):
        return None
    parts = line[1:-1].split("|")
    if not parts or not parts[0]:
        return None
    mpos: dict[str, float] = {}
    wpos: dict[str, float] = {}
    wco: dict[str, float] = {}
    feed = 0.0
    spindle = 0.0
    for part in parts[1:]:
        if ":" not in part:
            continue
        key, value = part.split(":", 1)
        nums = _numbers(value)
        if key == "MPos":
            mpos = _map_axes(nums, axis_names)
        elif key == "WPos":
            wpos = _map_axes(nums, axis_names)
        elif key == "WCO":
            wco = _map_axes(nums, axis_names)
        elif key == "FS" and nums:
            feed = nums[0]
            if len(nums) > 1:
                spindle = nums[1]
        elif key == "F" and nums:
            feed = nums[0]
    if mpos and not wpos:
        wpos = {name: mpos[name] - wco.get(name, 0.0) for name in mpos}
    if wpos and not mpos:
        mpos = {name: wpos[name] + wco.get(name, 0.0) for name in wpos}
    return Status(
        state=parts[0],
        mpos=mpos,
        wpos=wpos,
        feed=feed,
        spindle=spindle,
        raw=line,
        t=time.time(),
    )


def _parse_setting_value(text: str):
    try:
        return float(text) if "." in text else int(text)
    except ValueError:
        return text


class SimSerial:
    """Минимальный GRBL для проверки ядра без Arduino."""

    def __init__(self, axes: list[str] | None = None):
        self.axes = list(axes or ("X", "Y", "Z", "A"))
        self.pos = {name: 0.0 for name in self.axes}
        self.state = "Idle"
        self.settings = {
            0: 10,
            1: 25,
            3: 0,
            20: 0,
            21: 0,
            22: 0,
            100: 250.0,
            101: 250.0,
            102: 250.0,
            103: 250.0,
            110: 5000.0,
            111: 5000.0,
            112: 2000.0,
            113: 5000.0,
        }
        self.outputs: dict[int, bool] = {}
        self.lines: list[str] = []
        self.raw = bytearray()
        self.cancels = 0
        self._buf = bytearray()
        self._rx: queue.Queue[bytes] = queue.Queue()
        self._lock = threading.RLock()
        self.closed = False

    def write(self, data: bytes) -> None:
        with self._lock:
            if self.closed:
                raise OSError("serial closed")
            self.raw.extend(data)
            for byte in data:
                if byte == 0x85:
                    self.cancels += 1
                    self.state = "Idle"
                elif byte == 0x3F:  # ?
                    self._emit(self._status_line())
                elif byte == 0x18:
                    self.state = "Idle"
                    for name in self.pos:
                        self.pos[name] = 0.0
                    self._emit("Grbl 1.1h ['$' for help]")
                elif byte in (10, 13):
                    if self._buf:
                        self._handle(self._buf.decode("ascii", "ignore").strip())
                    self._buf.clear()
                elif byte >= 32:
                    self._buf.append(byte)

    def readline(self) -> bytes:
        try:
            return self._rx.get(timeout=0.1)
        except queue.Empty:
            return b""

    def close(self) -> None:
        with self._lock:
            self.closed = True
        self._rx.put(b"")

    def _emit(self, line: str) -> None:
        self._rx.put((line + "\n").encode("ascii", "ignore"))

    def _status_line(self) -> str:
        pos = ",".join(f"{self.pos.get(name, 0.0):.3f}" for name in self.axes)
        return f"<{self.state}|MPos:{pos}|FS:0,0>"

    def _handle(self, line: str) -> None:
        self.lines.append(line)
        if not line:
            return
        upper = line.upper()
        if upper.startswith("$J"):
            self._jog(line.split("=", 1)[1])
            self.state = "Jog"
            self._emit("ok")
            return
        if upper.startswith("$$"):
            for key in sorted(self.settings):
                self._emit(f"${key}={self.settings[key]}")
            self._emit("ok")
            return
        if upper.startswith("$") and "=" in line:
            key_text, value_text = line[1:].split("=", 1)
            self.settings[int(key_text.strip())] = _parse_setting_value(value_text.strip())
            self._emit("ok")
            return
        if upper.startswith("$X"):
            self.state = "Idle"
            self._emit("ok")
            return
        if upper.startswith("M64"):
            self._set_output(line, True)
            return
        if upper.startswith("M65"):
            self._set_output(line, False)
            return
        if upper.startswith(("M8", "M9", "M3", "M4", "M5", "G4", "G0", "G1", "G90", "G91", "G21")):
            self._emit("ok")
            return
        if upper.startswith("$"):
            self._emit("ok")
            return
        self._emit("ok")

    def _set_output(self, line: str, on: bool) -> None:
        match = re.search(r"P\s*(\d+)", line, re.I)
        index = int(match.group(1)) if match else 0
        self.outputs[index] = on
        self._emit("ok")

    def _jog(self, body: str) -> None:
        for axis, value in _WORD.findall(body):
            axis = axis.upper()
            if axis in self.pos:
                self.pos[axis] += float(value)


class GrblLink:
    def __init__(self, ser, axis_names: list[str]):
        self._ser = ser
        self.axis_names = list(axis_names)
        self.settings: dict[int, object] = {}
        self.status: Status | None = None
        self.alarm: str | None = None
        self.on_status = None
        self.on_alarm = None
        self.on_info = None
        self.poll_interval = 0.1
        self._status_lock = threading.Lock()
        self._line_lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._acks: queue.Queue[Reply] = queue.Queue()
        self._cb: queue.Queue = queue.Queue(maxsize=64)
        self._stop = threading.Event()
        self._reader: threading.Thread | None = None
        self._poller: threading.Thread | None = None
        self._callbacks: threading.Thread | None = None

    def start(self) -> None:
        self._reader = threading.Thread(target=self._read_loop, name="grbl-reader", daemon=True)
        self._poller = threading.Thread(target=self._poll_loop, name="grbl-status", daemon=True)
        self._callbacks = threading.Thread(target=self._callback_loop, name="grbl-callbacks", daemon=True)
        self._reader.start()
        self._poller.start()
        self._callbacks.start()

    def close(self) -> None:
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass
        for thread in (self._reader, self._poller, self._callbacks):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=1.0)

    def write_raw(self, data: bytes) -> None:
        with self._io_lock:
            self._ser.write(data)

    def jog_cancel(self) -> None:
        self.write_raw(b"\x85")

    def feed_hold(self) -> None:
        self.write_raw(b"!")

    def cycle_start(self) -> None:
        self.write_raw(b"~")

    def soft_reset(self) -> None:
        self.write_raw(b"\x18")

    def status_query(self) -> None:
        self.write_raw(b"?")

    def command(self, line: str, timeout: float = 2.0, epoch_fn=None, epoch=None) -> Reply:
        line = line.strip()
        if not line:
            return Reply(True, "")
        with self._line_lock:
            self._discard_acks()
            with self._io_lock:
                self._ser.write((line + "\n").encode("ascii", "ignore"))
            return self._wait_ack(timeout, epoch_fn, epoch)

    def get_status(self) -> Status | None:
        with self._status_lock:
            return self.status

    def _discard_acks(self) -> None:
        while True:
            try:
                self._acks.get_nowait()
            except queue.Empty:
                return

    def _wait_ack(self, timeout: float, epoch_fn=None, epoch=None) -> Reply:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if epoch_fn is not None and epoch is not None and epoch_fn() != epoch:
                self.jog_cancel()
                return Reply(False, "cancelled")
            alarm = self.alarm
            if alarm:
                self.alarm = None
                return Reply(False, alarm)
            try:
                return self._acks.get(timeout=0.05)
            except queue.Empty:
                continue
        self.jog_cancel()
        return Reply(False, "timeout")

    def _read_loop(self) -> None:
        pending = ""
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
            except Exception:
                if self._stop.is_set():
                    return
                time.sleep(0.05)
                continue
            if not raw:
                continue
            if isinstance(raw, bytes):
                piece = raw.decode("ascii", "ignore")
            else:
                piece = str(raw)
            pending += piece
            while "\n" in pending:
                chunk, pending = pending.split("\n", 1)
                line = chunk.strip()
                if line:
                    self._dispatch(line)

    def _dispatch(self, line: str) -> None:
        if line == "ok" or line.startswith("ok"):
            self._acks.put(Reply(True, line))
            return
        if line.startswith("error"):
            self._acks.put(Reply(False, line))
            return
        if line.startswith("ALARM"):
            self.alarm = line
            self._post("alarm", line)
            return
        if line.startswith("<"):
            status = parse_status(line, self.axis_names)
            if status is not None:
                with self._status_lock:
                    self.status = status
                self._post("status", status)
            return
        if line.startswith("$") and "=" in line:
            key_text, value_text = line[1:].split("=", 1)
            key_text = key_text.strip()
            if key_text.isdigit():
                self.settings[int(key_text)] = _parse_setting_value(value_text.strip())
            return
        if line.startswith("[") or line.startswith("Grbl"):
            self._post("info", line)

    def _post(self, kind: str, payload) -> None:
        try:
            self._cb.put_nowait((kind, payload))
        except queue.Full:
            if kind != "status":
                try:
                    self._cb.get_nowait()
                    self._cb.put_nowait((kind, payload))
                except queue.Empty:
                    pass

    def _callback_loop(self) -> None:
        while not self._stop.is_set():
            try:
                kind, payload = self._cb.get(timeout=0.1)
            except queue.Empty:
                continue
            callback = {"status": self.on_status, "alarm": self.on_alarm, "info": self.on_info}.get(kind)
            if callback is None:
                continue
            try:
                callback(payload)
            except Exception:
                continue

    def _poll_loop(self) -> None:
        while not self._stop.wait(self.poll_interval):
            try:
                self.status_query()
            except Exception:
                if self._stop.is_set():
                    return
                time.sleep(0.05)
