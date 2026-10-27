"""Публичное ядро. Его импортирует и тестовая программа, и ваша.

Потоки:
  grbl-reader     — читает ответы Arduino
  grbl-status     — шлёт `?` (положение, не блокирует движение)
  grbl-callbacks  — ваши on_status / on_alarm, чтобы не встать в deadlock
  grbl-jog        — пока клавиша зажата, подкладывает короткие $J и сразу
                    обрывает их байтом 0x85, когда клавишу отпустили
"""

from __future__ import annotations

import math
import re
import threading
import time

from grbl_core.link import GrblLink, Reply, SimSerial, Status
from grbl_core.profile import (
    AXIS_ORDER,
    DM542_MAX_STEP_HZ,
    DM542_RECOMMENDED_PULSE_US,
    Axis,
    default_axes,
    exceeds_dm542,
    grbl_axis_index,
    step_frequency_hz,
)

_COMMENT = re.compile(r"\([^)]*\)|;.*$")


class GrblError(RuntimeError):
    pass


class GrblCore:
    def __init__(
        self,
        port: str | None = None,
        baud: int | None = None,
        simulate: bool = False,
        axes: list[Axis] | None = None,
        poll_interval: float | None = None,
    ):
        from grbl_core.machine_config import (
            BAUD,
            POLL_INTERVAL,
            PORT,
            PUMP_AXIS,
            PUMP_INDEX,
            PUMP_MODE,
        )

        self.port = PORT if port is None else port
        self.baud = BAUD if baud is None else baud
        self.simulate = simulate
        self.axes = {axis.name: axis for axis in (axes or default_axes())}
        self.poll_interval = POLL_INTERVAL if poll_interval is None else poll_interval
        self.pump_mode = PUMP_MODE
        self.pump_index = PUMP_INDEX
        self.pump_axis = PUMP_AXIS
        self.pump_is_on = False
        self.on_log = None
        self.on_status = None
        self.on_alarm = None
        self.sim: SimSerial | None = None

        self._axes_lock = threading.RLock()
        self._held_lock = threading.Lock()
        self._held: dict[str, int] = {}
        self._jog_generation = 0
        self._held_event = threading.Event()
        self._stop = threading.Event()
        self._abort = threading.Event()
        self._link: GrblLink | None = None
        self._jog_thread: threading.Thread | None = None

    # --- соединение -------------------------------------------------------

    def connect(self) -> None:
        if self._link is not None:
            raise GrblError("уже подключено")
        names = [name for name in AXIS_ORDER if name in self.axes]
        if self.simulate:
            self.sim = SimSerial(names)
            transport = self.sim
        else:
            transport = _open_transport(self.port, self.baud)
        link = GrblLink(transport, names)
        link.poll_interval = self.poll_interval
        link.on_status = self._relay_status
        link.on_alarm = self._relay_alarm
        link.on_info = self._relay_info
        self._link = link
        if not self.simulate:
            time.sleep(2.5)
        link.start()
        try:
            reply = link.command("$X", timeout=2.0)
            if not reply.ok:
                raise GrblError(reply.text or "нет ответа на $X")
        except Exception:
            self.close()
            raise
        self._stop.clear()
        self._jog_thread = threading.Thread(target=self._jog_loop, name="grbl-jog", daemon=True)
        self._jog_thread.start()

    def close(self) -> None:
        self._stop.set()
        self._abort.set()
        self._held_event.set()
        link = self._link
        if link is not None:
            try:
                link.jog_cancel()
            except Exception:
                pass
        thread = self._jog_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.5)
        if link is not None:
            link.close()
        self._link = None
        self._jog_thread = None

    def __enter__(self) -> "GrblCore":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # --- живые параметры --------------------------------------------------

    def axis(self, name: str) -> Axis:
        try:
            return self.axes[name.upper()]
        except KeyError as exc:
            raise GrblError(f"нет оси {name}") from exc

    def set_step(self, name: str, step: float) -> float:
        if step <= 0:
            raise ValueError("шаг должен быть > 0")
        with self._axes_lock:
            self.axis(name).step = float(step)
            return self.axis(name).step

    def set_feed(self, name: str, feed: float) -> float:
        if feed <= 0:
            raise ValueError("подача должна быть > 0")
        with self._axes_lock:
            axis = self.axis(name)
            if feed > axis.max_feed:
                raise ValueError(
                    f"подача {feed} выше max_feed {axis.max_feed}. "
                    "Сначала поднимите max_feed, если драйвер и механика тянут."
                )
            axis.feed = float(feed)
        self._retarget()
        return feed

    def set_max_feed(self, name: str, feed: float, to_grbl: bool = False) -> float:
        if feed <= 0:
            raise ValueError("подача должна быть > 0")
        with self._axes_lock:
            axis = self.axis(name)
            axis.max_feed = float(feed)
            if axis.feed > axis.max_feed:
                axis.feed = axis.max_feed
        if to_grbl:
            self.push_axis(name)
        self._retarget()
        return feed

    def scale_steps(self, factor: float) -> None:
        if factor <= 0:
            raise ValueError("множитель должен быть > 0")
        with self._axes_lock:
            for axis in self.axes.values():
                axis.step = max(0.001, axis.step * factor)

    def scale_feeds(self, factor: float) -> None:
        if factor <= 0:
            raise ValueError("множитель должен быть > 0")
        with self._axes_lock:
            for axis in self.axes.values():
                axis.feed = min(axis.max_feed, max(1.0, axis.feed * factor))
        self._retarget()

    def configure(self, name: str, to_grbl: bool = False, **fields) -> Axis:
        allowed = {
            "step",
            "feed",
            "max_feed",
            "accel",
            "continuous_window",
            "max_travel",
            "min_pos",
            "steps_per_unit",
            "unit",
        }
        unknown = set(fields) - allowed
        if unknown:
            raise ValueError(f"неизвестные поля: {', '.join(sorted(unknown))}")
        with self._axes_lock:
            axis = self.axis(name)
            for key, value in fields.items():
                setattr(axis, key, value)
            axis._check()
            if axis.feed > axis.max_feed:
                axis.feed = axis.max_feed
        if to_grbl:
            self.push_axis(name)
        if {"feed", "max_feed", "continuous_window", "max_travel"} & set(fields):
            self._retarget()
        return axis

    def set_steps_per_unit(self, name: str, steps: float, to_grbl: bool = True) -> None:
        if steps <= 0:
            raise ValueError("steps/unit должен быть > 0")
        with self._axes_lock:
            self.axis(name).steps_per_unit = float(steps)
        if to_grbl and self._link is not None:
            self.push_axis(name)

    # --- команды движения -------------------------------------------------

    def jog_step(self, name: str, direction: int = 1) -> Reply:
        """Один шаг текущим `axis.step` и `axis.feed`. Не зависит от G90/G91."""
        self._require()
        name = name.upper()
        with self._held_lock:
            if self._held:
                raise GrblError("сначала отпустите непрерывный ход")
        with self._axes_lock:
            axis = self.axis(name)
            distance = axis.step * (1 if direction > 0 else -1)
            feed = axis.clamped_feed()
        reply = self._link.command(f"$J=G91 {name}{distance:.4f} F{feed:.1f}", timeout=2.0)
        if not reply.ok:
            raise GrblError(reply.text)
        time.sleep(abs(distance) / feed * 60.0 + 0.4)
        return reply

    def hold(self, name: str, direction: int) -> None:
        """Зажать ось. direction: +1 или -1. Повтор того же знака игнорируется."""
        self._require()
        self._mutate_held(name, direction)

    def release(self, name: str) -> None:
        self._require()
        self._mutate_held(name, 0)

    def release_all(self) -> None:
        """Мгновенный стоп ручного хода (0x85), не soft-reset."""
        if self._link is None:
            return
        with self._held_lock:
            if not self._held:
                self._jog_generation += 1
            else:
                self._held.clear()
                self._jog_generation += 1
        self._link.jog_cancel()
        self._held_event.set()

    def stop(self) -> None:
        self.release_all()

    def reset(self) -> None:
        """Мягкий сброс контроллера, как ctrl-x. Движение обрывается.

        После него контроллер снова заперт: вызовите unlock().
        Потоки ядра при этом не останавливаются.
        """
        self._abort.set()
        with self._held_lock:
            self._held.clear()
            self._jog_generation += 1
        self._held_event.set()
        if self._link is not None:
            self._link.soft_reset()

    def command(self, line: str, timeout: float = 5.0) -> str:
        self._require()
        reply = self._link.command(line, timeout=timeout)
        if not reply.ok:
            raise GrblError(reply.text)
        return reply.text

    def run_program(self, gcode: str, timeout_per_line: float = 120.0) -> int:
        """Потоковая отправка программы. Ждёт `ok` на каждую строку. Возвращает число строк."""
        self._require()
        with self._held_lock:
            if self._held:
                raise GrblError("нельзя слать программу, пока зажат jog")
        self._abort.clear()
        sent = 0
        for raw in gcode.splitlines():
            if self._stop.is_set() or self._abort.is_set():
                break
            line = _COMMENT.sub("", raw).strip()
            if not line:
                continue
            reply = self._link.command(line, timeout=timeout_per_line)
            if not reply.ok:
                raise GrblError(f"{line}: {reply.text}")
            sent += 1
        return sent

    def abort_program(self) -> None:
        self._abort.set()
        if self._link is not None:
            self._link.feed_hold()

    def resume(self) -> None:
        self._require()
        self._link.cycle_start()

    def unlock(self) -> None:
        self.command("$X")

    def home(self) -> None:
        self.command("$H", timeout=120)

    # --- помпа ------------------------------------------------------------

    def set_pump(self, on: bool) -> None:
        self._require()
        self.pump_is_on = bool(on)
        if self.pump_mode == "axis":
            if on:
                self.hold(self.pump_axis, +1)
            else:
                self.release(self.pump_axis)
            return
        if self.pump_mode == "coolant":
            self.command("M8" if on else "M9")
            return
        # immediate digital output, не ждёт конца кадра
        self.command(f"M64 P{self.pump_index}" if on else f"M65 P{self.pump_index}")

    # --- настройки GRBL / DM542 ------------------------------------------

    def set_grbl(self, number: int, value) -> None:
        self._require()
        if isinstance(value, float):
            text = f"{value:.4f}".rstrip("0").rstrip(".")
        else:
            text = str(value)
        reply = self._link.command(f"${int(number)}={text}", timeout=3.0)
        if not reply.ok:
            raise GrblError(reply.text)

    def refresh_settings(self) -> dict:
        self._require()
        reply = self._link.command("$$", timeout=3.0)
        if not reply.ok:
            raise GrblError(reply.text)
        return dict(self._link.settings)

    def push_axis(self, name: str) -> None:
        """Записать steps/unit, max rate, accel и travel этой оси в EEPROM."""
        self._require()
        with self._axes_lock:
            axis = self.axis(name)
            index = grbl_axis_index(axis.name)
            steps = axis.steps_per_unit
            max_feed = axis.max_feed
            accel = axis.accel
            travel = axis.max_travel
        if steps > 0:
            self._warn_rate(axis.name, steps, max_feed)
            self.set_grbl(100 + index, steps)
        self.set_grbl(110 + index, max_feed)
        if accel > 0:
            self.set_grbl(120 + index, accel)
        if travel > 0:
            self.set_grbl(130 + index, travel)

    def apply_dm542_timing(self) -> None:
        """$0 = 10 мкс. Даташит принимает от 2.5, но 10 устойчивее на длинном кабеле."""
        self.set_grbl(0, DM542_RECOMMENDED_PULSE_US)

    def bench_mode(self) -> None:
        """Выключить концевики, homing и soft-limits. Только для стола без датчиков."""
        self.set_grbl(20, 0)
        self.set_grbl(21, 0)
        self.set_grbl(22, 0)
        self.command("$X")

    def apply_machine(self) -> None:
        """Записать в Mega всё, что задано в machine_config.py.

        Шаги на мм, потолок скорости, разгон, ход, длительность импульса,
        инверсию направлений и режим без концевиков. Ток DIP не записывается:
        его на драйвере крутят руками.
        """
        from grbl_core.machine_config import BENCH_MODE, PULSE_US, direction_mask

        if BENCH_MODE:
            self.bench_mode()
        self.set_grbl(0, PULSE_US)
        self.set_grbl(3, direction_mask())
        for name in list(self.axes):
            self.push_axis(name)

    def driver_warnings(self) -> list[str]:
        notes = []
        with self._axes_lock:
            snapshot = [(a.name, a.steps_per_unit, a.max_feed, a.unit) for a in self.axes.values()]
        for name, steps, feed, unit in snapshot:
            if steps <= 0:
                continue
            hz = step_frequency_hz(steps, feed)
            if hz > DM542_MAX_STEP_HZ:
                notes.append(
                    f"{name}: {hz:.0f} Гц при max_feed {feed} {unit}/мин "
                    f"> {DM542_MAX_STEP_HZ} Гц (потолок DM542)"
                )
        return notes

    def get_status(self) -> Status | None:
        if self._link is None:
            return None
        return self._link.get_status()

    def refresh_status(self, timeout: float = 0.6) -> Status:
        self._require()
        started = time.time()
        self._link.status_query()
        deadline = started + timeout
        while time.time() < deadline:
            status = self.get_status()
            if status is not None and status.t >= started:
                return status
            time.sleep(0.01)
        raise GrblError("нет статусного ответа")

    # --- внутренности jog -------------------------------------------------

    def _require(self) -> GrblLink:
        if self._link is None:
            raise GrblError("нет соединения, сначала connect()")
        return self._link

    def _epoch(self) -> int:
        with self._held_lock:
            return self._jog_generation

    def _snapshot(self) -> tuple[dict[str, int], int]:
        with self._held_lock:
            return dict(self._held), self._jog_generation

    def _mutate_held(self, name: str, direction: int) -> None:
        name = name.upper()
        new = 0 if direction == 0 else (1 if direction > 0 else -1)
        with self._held_lock:
            old_map = dict(self._held)
            old = old_map.get(name, 0)
            if old == new:
                return
            if name not in self.axes and new != 0:
                raise GrblError(f"нет оси {name}")
            if new == 0:
                self._held.pop(name, None)
            else:
                self._held[name] = new
            self._jog_generation += 1
            had_motion = bool(old_map)
        if had_motion:
            self._link.jog_cancel()
        self._held_event.set()

    def _retarget(self) -> None:
        """Скорость/окно изменились в полёте — оборвать текущий $J и собрать новый."""
        with self._held_lock:
            moving = bool(self._held)
            if moving:
                self._jog_generation += 1
        if moving and self._link is not None:
            self._link.jog_cancel()
            self._held_event.set()

    def _plan(self, held: dict[str, int]) -> tuple[str | None, float]:
        with self._axes_lock:
            specs = {
                axis.name: axis
                for axis in self.axes.values()
            }
        status = self.get_status()
        words: list[tuple[str, float]] = []
        feeds: list[float] = []
        for name in AXIS_ORDER:
            direction = held.get(name)
            axis = specs.get(name)
            if not direction or axis is None:
                continue
            room = axis.continuous_window
            if axis.max_travel > 0 and status is not None:
                pos = status.mpos.get(name, 0.0)
                if direction > 0:
                    room = min(room, max(0.0, axis.max_travel - pos - 0.05))
                else:
                    room = min(room, max(0.0, pos - axis.min_pos - 0.05))
            if room < 1e-4:
                continue
            words.append((name, room * direction))
            feeds.append(axis.clamped_feed())
        if not words or not feeds:
            return None, 0.0
        feed = min(feeds)
        path = math.sqrt(sum(distance * distance for _, distance in words))
        duration = path / feed * 60.0 if feed else 0.0
        body = " ".join(f"{name}{distance:.4f}" for name, distance in words)
        return f"$J=G91 {body} F{feed:.1f}", duration

    def _jog_loop(self) -> None:
        active_gen = self._epoch()
        motion_live = False
        pace = 0.2
        while not self._stop.is_set():
            self._held_event.wait(pace)
            if self._stop.is_set():
                break
            self._held_event.clear()
            try:
                held, gen = self._snapshot()
                link = self._link
                if link is None:
                    break
                if gen != active_gen:
                    if motion_live:
                        link.jog_cancel()
                        link.command("G4P0", timeout=0.8)
                        motion_live = False
                    active_gen = gen
                if not held:
                    pace = 0.2
                    continue
                if self._epoch() != gen:
                    pace = 0.02
                    continue
                command, duration = self._plan(held)
                if not command:
                    pace = 0.1
                    continue
                reply = link.command(command, timeout=2.0, epoch_fn=self._epoch, epoch=gen)
                if self._epoch() != gen or not reply.ok:
                    motion_live = False
                    if not reply.ok and reply.text != "cancelled":
                        self._log(f"jog отклонён: {reply.text}")
                    pace = 0.2 if not reply.ok else 0.04
                    continue
                motion_live = True
                pace = max(0.05, duration * 0.7)
            except Exception as exc:
                motion_live = False
                self._log(f"jog: {exc}")
                pace = 0.3

    def _warn_rate(self, name: str, steps: float, feed: float) -> None:
        if exceeds_dm542(steps, feed):
            hz = step_frequency_hz(steps, feed)
            self._log(
                f"{name}: {hz:.0f} Гц выше {DM542_MAX_STEP_HZ} Гц. "
                "Уменьшите микрошаг на DIP или max_feed."
            )

    def _log(self, message: str) -> None:
        if self.on_log is not None:
            self.on_log(message)

    def _relay_status(self, status: Status) -> None:
        if self.on_status is not None:
            self.on_status(status)

    def _relay_alarm(self, message: str) -> None:
        self._log(message)
        if self.on_alarm is not None:
            self.on_alarm(message)

    def _relay_info(self, message: str) -> None:
        self._log(message)


def _open_transport(port: str, baud: int):
    import serial

    try:
        return serial.Serial(port, baud, timeout=0.1, write_timeout=1)
    except serial.SerialException as exc:
        raise GrblError(
            f"не открылся {port}: {exc}. "
            "Проверьте кабель и что пользователь в группе dialout."
        ) from exc
