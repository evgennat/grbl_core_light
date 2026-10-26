"""Профиль осей и цифры из даташита DM542 (Elsor).

Ток и микрошаг задаются DIP-переключателями на самом драйвере.
Прошивка видит только импульсы STEP/DIR, поэтому ядро может менять
шаг, подачу, ускорение и длительность импульса, но не амперы.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Пин SW: True = ON, False = OFF.
_ON, _OFF = True, False

# SW1 SW2 SW3 → пиковый ток, А
DM542_CURRENT = (
    (1.00, 0.71, _ON, _ON, _ON),
    (1.46, 1.04, _OFF, _ON, _ON),
    (1.91, 1.36, _ON, _OFF, _ON),
    (2.37, 1.69, _OFF, _OFF, _ON),
    (2.84, 2.03, _ON, _ON, _OFF),
    (3.31, 2.36, _OFF, _ON, _OFF),
    (3.76, 2.69, _ON, _OFF, _OFF),
    (4.20, 3.00, _OFF, _OFF, _OFF),
)

# SW5 SW6 SW7 SW8 → микрошаг и импульсов на оборот (двигатель 1.8°)
DM542_MICROSTEP = (
    (2, 400, _OFF, _ON, _ON, _ON),
    (4, 800, _ON, _OFF, _ON, _ON),
    (8, 1600, _OFF, _OFF, _ON, _ON),
    (16, 3200, _ON, _ON, _OFF, _ON),
    (32, 6400, _OFF, _ON, _OFF, _ON),
    (64, 12800, _ON, _OFF, _OFF, _ON),
    (128, 25600, _OFF, _OFF, _OFF, _ON),
    (5, 1000, _ON, _ON, _ON, _OFF),
    (10, 2000, _OFF, _ON, _ON, _OFF),
    (20, 4000, _ON, _OFF, _ON, _OFF),
    (25, 5000, _OFF, _OFF, _ON, _OFF),
    (40, 8000, _ON, _ON, _OFF, _OFF),
    (50, 10000, _OFF, _ON, _OFF, _OFF),
    (100, 20000, _ON, _OFF, _OFF, _OFF),
    (125, 25000, _OFF, _OFF, _OFF, _OFF),
)

# Даташит: импульс ≥ 2.5 мкс, пауза ≥ 2.5 мкс → 200 кГц.
# DIR должен установиться минимум за 5 мкс до PUL — это делает GRBL сам.
DM542_MIN_PULSE_US = 2.5
DM542_RECOMMENDED_PULSE_US = 10
DM542_MAX_STEP_HZ = 200_000
DM542_SUPPLY_V = (20, 45)  # рабочий совет; абсолютный потолок драйвера 50 В

AXIS_ORDER = ("X", "Y", "Z", "A", "B", "C")

# grbl-Mega-5X, cpu_map RAMPS, без щита — те же пины Mega.
MEGA_PINS = {
    "X": {"step": "A0", "dir": "A1", "enable": "D38"},
    "Y": {"step": "A6", "dir": "A7", "enable": "A2"},
    "Z": {"step": "D46", "dir": "D48", "enable": "A8"},
    "A": {"step": "D26", "dir": "D28", "enable": "D24"},
    "B": {"step": "D36", "dir": "D34", "enable": "D30"},
    "C": {"step": "D49", "dir": "D51", "enable": "D53"},
}

# M64/M65 Pn → пины Mega (digital output 0..3).
DIGITAL_OUTPUT_PINS = {0: "D16", 1: "D17", 2: "D23", 3: "D25"}


def _sw(flag: bool) -> str:
    return "ON" if flag else "OFF"


def microstep_row(microstep: int) -> tuple:
    for row in DM542_MICROSTEP:
        if row[0] == microstep:
            return row
    known = ", ".join(str(row[0]) for row in DM542_MICROSTEP)
    raise ValueError(f"у DM542 нет микрошага {microstep}. Доступны: {known}")


def steps_per_rev(microstep: int) -> int:
    return int(microstep_row(microstep)[1])


def describe_microstep(microstep: int) -> str:
    _div, per_rev, *sw = microstep_row(microstep)
    names = " ".join(f"SW{5 + i}={_sw(bit)}" for i, bit in enumerate(sw))
    return f"1/{microstep} ({per_rev} шаг/об) {names}"


def describe_current(peak_a: float) -> str:
    for peak, rms, *sw in DM542_CURRENT:
        if abs(peak - peak_a) < 0.05:
            names = " ".join(f"SW{1 + i}={_sw(bit)}" for i, bit in enumerate(sw))
            return f"{peak:.2f} А пик / {rms:.2f} А RMS, {names}, SW4=OFF (удержание 50%)"
    known = ", ".join(f"{row[0]:.2f}" for row in DM542_CURRENT)
    raise ValueError(f"нет такой уставки тока. Пиковые значения: {known}")


def ballscrew_steps_per_mm(
    pitch_mm: float,
    microstep: int = 8,
    motor_full_steps: int = 200,
    gear_ratio: float = 1.0,
) -> float:
    """Импульсов на мм. gear_ratio = оборотов мотора / оборотов винта."""
    if pitch_mm <= 0:
        raise ValueError("шаг винта должен быть > 0")
    if gear_ratio <= 0:
        raise ValueError("передаточное отношение должно быть > 0")
    return motor_full_steps * microstep * gear_ratio / pitch_mm


def rotary_steps_per_deg(
    microstep: int = 8,
    motor_full_steps: int = 200,
    gear_ratio: float = 1.0,
) -> float:
    """Импульсов на градус поворотного стола. gear_ratio = оборотов мотора / оборот стола."""
    if gear_ratio <= 0:
        raise ValueError("передаточное отношение должно быть > 0")
    return motor_full_steps * microstep * gear_ratio / 360.0


def step_frequency_hz(steps_per_unit: float, units_per_min: float) -> float:
    return steps_per_unit * units_per_min / 60.0


def exceeds_dm542(steps_per_unit: float, units_per_min: float) -> bool:
    if steps_per_unit <= 0:
        return False
    return step_frequency_hz(steps_per_unit, units_per_min) > DM542_MAX_STEP_HZ


@dataclass
class Axis:
    """Живые параметры оси. Меняются на лету, jog читает их в момент отправки."""

    name: str
    unit: str = "mm"
    step: float = 1.0
    feed: float = 1200.0
    max_feed: float = 20000.0
    accel: float = 200.0
    continuous_window: float = 25.0
    max_travel: float = 0.0
    min_pos: float = 0.0
    steps_per_unit: float = 0.0
    extras: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.name = self.name.upper()
        if self.name not in AXIS_ORDER:
            raise ValueError(f"ось {self.name} не из {AXIS_ORDER}")
        self._check()

    def _check(self) -> None:
        if self.step <= 0:
            raise ValueError("шаг должен быть > 0")
        if self.feed <= 0 or self.max_feed <= 0:
            raise ValueError("подача должна быть > 0")
        if self.continuous_window <= 0:
            raise ValueError("окно непрерывного хода должно быть > 0")

    def clamped_feed(self) -> float:
        return min(self.feed, self.max_feed)


def default_axes() -> list[Axis]:
    """Оси из machine_config.py. Шаг винта и микрошаг сами становятся steps_per_unit."""
    from grbl_core.machine_config import AXES, steps_per_unit

    built: list[Axis] = []
    for name, row in AXES.items():
        unit = "deg" if row["kind"] == "rotary" else "mm"
        built.append(
            Axis(
                name,
                unit=unit,
                step=float(row["step"]),
                feed=float(row["feed"]),
                max_feed=float(row["max_feed"]),
                accel=float(row["accel"]),
                continuous_window=float(row["continuous_window"]),
                max_travel=float(row["max_travel"]),
                min_pos=float(row["min_pos"]),
                steps_per_unit=steps_per_unit(name),
            )
        )
    return built


def grbl_axis_index(name: str) -> int:
    return AXIS_ORDER.index(name.upper())
