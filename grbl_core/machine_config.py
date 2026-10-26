# -*- coding: utf-8 -*-
"""Все числа станка в одном месте.

Правится только этот файл. Ядро, пульт и чужая программа читают его сами.
Число в файле — значение по умолчанию. Его можно перекрыть переменной
окружения с тем же смыслом, не трогая код. Пример в терминале Pi:

    export CNC_PORT=/dev/ttyUSB0
    export CNC_X_PITCH=10
    export CNC_A_GEAR=90

Имена переменных написаны справа в комментарии каждой строки.
Пустая переменная окружения не считается: останется число из файла.

Запуск «показать, что получилось»:

    python3 -m grbl_core.machine_config
"""

from __future__ import annotations

import os

# --- связь с Mega -------------------------------------------------------

PORT = "/dev/ttyACM0"          # CNC_PORT
BAUD = 115200                  # CNC_BAUD
POLL_INTERVAL = 0.08           # CNC_POLL_S, секунды между запросами координат

# --- драйвер DM542, общее для всех осей ---------------------------------
# Ток DIP-переключателями на корпусе. В Mega это число не уходит,
# оно здесь чтобы сборщик и программа видели одну и ту же уставку.
# Пиковые амперы DM542: 1.00  1.46  1.91  2.37  2.84  3.31  3.76  4.20
CURRENT_A = 2.37               # CNC_CURRENT_A
MICROSTEP = 8                  # CNC_MICROSTEP, если у оси свой — см. ось
MOTOR_FULL_STEPS = 200         # CNC_MOTOR_STEPS, 200 = двигатель 1.8°
PULSE_US = 10                  # CNC_PULSE_US, пишется в $0. Минимум по даташиту 2.5

# True, пока нет концевиков: не ждать homing и не держать мягкие лимиты.
BENCH_MODE = True              # CNC_BENCH, 1/0

# --- оси ----------------------------------------------------------------
# kind: "linear" (винт, мм) или "rotary" (стол, градусы).
# pitch_mm  — шаг винта, мм на оборот. У стола не используется.
# gear_ratio — оборотов мотора на один оборот винта или стола. Редуктора нет = 1.
# step — на сколько сдвинуть за один jog_step или одно нажатие в режиме шага.
# feed — рабочая подача jog, мм/мин или градус/мин.
# max_feed — потолок, выше него set_feed не пустит. Пишется в $110…
# accel — разгон, мм/с² или градус/с². Пишется в $120…
# continuous_window — длина одного куска, пока клавиша зажата.
# max_travel — мягкая граница в плюс. 0 = не ограничивать.
# min_pos — мягкая граница в минус.
# invert — ось едет не в ту сторону. Провода не перекидывать: станет бит в $3.

AXES = {
    "X": {
        "kind": "linear",
        "pitch_mm": 5.0,             # CNC_X_PITCH
        "gear_ratio": 1.0,           # CNC_X_GEAR
        "microstep": 8,              # CNC_X_MICROSTEP
        "motor_full_steps": 200,     # CNC_X_MOTOR_STEPS
        "step": 1.0,                 # CNC_X_STEP
        "feed": 1200.0,              # CNC_X_FEED
        "max_feed": 3000.0,          # CNC_X_MAX_FEED
        "accel": 200.0,              # CNC_X_ACCEL
        "continuous_window": 25.0,   # CNC_X_WINDOW
        "max_travel": 0.0,           # CNC_X_MAX_TRAVEL
        "min_pos": 0.0,              # CNC_X_MIN_POS
        "invert": False,             # CNC_X_INVERT
    },
    "Y": {
        "kind": "linear",
        "pitch_mm": 5.0,
        "gear_ratio": 1.0,
        "microstep": 8,
        "motor_full_steps": 200,
        "step": 1.0,
        "feed": 1200.0,
        "max_feed": 3000.0,
        "accel": 200.0,
        "continuous_window": 25.0,
        "max_travel": 0.0,
        "min_pos": 0.0,
        "invert": False,
    },
    "Z": {
        "kind": "linear",
        "pitch_mm": 5.0,
        "gear_ratio": 1.0,
        "microstep": 8,
        "motor_full_steps": 200,
        "step": 0.5,
        "feed": 600.0,
        "max_feed": 1500.0,
        "accel": 80.0,
        "continuous_window": 10.0,
        "max_travel": 0.0,
        "min_pos": 0.0,
        "invert": False,
    },
    "A": {
        "kind": "rotary",
        "pitch_mm": 0.0,
        "gear_ratio": 1.0,           # CNC_A_GEAR, оборотов мотора на оборот стола
        "microstep": 8,
        "motor_full_steps": 200,
        "step": 1.0,
        "feed": 900.0,
        "max_feed": 2000.0,
        "accel": 100.0,
        "continuous_window": 20.0,
        "max_travel": 0.0,
        "min_pos": 0.0,
        "invert": False,
    },
}

# Имена переменных окружения, которые перекрывают поле оси.
_AXIS_ENV = {
    "pitch_mm": "PITCH",
    "gear_ratio": "GEAR",
    "microstep": "MICROSTEP",
    "motor_full_steps": "MOTOR_STEPS",
    "step": "STEP",
    "feed": "FEED",
    "max_feed": "MAX_FEED",
    "accel": "ACCEL",
    "continuous_window": "WINDOW",
    "max_travel": "MAX_TRAVEL",
    "min_pos": "MIN_POS",
    "invert": "INVERT",
}

# --- помпа --------------------------------------------------------------
# digital — выход вкл/выкл, M64/M65. index 0 = пин D16, 1 = D17, 2 = D23, 3 = D25.
# coolant — команды M8/M9.
# axis    — помпа тоже шаговый драйвер, буква оси в PUMP_AXIS (обычно B).

PUMP_MODE = "digital"          # CNC_PUMP_MODE
PUMP_INDEX = 0                 # CNC_PUMP_INDEX
PUMP_AXIS = "B"                # CNC_PUMP_AXIS


def _raw(name: str):
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return None
    return value.strip()


def _as_bool(text: str) -> bool:
    return text.lower() in ("1", "true", "yes", "on", "да")


def _cast(value: str, sample):
    if isinstance(sample, bool):
        return _as_bool(value)
    if isinstance(sample, int) and not isinstance(sample, bool):
        return int(float(value.replace(",", ".")))
    if isinstance(sample, float):
        return float(value.replace(",", "."))
    return value


def _apply_env() -> None:
    """Подменяет константы модуля, если в окружении задано то же имя."""
    global PORT, BAUD, POLL_INTERVAL, CURRENT_A, MICROSTEP, MOTOR_FULL_STEPS
    global PULSE_US, BENCH_MODE, PUMP_MODE, PUMP_INDEX, PUMP_AXIS
    pairs = (
        ("CNC_PORT", "PORT"),
        ("CNC_BAUD", "BAUD"),
        ("CNC_POLL_S", "POLL_INTERVAL"),
        ("CNC_CURRENT_A", "CURRENT_A"),
        ("CNC_MICROSTEP", "MICROSTEP"),
        ("CNC_MOTOR_STEPS", "MOTOR_FULL_STEPS"),
        ("CNC_PULSE_US", "PULSE_US"),
        ("CNC_BENCH", "BENCH_MODE"),
        ("CNC_PUMP_MODE", "PUMP_MODE"),
        ("CNC_PUMP_INDEX", "PUMP_INDEX"),
        ("CNC_PUMP_AXIS", "PUMP_AXIS"),
    )
    for env_name, attr in pairs:
        raw = _raw(env_name)
        if raw is None:
            continue
        globals()[attr] = _cast(raw, globals()[attr])

    for axis, row in AXES.items():
        for field, suffix in _AXIS_ENV.items():
            raw = _raw(f"CNC_{axis}_{suffix}")
            if raw is None:
                continue
            row[field] = _cast(raw, row[field])


_apply_env()


def direction_mask() -> int:
    """Маска $3. Бит 0 = X, 1 = Y, 2 = Z, 3 = A, 4 = B, 5 = C."""
    bits = {"X": 1, "Y": 2, "Z": 4, "A": 8, "B": 16, "C": 32}
    mask = 0
    for name, row in AXES.items():
        if row.get("invert"):
            mask |= bits[name]
    return mask


def steps_per_unit(name: str) -> float:
    """Импульсов на мм (винт) или на градус (стол). Считается из чисел выше."""
    row = AXES[name]
    micro = int(row.get("microstep", MICROSTEP))
    full = int(row.get("motor_full_steps", MOTOR_FULL_STEPS))
    gear = float(row["gear_ratio"])
    if row["kind"] == "rotary":
        return full * micro * gear / 360.0
    pitch = float(row["pitch_mm"])
    if pitch <= 0:
        raise ValueError(f"у оси {name} шаг винта pitch_mm должен быть > 0")
    return full * micro * gear / pitch


def report() -> str:
    lines = [
        f"порт {PORT}  {BAUD} бод",
        f"импульс STEP {PULSE_US} мкс, концевики выключены: {BENCH_MODE}",
        f"ток на DIP (в плату не пишется): {CURRENT_A} А пик",
        f"помпа {PUMP_MODE}, индекс {PUMP_INDEX}, ось {PUMP_AXIS}",
        f"инверсия направлений $3 = {direction_mask()}",
        "",
    ]
    for name, row in AXES.items():
        unit = "град" if row["kind"] == "rotary" else "мм"
        lines.append(
            f"{name}  {steps_per_unit(name):.4f} имп/{unit}  "
            f"шаг jog {row['step']} {unit}  "
            f"подача {row['feed']}  потолок {row['max_feed']}  "
            f"окно {row['continuous_window']}  "
            f"микрошаг {row['microstep']}  редуктор {row['gear_ratio']}"
        )
    return "\n".join(lines)


if __name__ == "__main__":
    print(report())
    try:
        from grbl_core.profile import describe_current, describe_microstep
    except ImportError:
        from profile import describe_current, describe_microstep
    print()
    print(describe_current(float(CURRENT_A)))
    print("микрошаг по умолчанию:", describe_microstep(int(MICROSTEP)))
