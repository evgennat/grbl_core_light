#!/usr/bin/env python3
"""Ручной пульт. Импортирует ядро и ничего больше про протокол не знает.

Удержание (по умолчанию) — ось едет, пока клавиша нажата, и встаёт сразу,
как только её отпустили. Шаг — один дискрет на нажатие.

  W / S     Y+ / Y-
  A / D     X- / X+
  Q / E     Z+ / Z-
  R / F     стол A+ / A-
  Пробел    стоп
  P         помпа
  , / .     шаг / 2    и   шаг * 2
  - / =     подача / 2 и   подача * 2
  1 / 2     режим шага / режим удержания
  Esc       выход

  python3 examples/keyboard_jog.py --simulate --demo
  python3 examples/keyboard_jog.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from grbl_core import GrblCore

KEYS = {
    "a": ("A", -1),  # ось X станка
    "d": ("A", +1),
    "w": ("Y", +1),  # вверх
    "s": ("Y", -1),  # вниз
    "q": ("Z", +1),  # портал от себя
    "e": ("Z", -1),  # портал на себя
    "r": ("X", +1),  # стол
    "f": ("X", -1),
}


def demo(cnc: GrblCore) -> None:
    cnc.set_step("X", 0.5)
    cnc.set_feed("X", 1800)
    cnc.jog_step("X", +1)
    cnc.jog_step("X", +1)
    cnc.hold("Y", +1)
    time.sleep(0.25)
    cnc.set_feed("Y", 2400)  # в полёте
    time.sleep(0.2)
    cnc.release("Y")
    cnc.set_pump(True)
    cnc.set_pump(False)
    status = cnc.refresh_status()
    print(f"состояние {status.state}  MPos {status.mpos}")
    print(f"шаг X={cnc.axis('X').step}  подача Y={cnc.axis('Y').feed}")


def run_keyboard(cnc: GrblCore) -> None:
    from pynput import keyboard

    mode = {"hold": True}
    pressed: set[str] = set()

    def show() -> None:
        kind = "удержание" if mode["hold"] else "шаг"
        bits = "  ".join(
            f"{name}:{axis.step:g}{axis.unit}@{axis.feed:g}"
            for name, axis in cnc.axes.items()
        )
        print(f"[{kind}] {bits}")

    def on_press(key):
        char = _char(key)
        if key == keyboard.Key.esc:
            cnc.close()
            return False
        if key == keyboard.Key.space:
            cnc.stop()
            print("стоп")
            return None
        if char is None or char in pressed:
            return None
        pressed.add(char)
        if char in KEYS:
            axis, direction = KEYS[char]
            if mode["hold"]:
                cnc.hold(axis, direction)
            else:
                cnc.jog_step(axis, direction)
        elif char == "p":
            cnc.set_pump(not cnc.pump_is_on)
            print("помпа", "вкл" if cnc.pump_is_on else "выкл")
        elif char == ",":
            cnc.scale_steps(0.5)
            show()
        elif char == ".":
            cnc.scale_steps(2.0)
            show()
        elif char == "-":
            cnc.scale_feeds(0.5)
            show()
        elif char in ("=", "+"):
            cnc.scale_feeds(2.0)
            show()
        elif char == "1":
            cnc.stop()
            mode["hold"] = False
            show()
        elif char == "2":
            mode["hold"] = True
            show()
        return None

    def on_release(key):
        char = _char(key)
        if char is None:
            return None
        pressed.discard(char)
        if not (mode["hold"] and char in KEYS):
            return None
        axis = KEYS[char][0]

        def later() -> None:
            if char in pressed:
                return
            for other, (name, direction) in KEYS.items():
                if other != char and other in pressed and name == axis:
                    cnc.hold(name, direction)
                    return
            cnc.release(axis)

        threading.Timer(0.08, later).start()
        return None

    show()
    print("Esc — выход, пробел — стоп, P — помпа")
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()


def _char(key):
    try:
        return key.char.lower()
    except AttributeError:
        return None


def main() -> None:
    from grbl_core.machine_config import BAUD, PORT

    parser = argparse.ArgumentParser(description="ручной пульт GRBL")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--demo", action="store_true", help="прогнать движения без клавиатуры")
    args = parser.parse_args()

    cnc = GrblCore(port=args.port, baud=args.baud, simulate=args.simulate)
    cnc.on_log = lambda message: print(message)
    cnc.connect()
    cnc.set_step("X", 5)
    cnc.set_feed("X", 300)
    cnc.configure("A", step=20, feed=1000, continuous_window=60)
    cnc.configure("Y", step=40, feed=1200, continuous_window=80)
    cnc.configure("Z", step=50, feed=900, continuous_window=50)
    try:
        if args.demo:
            demo(cnc)
        else:
            run_keyboard(cnc)
    finally:
        cnc.close()


if __name__ == "__main__":
    main()
