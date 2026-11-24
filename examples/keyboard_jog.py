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
  U         разблокировать после ALARM ($X)
  I         инфо: статус и настройки платы ($$, MPos)
  B         режим стенда без концевиков ($20=0, $21=0, $22=0)
  M         записать все настройки в Mega (apply_machine)
  , / .     шаг / 2    и   шаг * 2
  - / =     подача / 2 и   подача * 2
  1 / 2     режим шага / режим удержания
  Esc       выход

При запуске спрашивает множители подачи и шага (или берёт из --speed / --step).
Пример:
  python3 examples/keyboard_jog.py --simulate
  python3 examples/keyboard_jog.py --port COM3 --speed 2.0 --step 0.5
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from grbl_core import GrblCore, GrblError

KEYS = {
    "a": ("X", -1),  # X−
    "d": ("X", +1),  # X+
    "w": ("Y", +1),  # Y+
    "s": ("Y", -1),  # Y−
    "q": ("Z", +1),  # Z+
    "e": ("Z", -1),  # Z−
    "r": ("A", +1),  # стол A+
    "f": ("A", -1),  # стол A−
}


def _safe_jog_step(cnc: GrblCore, axis: str, direction: int) -> None:
    """Запуск jog_step в фоне — не блокирует поток клавиатуры."""
    try:
        cnc.jog_step(axis, direction)
    except GrblError:
        pass  # ось уже едет или нет связи — молча пропускаем


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
    release_timers: dict[str, threading.Timer] = {}

    def cancel_timer(axis: str) -> None:
        timer = release_timers.pop(axis, None)
        if timer is not None:
            timer.cancel()

    def cancel_all_timers() -> None:
        for timer in list(release_timers.values()):
            timer.cancel()
        release_timers.clear()

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
            cancel_all_timers()
            cnc.close()
            return False
        if key == keyboard.Key.space:
            cancel_all_timers()
            pressed.clear()
            cnc.stop()
            print("стоп")
            return None
        if char is None or char in pressed:
            return None
        pressed.add(char)
        if char in KEYS:
            axis, direction = KEYS[char]
            cancel_timer(axis)
            if mode["hold"]:
                cnc.hold(axis, direction)
            else:
                # jog_step в отдельном потоке — не блокирует клавиатуру
                threading.Thread(
                    target=_safe_jog_step,
                    args=(cnc, axis, direction),
                    daemon=True,
                ).start()
        elif char == "p":
            cnc.set_pump(not cnc.pump_is_on)
            print("помпа", "вкл" if cnc.pump_is_on else "выкл")
        elif char == "u":
            try:
                cnc.unlock()
                print("разблокировано ($X)")
            except Exception as exc:
                print(f"разблокировка не удалась: {exc}")
        elif char == "i":
            st = cnc.get_status()
            pos_str = f"MPos {st.mpos} WPos {st.wpos}" if st else "нет данных"
            print(f"[ИНФО] состояние: {st.state if st else '?'}, {pos_str}")
            try:
                settings = cnc.refresh_settings()
                print(f"[ИНФО] $0={settings.get(0)} (пульс) $3={settings.get(3)} (инверсия DIR)")
                print(f"[ИНФО] $20={settings.get(20)} (soft limits) $21={settings.get(21)} (hard limits) $22={settings.get(22)} (homing)")
            except Exception as exc:
                print(f"[ИНФО] ошибка чтения $$: {exc}")
        elif char == "b":
            try:
                cnc.bench_mode()
                print("Режим стенда применен: $20=0, $21=0, $22=0, $X")
            except Exception as exc:
                print(f"Ошибка bench_mode: {exc}")
        elif char == "m":
            try:
                cnc.apply_machine()
                print("Параметры станка записаны в Mega (apply_machine)")
            except Exception as exc:
                print(f"Ошибка apply_machine: {exc}")
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
            cancel_all_timers()
            pressed.clear()
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
        cancel_timer(axis)

        def later() -> None:
            release_timers.pop(axis, None)
            if char in pressed:
                return
            for other, (name, direction) in KEYS.items():
                if other != char and other in pressed and name == axis:
                    cnc.hold(name, direction)
                    return
            cnc.release(axis)

        # 30 мс фильтрация typematic repeat вместо 80 мс
        timer = threading.Timer(0.03, later)
        timer.daemon = True
        release_timers[axis] = timer
        timer.start()
        return None

    show()
    print("Esc — выход, пробел — стоп, U — разблокировать, I — инфо, B — bench_mode, M — apply_machine, P — помпа")
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()


def _char(key):
    try:
        return key.char.lower()
    except AttributeError:
        return None


def _ask_multiplier(prompt: str, default: float) -> float:
    """Спросить множитель у пользователя. Пустой ввод → default."""
    while True:
        try:
            raw = input(f"{prompt} [по умолчанию {default:g}]: ").strip()
            if raw == "":
                return default
            value = float(raw.replace(",", "."))
            if value <= 0:
                print("  Число должно быть положительным, попробуйте снова.")
                continue
            return value
        except ValueError:
            print("  Неверный формат — введите число, например 1.5")
        except (EOFError, KeyboardInterrupt):
            return default


def main() -> None:
    from grbl_core.machine_config import BAUD, PORT

    parser = argparse.ArgumentParser(description="ручной пульт GRBL")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--demo", action="store_true", help="прогнать движения без клавиатуры")
    parser.add_argument(
        "--speed", type=float, default=None, metavar="K",
        help="множитель подачи (1.0 = без изменений). Если не задан — спросит при запуске.",
    )
    parser.add_argument(
        "--step", type=float, default=None, metavar="K",
        help="множитель шага (1.0 = без изменений). Если не задан — спросит при запуске.",
    )
    args = parser.parse_args()

    # --- Интерактивный ввод множителей (только если не переданы флагами) ---
    if not args.demo:
        print("=== Настройка пульта ===")
        speed_k = args.speed if args.speed is not None else _ask_multiplier("Множитель подачи  (скорость ×K)", 1.0)
        step_k  = args.step  if args.step  is not None else _ask_multiplier("Множитель шага    (расстояние ×K)", 1.0)
        print(f"  → подача ×{speed_k:g}   шаг ×{step_k:g}")
        print("========================")
    else:
        speed_k = args.speed if args.speed is not None else 1.0
        step_k  = args.step  if args.step  is not None else 1.0

    cnc = GrblCore(port=args.port, baud=args.baud, simulate=args.simulate)
    cnc.on_log = lambda message: print(message)
    cnc.on_alarm = lambda msg: print(f"\n!! {msg} — нажмите U для разблокировки")
    cnc.connect()

    try:
        cnc.apply_machine()
        print("Настройки станка записаны в Mega ($0, $3, bench_mode $20=0/$21=0/$22=0, шаги, скорости)")
    except Exception as exc:
        print(f"Предупреждение: не удалось записать настройки в Mega: {exc}")

    # Базовые значения из конфига, умноженные на множители
    cnc.set_step("X", 5  * step_k)
    cnc.set_feed("X", 300 * speed_k)
    cnc.configure("A", step=20 * step_k, feed=1000 * speed_k, continuous_window=60)
    cnc.configure("Y", step=40 * step_k, feed=1200 * speed_k, continuous_window=80)
    cnc.configure("Z", step=50 * step_k, feed=900  * speed_k, continuous_window=50)

    try:
        if args.demo:
            demo(cnc)
        else:
            run_keyboard(cnc)
    finally:
        cnc.close()


if __name__ == "__main__":
    main()
