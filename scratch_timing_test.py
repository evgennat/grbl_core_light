"""Диагностика задержек jog-loop при удержании клавиш."""
import sys, time
sys.path.insert(0, '.')

from grbl_core.machine_config import POLL_INTERVAL
from grbl_core import GrblCore

print(f"POLL_INTERVAL = {POLL_INTERVAL}")

cnc = GrblCore(simulate=True, poll_interval=0.02)
logs = []
cnc.on_log = logs.append
cnc.connect()

# Сценарий: hold → release → быстрое повторное hold
t0 = time.time()
cnc.hold('X', +1)
time.sleep(0.15)
cnc.release('X')
time.sleep(0.06)   # 60 мс — быстрое нажатие снова
cnc.hold('X', +1)
time.sleep(0.3)
cnc.release('X')
time.sleep(0.6)

jogs = [l for l in cnc.sim.lines if l.startswith('$J')]
print(f"jog-команды отправлено: {len(jogs)}")
for l in jogs:
    print(' ', l)
print(f"cancels (0x85): {cnc.sim.cancels}")
cnc.close()
print(f"всего: {time.time()-t0:.2f}s")
print()

# Тест jog_step: он блокирует поток клавиатуры!
print("=== jog_step блокировка ===")
cnc2 = GrblCore(simulate=True)
cnc2.connect()
t1 = time.time()
# X step=5, feed=300 → ждать 5/300*60 + 0.4 = 1.4с!
axis = cnc2.axis('X')
print(f"X step={axis.step}, feed={axis.feed}")
expected_block = axis.step / axis.feed * 60.0 + 0.4
print(f"jog_step заблокирует поток клавиатуры на ~{expected_block:.2f}s")
cnc2.close()
