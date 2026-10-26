"""Проверка ядра без станка: симулятор GRBL и потоки."""

from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from grbl_core import (
    GrblCore,
    ballscrew_steps_per_mm,
    rotary_steps_per_deg,
    step_frequency_hz,
)
from grbl_core.profile import DM542_MAX_STEP_HZ, describe_current, describe_microstep


class ProfileTest(unittest.TestCase):
    def test_ballscrew_and_rotary(self):
        # 200 * 8 / 5 мм = 320 шаг/мм
        self.assertAlmostEqual(ballscrew_steps_per_mm(5, microstep=8), 320.0)
        self.assertAlmostEqual(rotary_steps_per_deg(8, gear_ratio=90), 200 * 8 * 90 / 360.0)

    def test_machine_config_feeds_the_axes(self):
        from grbl_core.profile import default_axes

        axes = {axis.name: axis for axis in default_axes()}
        self.assertAlmostEqual(axes["X"].steps_per_unit, 320.0)
        self.assertAlmostEqual(axes["X"].step, 1.0)
        self.assertAlmostEqual(axes["Y"].feed, 1200.0)

    def test_dm542_ceiling(self):
        hz = step_frequency_hz(320, 60000)
        self.assertGreater(hz, DM542_MAX_STEP_HZ)
        self.assertIn("SW5=OFF", describe_microstep(8))
        self.assertIn("2.37", describe_current(2.37))


class CoreTest(unittest.TestCase):
    def setUp(self):
        self.cnc = GrblCore(simulate=True, poll_interval=0.02)
        self.logs = []
        self.cnc.on_log = self.logs.append
        self.cnc.connect()

    def tearDown(self):
        self.cnc.close()

    def test_step_updates_position_and_live_feed(self):
        self.cnc.set_step("X", 0.5)
        self.cnc.set_feed("X", 1800)
        self.cnc.jog_step("X", +1)
        self.cnc.jog_step("X", +1)
        self.assertAlmostEqual(self.cnc.sim.pos["X"], 1.0, places=3)
        status = self.cnc.refresh_status()
        self.assertAlmostEqual(status.mpos["X"], 1.0, places=3)
        self.assertTrue(any(line.startswith("$J=G91 X0.5000 F1800") for line in self.cnc.sim.lines))

    def test_hold_release_cancels(self):
        self.cnc.hold("Y", +1)
        self._wait_for(lambda: any(line.startswith("$J") and "Y" in line for line in self.cnc.sim.lines))
        self.cnc.release("Y")
        self._wait_for(lambda: self.cnc.sim.cancels >= 1)
        time.sleep(0.25)
        jogs = [line for line in self.cnc.sim.lines if line.startswith("$J")]
        time.sleep(0.35)
        later = [line for line in self.cnc.sim.lines if line.startswith("$J")]
        self.assertEqual(jogs, later)

    def test_feed_change_while_moving(self):
        self.cnc.set_max_feed("Y", 5000)
        self.cnc.hold("Y", +1)
        self._wait_for(lambda: any("F1200" in line for line in self.cnc.sim.lines))
        self.cnc.set_feed("Y", 2400)
        self._wait_for(lambda: any("F2400" in line for line in self.cnc.sim.lines))
        self.cnc.release("Y")

    def test_pump_and_settings(self):
        self.cnc.set_pump(True)
        self.cnc.set_pump(False)
        self.assertEqual(self.cnc.sim.outputs[0], False)
        self.cnc.apply_dm542_timing()
        self.assertEqual(self.cnc.sim.settings[0], 10)
        self.cnc.set_steps_per_unit("X", 320)
        self.assertEqual(self.cnc.sim.settings[100], 320)
        self.assertTrue(self.cnc.driver_warnings() == [] or isinstance(self.cnc.driver_warnings(), list))

    def test_threads_hammer_settings(self):
        def hammer():
            for i in range(30):
                self.cnc.set_step("Z", 0.1 + (i % 4) * 0.1)
                self.cnc.set_max_feed("A", 4000)
                self.cnc.set_feed("A", 500 + (i % 5) * 100)

        self.cnc.hold("X", +1)
        threads = [threading.Thread(target=hammer) for _ in range(3)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.cnc.release_all()
        self.assertGreater(self.cnc.sim.cancels, 0)

    def test_scale_and_program(self):
        self.cnc.scale_steps(2)
        self.assertAlmostEqual(self.cnc.axis("X").step, 2.0)
        sent = self.cnc.run_program("G21\nG91\nG0 X1\n; comment\n")
        self.assertEqual(sent, 3)

    def _wait_for(self, predicate, timeout=1.5):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        self.fail("условие не выполнено")


if __name__ == "__main__":
    unittest.main()
