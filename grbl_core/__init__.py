"""Ядро управления станком: Raspberry Pi → grbl-Mega-5X → DM542.

Импорт:

    from grbl_core import GrblCore

    cnc = GrblCore()
    cnc.connect()
    cnc.apply_machine()   # числа из grbl_core/machine_config.py
"""

from grbl_core.core import GrblCore, GrblError
from grbl_core.profile import (
    DIGITAL_OUTPUT_PINS,
    DM542_MAX_STEP_HZ,
    DM542_MIN_PULSE_US,
    DM542_RECOMMENDED_PULSE_US,
    MEGA_PINS,
    Axis,
    ballscrew_steps_per_mm,
    default_axes,
    describe_current,
    describe_microstep,
    rotary_steps_per_deg,
    step_frequency_hz,
)

__all__ = [
    "GrblCore",
    "GrblError",
    "Axis",
    "default_axes",
    "ballscrew_steps_per_mm",
    "rotary_steps_per_deg",
    "step_frequency_hz",
    "describe_microstep",
    "describe_current",
    "MEGA_PINS",
    "DIGITAL_OUTPUT_PINS",
    "DM542_MAX_STEP_HZ",
    "DM542_MIN_PULSE_US",
    "DM542_RECOMMENDED_PULSE_US",
]
