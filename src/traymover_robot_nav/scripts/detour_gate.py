#!/usr/bin/env python3
"""Compatibility imports for the production supervisor's shared detour core."""
import importlib.util
from pathlib import Path
import sys

try:
    import detour_core as _core
except ModuleNotFoundError as error:
    if error.name != 'detour_core':
        raise
    # Support existing file-spec test loaders as well as installed ROS executables.
    _spec = importlib.util.spec_from_file_location(
        'detour_core', Path(__file__).with_name('detour_core.py'))
    _core = importlib.util.module_from_spec(_spec)
    sys.modules[_spec.name] = _core
    _spec.loader.exec_module(_core)

for _name in ('AUTO_ESTOP_STOP_DISTANCE_M', 'Pose', 'Velocity', 'Ray', 'Scan',
              'Config', 'Observation', 'Candidate', 'angle_difference', 'Scene',
              'DetourGate', 'turn_command', '_valid_ray', '_hit_point'):
    globals()[_name] = getattr(_core, _name)
