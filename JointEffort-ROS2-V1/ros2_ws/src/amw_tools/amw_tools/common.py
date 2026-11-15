"""Paths and small helpers shared by the amw tools."""
import math
import os
import sys
from pathlib import Path

# ros2_ws: set by the `amw` launcher; otherwise found from this file (source tree or symlink install).
WS = Path(os.environ.get('AMW_WS') or Path(__file__).resolve().parents[3])
SRC = WS / 'src'
ARM_YAML = SRC / 'amw_bringup/config/arm.yaml'
GAINS_YAML = SRC / 'amw_bringup/config/gains.yaml'
URDF = SRC / 'amw_description/urdf/amw.urdf'
SRDF = SRC / 'amw_description/srdf/amw.srdf'

for pkg in ('amw_bringup', 'amw_teleop'):            # importable before `amw build`
    if str(SRC / pkg) not in sys.path and (SRC / pkg).is_dir():
        sys.path.insert(0, str(SRC / pkg))

from amw_bringup.arm import JOINTS, ROLES, ArmConfigError, load  # noqa: E402


def load_arm(path=None):
    try:
        return load(path or ARM_YAML)
    except ArmConfigError as e:
        raise SystemExit(f'arm.yaml: {e}')


def deg(rad):
    return math.degrees(rad)


def open_bus(arm, port=None):
    from .stsbus import Bus, BusError
    try:
        return Bus(port or arm.serial_port, arm.baud_rate)
    except BusError as e:
        raise SystemExit(str(e))


def ask(question):
    try:
        return input(question + ' [y/N] ').strip().lower() in ('y', 'yes')
    except EOFError:
        return False


__all__ = ['WS', 'SRC', 'ARM_YAML', 'GAINS_YAML', 'URDF', 'SRDF', 'JOINTS', 'ROLES', 'load_arm', 'deg',
           'open_bus', 'ask']
