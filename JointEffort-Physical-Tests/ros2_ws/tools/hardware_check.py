#!/usr/bin/env python3
"""Check the measured hardware file offline. Does not open a serial port."""
import argparse
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from joint_effort_hardware.configuration import read, description


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('calibration', type=Path)
    args = p.parse_args()
    urdf = Path(get_package_share_directory('joint_effort_iron'))/'urdf/iron.urdf'
    try:
        config = read(args.calibration, urdf)
        description(config, urdf)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        p.exit(1, f'Hardware settings incomplete: {exc}\nNo port opened or motor command sent.\n')
    print('Settings pass offline checks. Physical motion and measured geometry are not yet verified by this check.')
    print('Motor IDs:', [r['motor_id'] for r in config['joints']])


if __name__ == '__main__': main()
