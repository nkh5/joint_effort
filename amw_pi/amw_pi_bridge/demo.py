"""Offline illustrative contract fixture; these numbers are NOT robot calibration."""
import json
from .contract import NAMES, validate_config, preview_trajectory, sample_all


def fixture():
    return {'schema': 1, 'calibration_confirmed': True,
            'serial_port': '/dev/SIMULATION_ONLY', 'baud_rate': 1000000,
            'joints': [dict(name=n, motor_id=i+1, model_number=777,
                            zero_counts=2048, direction=-1, gear_ratio=1.,
                            min_counts=100, max_counts=3995,
                            min_rad=-1., max_rad=1., max_velocity_rad_s=.15)
                       for i, n in enumerate(NAMES)]}


class FakeBus:
    def sample(self, ident):
        return 777, 0, 0, 2048, 180.


def main():
    config = fixture()
    validate_config(config)
    points = [{'positions': [.01]*5, 'velocities': [.05]*5, 'time_s': .2}]
    print(json.dumps({'SIMULATION_ONLY': True, 'hardware_tested': False,
                      'measured_fixture': sample_all(FakeBus(), config),
                      'trajectory_preview': preview_trajectory(config, list(reversed(NAMES)), points)}, indent=2))


if __name__ == '__main__':
    main()
