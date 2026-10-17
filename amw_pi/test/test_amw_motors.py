"""Hardware-free commissioning regression tests. Run: python -m unittest -v."""
import unittest
from unittest.mock import patch
from amw_pi_bridge.amw_motors import STSBus, BusError, degrees


class FakeSDK:
    def __init__(self, ids=(1,), torque=0):
        self.servos = {i: {3: 777, 5: i, 33: 0, 40: torque, 55: 1, 56: 2048} for i in ids}
        self.writes = []
        self.fail_readdress = False
        self.fault = 0

    def read1ByteTxRx(self, i, reg):
        if i not in self.servos:
            return 0, -6, 0
        return self.servos[i][reg], 0, self.fault

    read2ByteTxRx = read1ByteTxRx

    def ReadPos(self, i):
        return self.read1ByteTxRx(i, 56)

    def write1ByteTxRx(self, i, reg, value):
        self.writes.append((i, reg, value))
        if i not in self.servos:
            return -6, 0
        if reg == 5:
            if self.servos[i][55] != 0:
                raise AssertionError("ID write attempted while EEPROM locked")
            if self.fail_readdress:
                return -6, 0
            servo = self.servos.pop(i)
            servo[5] = value
            self.servos[value] = servo
        else:
            self.servos[i][reg] = value
        return -6, 0  # Model real hardware with no write acknowledgment.

    def getTxRxResult(self, result):
        return f"communication failure {result}"

    def getRxPacketError(self, error):
        return f"servo fault {error}"


class Tests(unittest.TestCase):
    def setUp(self):
        sleeper = patch("amw_pi_bridge.amw_motors.time.sleep")
        sleeper.start()
        self.addCleanup(sleeper.stop)

    def test_encoder_boundary(self):
        self.assertEqual(degrees(0), 0)
        self.assertEqual(degrees(2048), 180)
        self.assertAlmostEqual(degrees(4095), 359.912109375)
        for bad in (-1, 4096, 32767):
            with self.assertRaises(ValueError):
                degrees(bad)

    def test_scan_nonconsecutive_and_no_writes(self):
        sdk = FakeSDK((1, 7, 253))
        found, noisy = STSBus(sdk).discover()
        self.assertEqual(set(found), {1, 7, 253})
        self.assertEqual(noisy, [])
        self.assertEqual(sdk.writes, [])

    def test_failed_position_is_not_zero(self):
        with self.assertRaises(BusError):
            STSBus(FakeSDK()).sample(2)

    def test_fault_not_silently_accepted(self):
        sdk = FakeSDK()
        sdk.fault = 4
        self.assertEqual(STSBus(sdk).discover()[0], {1: 4})
        with self.assertRaises(BusError):
            STSBus(sdk).sample(1)

    def test_angle_not_claimed_for_other_mode(self):
        sdk = FakeSDK()
        sdk.servos[1][33] = 1
        self.assertIsNone(STSBus(sdk).sample(1)[-1])

    def test_multiple_addresses_refused_without_writes(self):
        sdk = FakeSDK((1, 2))
        with self.assertRaises(BusError):
            STSBus(sdk).assign_isolated(1, 3)
        self.assertFalse(sdk.writes)

    def test_torque_enabled_refused(self):
        sdk = FakeSDK(torque=1)
        with self.assertRaises(BusError):
            STSBus(sdk).assign_isolated(1, 2)
        self.assertFalse(sdk.writes)

    def test_readdress_with_missing_ack_verified_and_locked(self):
        sdk = FakeSDK(torque=1)
        STSBus(sdk).assign_isolated(1, 2, disable_torque=True)
        self.assertEqual(set(sdk.servos), {2})
        self.assertEqual(sdk.servos[2][55], 1)
        self.assertEqual(sdk.servos[2][40], 0)
        self.assertEqual(sdk.writes, [(1, 40, 0), (1, 55, 0), (1, 5, 2), (2, 55, 1)])

    def test_failed_id_write_relocks_old_address(self):
        sdk = FakeSDK()
        sdk.fail_readdress = True
        with self.assertRaises(BusError):
            STSBus(sdk).assign_isolated(1, 2)
        self.assertEqual(sdk.servos[1][55], 1)
        self.assertEqual(sum(reg == 5 for _, reg, _ in sdk.writes), 1)

    def test_same_id_is_noop(self):
        sdk = FakeSDK()
        STSBus(sdk).assign_isolated(1, 1)
        self.assertFalse(sdk.writes)

    def test_duplicate_physical_devices_cannot_be_inferred(self):
        # Identical replies from two devices are indistinguishable from one.
        sdk = FakeSDK((1,))
        self.assertEqual(len(STSBus(sdk).discover()[0]), 1)
        # This is why the physical isolation prompt is mandatory for writes.

    def test_broadcast_id_rejected_without_write(self):
        sdk = FakeSDK()
        with self.assertRaises(BusError):
            STSBus(sdk).assign_isolated(1, 254)
        self.assertFalse(sdk.writes)


if __name__ == "__main__":
    unittest.main()
