"""Hardware-free checks: python3 -m unittest discover -s guard -p check_pi_imu_tests.py"""
import csv
import importlib
import io
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from contextlib import ExitStack, redirect_stdout

import pi_motion


class PiImuTests(unittest.TestCase):
    def test_confirmation_requires_three_post_relay_samples(self):
        listener = pi_motion.PiMotionListener()
        for t in (10, 20, 30):
            listener._record((0.1, 0, 1), t, t + 1)
        listener.arm(40)
        listener._record((0.1, 0, 1), 39, 41)  # read straddles ON
        for t in (50, 60):
            listener._record((0.1, 0, 1), t, t + 1)
        self.assertIsNone(listener.first_event_after(40))
        listener._record((0, 0, 1), 70, 71)  # noise resets confirmation
        for t in (80, 90, 100):
            listener._record((0.1, 0, 1), t, t + 1)
        event = listener.first_event_after(40)
        self.assertEqual(event.received_monotonic_ns, 101)
        listener._record((0.2, 0, 1), 110, 111)
        self.assertIs(listener.first_event_after(40), event)

    def test_signed_acceleration_and_stale_sample(self):
        listener = pi_motion.PiMotionListener()
        from unittest.mock import Mock
        listener._bus = Mock()
        listener._bus.read_i2c_block_data.return_value = [0xC0, 0, 0, 0, 0x40, 0]
        self.assertEqual(listener._read_accel(), (-1.0, 0.0, 1.0))
        listener._record((0, 0, 1), 1, 2)
        self.assertIsNone(listener.latest_raw_sample())
        data = {}
        pi_motion.update_imu_data(data, None)
        self.assertFalse(data['imu_valid'])
        self.assertIsNone(data['roll_deg'])

    def test_start_failure_closes_bus(self):
        from unittest.mock import Mock
        bus = Mock()
        bus.write_byte_data.side_effect = OSError('disconnected')
        with patch.object(pi_motion, 'SMBus', return_value=bus):
            with self.assertRaises(OSError):
                pi_motion.PiMotionListener().start()
        bus.close.assert_called_once()

    def test_all_experiments_with_and_without_imu(self):
        from unittest.mock import Mock
        names = ['test4_equal_level_opening_time', 'test5_h_out_16_delayed_motor',
                 'test6_descending_level_delayed_motor']
        for name in names:
            for enabled in (True, False):
                with self.subTest(name=name, imu=enabled), tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
                    module = importlib.import_module(name)
                    path = Path(directory) / 'result.csv'
                    reader = Mock()
                    count = [0]

                    def read_all():
                        count[0] += 1
                        h = 17 if count[0] == 1 else 16
                        return dict(h_out_cm=h, h_in_cm=16, level_difference_cm=h-16,
                                    outside_raw_distance_cm=13, outside_distance_cm=13,
                                    inside_raw_distance_cm=14, inside_distance_cm=14,
                                    outside_valid=True, inside_valid=True)

                    reader.read_all.side_effect = read_all
                    listener = Mock()
                    sample = pi_motion.ImuSample(time.monotonic_ns(), 'sample', 0, 0, 1, 0)
                    listener.latest_raw_sample.return_value = sample
                    listener.first_event_after.return_value = None
                    stack.enter_context(patch('sys.argv', [name, '--csv', str(path), '--interval', '0.001'] + ([] if enabled else ['--no-imu'])))
                    stack.enter_context(patch.object(module.os.path, 'isfile', return_value=True))
                    ctor = stack.enter_context(patch.object(module.sensor_input, 'SensorReader', return_value=reader))
                    imu_ctor = stack.enter_context(patch.object(module, 'PiMotionListener', return_value=listener))
                    stack.enter_context(patch.object(module.relay_controller, 'init'))
                    stack.enter_context(patch.object(module.relay_controller, 'close'))
                    def pulse(duration, on_started):
                        on_started(time.monotonic_ns())
                        return {'relay_on': True}
                    relay = stack.enter_context(patch.object(module.relay_controller, 'run_trial_pulse', side_effect=pulse))
                    for constant in ('PRE_MOTOR_RECORD_S', 'DELAY_AFTER_DETECTION_S', 'MOTOR_PULSE_S', 'POST_MOTOR_RECORD_S'):
                        if hasattr(module, constant):
                            stack.enter_context(patch.object(module, constant, 0.002))
                    with redirect_stdout(io.StringIO()):
                        module.main()
                    with path.open(encoding='utf-8') as file:
                        rows = list(csv.DictReader(file))
                    self.assertTrue(rows)
                    self.assertTrue(any(row['motor_relay_on_at'] for row in rows))
                    self.assertEqual(rows[-1]['imu_valid'], '1' if enabled else '0')
                    self.assertIn('motion_pi_monotonic_ns', rows[-1])
                    self.assertNotIn('motion_arduino_micros', rows[-1])
                    ctor.assert_called_once_with(use_imu=False)
                    relay.assert_called_once()
                    reader.shutdown.assert_called_once()
                    if enabled:
                        listener.start.assert_called_once()
                        listener.arm.assert_called_once()
                        listener.close.assert_called_once()
                    else:
                        imu_ctor.assert_not_called()


if __name__ == '__main__':
    unittest.main()
