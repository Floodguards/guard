"""Hardware-free checks: python3 -m unittest discover -s guard -p check_pi_imu_tests.py"""
import csv
import importlib
import io
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from contextlib import ExitStack, redirect_stdout

import pi_motion
import relay_controller
from live_level_display import LiveLevelDisplay


class PiImuTests(unittest.TestCase):
    def test_live_inside_level_and_sensor_warning(self):
        display = LiveLevelDisplay()
        data = dict(h_out_cm=8.0, h_in_cm=1.0, outside_valid=True,
                    inside_valid=True, loop_time=1.0)
        self.assertIn('h_out=8.00cm | h_in=1.00cm', display.format(data))
        data.update(h_in_cm=3.0, loop_time=1.1)
        self.assertIn('내부 수위 급변 +2.00cm', display.format(data))
        data.update(h_in_cm=None, inside_valid=False, loop_time=1.2)
        self.assertIn('h_in=측정 실패 | 내부 센서 확인 필요', display.format(data))

    def test_interrupted_trial_pulse_turns_relay_off(self):
        class FakeRelay:
            def __init__(self):
                self.active = False

            def on(self):
                self.active = True

            def off(self):
                self.active = False

        relay = FakeRelay()
        stop = threading.Event()
        started = threading.Event()
        with patch.object(relay_controller, '_relay', relay):
            worker = threading.Thread(
                target=relay_controller.run_trial_pulse,
                args=(2.0,),
                kwargs={'stop_event': stop, 'on_started': lambda _: started.set()},
            )
            worker.start()
            self.assertTrue(started.wait(timeout=0.5))
            self.assertTrue(relay.active)
            stop.set()
            worker.join(timeout=0.5)
        self.assertFalse(worker.is_alive())
        self.assertFalse(relay.active)

    def test_confirmation_requires_sustained_post_relay_samples(self):
        listener = pi_motion.PiMotionListener()
        record = lambda accel, start, end: listener._record(accel, start * 500_000, end * 500_000)
        for t in (10, 20, 30):
            record((0.1, 0, 1), t, t + 1)
        listener.arm(40 * 500_000)
        record((0.1, 0, 1), 39, 41)  # read straddles ON
        for t in (50, 60):
            record((0.1, 0, 1), t, t + 1)
        self.assertIsNone(listener.first_event_after(40))
        record((0, 0, 1), 70, 71)  # noise resets confirmation
        for t in (80, 90, 100):
            record((0.1, 0, 1), t, t + 1)
        event = listener.first_event_after(40)
        self.assertEqual(event.received_monotonic_ns, 81 * 500_000)
        self.assertEqual(event.confirmed_monotonic_ns, 101 * 500_000)
        record((0.2, 0, 1), 110, 111)
        self.assertIs(listener.first_event_after(40), event)

    def test_two_ms_polling_does_not_shorten_confirmation(self):
        listener = pi_motion.PiMotionListener()
        listener.arm(0)
        for t in (0, 2, 4, 6, 8):
            listener._record((0.1, 0, 1), t * 1_000_000, t * 1_000_000)
            self.assertIsNone(listener.first_event_after(0))
        listener._record((0.1, 0, 1), 10_000_000, 10_000_000)
        self.assertEqual(listener.first_event_after(0).received_monotonic_ns, 0)
        self.assertEqual(listener.first_event_after(0).confirmed_monotonic_ns, 10_000_000)

    def test_yz_only_and_threshold_equality_do_not_trigger(self):
        for x in (0, 0.05, -0.05):
            listener = pi_motion.PiMotionListener()
            listener.arm(0)
            for t in range(0, 22, 2):
                listener._record((x, 0.9, 1.8), t * 1_000_000, t * 1_000_000)
            self.assertIsNone(listener.first_event_after(0))
            self.assertEqual(listener._latest.ay_g, 0.9)
            self.assertEqual(listener._latest.az_g, 1.8)

    def test_negative_x_relative_to_baseline_and_gap_restart(self):
        listener = pi_motion.PiMotionListener()
        listener._baseline = (-1, 0, 0)
        listener.arm(0)
        for t in (0, 2, 4, 20, 22, 24, 26, 28):
            listener._record((-1.06, 0, 0), t * 1_000_000, t * 1_000_000)
            self.assertIsNone(listener.first_event_after(0))
        listener._record((-1.06, 0, 0), 30_000_000, 30_000_000)
        event = listener.first_event_after(0)
        self.assertEqual(event.received_monotonic_ns, 20_000_000)
        self.assertAlmostEqual(event.x_delta_g, -0.06)
        listener.arm(40_000_000)
        self.assertIsNone(listener.first_event_after(40_000_000))

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
                    listener.first_event_after.side_effect = lambda ns: (
                        pi_motion.MotionEvent(ns + 20_000_000, 'candidate', 0.2, -0.06,
                                              ns + 30_000_000, 'confirmed')
                        if ns is not None else None
                    )
                    stack.enter_context(patch('sys.argv', [name, '--csv', str(path), '--interval', '0.001'] + ([] if enabled else ['--no-imu'])))
                    stack.enter_context(patch.object(module.os.path, 'isfile', return_value=True))
                    ctor = stack.enter_context(patch.object(module.sensor_input, 'SensorReader', return_value=reader))
                    imu_ctor = stack.enter_context(patch.object(module, 'PiMotionListener', return_value=listener))
                    stack.enter_context(patch.object(module.relay_controller, 'init'))
                    stack.enter_context(patch.object(module.relay_controller, 'close'))
                    def pulse(duration, on_started, stop_event=None):
                        on_started(time.monotonic_ns())
                        return {'relay_on': True}
                    relay = stack.enter_context(patch.object(module.relay_controller, 'run_trial_pulse', side_effect=pulse))
                    for constant in ('PRE_MOTOR_RECORD_S', 'DELAY_AFTER_DETECTION_S', 'MOTOR_PULSE_S', 'POST_MOTOR_RECORD_S'):
                        if hasattr(module, constant):
                            stack.enter_context(patch.object(module, constant, 0.002))
                    output = io.StringIO()
                    with redirect_stdout(output):
                        module.main()
                    with path.open(encoding='utf-8') as file:
                        rows = list(csv.DictReader(file))
                    self.assertTrue(rows)
                    self.assertIn('h_in=16.00cm', output.getvalue())
                    self.assertTrue(any(row['motor_relay_on_at'] for row in rows))
                    self.assertEqual(rows[-1]['imu_valid'], '1' if enabled else '0')
                    self.assertIn('motion_pi_monotonic_ns', rows[-1])
                    self.assertNotIn('motion_arduino_micros', rows[-1])
                    ctor.assert_called_once_with(use_imu=False)
                    relay.assert_called_once()
                    reader.shutdown.assert_called_once()
                    if enabled:
                        self.assertEqual(rows[-1]['motion_latency_ms'], '20.0')
                        self.assertEqual(rows[-1]['motion_confirmation_ms'], '10.0')
                        self.assertEqual(rows[-1]['motion_detected_at'], 'candidate')
                        self.assertEqual(rows[-1]['motion_confirmed_at'], 'confirmed')
                        self.assertEqual(rows[-1]['motion_x_delta_g'], '-0.06')
                        listener.start.assert_called_once()
                        listener.arm.assert_called_once()
                        listener.close.assert_called_once()
                    else:
                        imu_ctor.assert_not_called()


if __name__ == '__main__':
    unittest.main()
