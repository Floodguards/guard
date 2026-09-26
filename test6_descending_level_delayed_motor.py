"""Test 6: delayed motor pulse at a selected descending outside-water level.

The test first requires a valid h_out value above the arm level (16 cm by
default). It then waits for h_out to fall to the selected target, records that
crossing, waits 3 seconds, runs a 2-second relay pulse, records 3 more seconds,
and exits. Each execution performs one attempt only.
"""

import argparse
import os
import threading
import time
from datetime import datetime

import relay_controller
import sensor_input
from test5_h_out_16_delayed_motor import (
    FIELDNAMES,
    csv_needs_header,
    write_sample,
)


DEFAULT_ARM_ABOVE_H_OUT_CM = 16.0
DEFAULT_TRIGGER_H_OUT_CM = 16.0
LOOP_INTERVAL_S = 0.1
DELAY_AFTER_DETECTION_S = 3.0
MOTOR_PULSE_S = 2.0
POST_MOTOR_RECORD_S = 3.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test 6: 16cm 초과 후 하강 목표 수위에서 지연 모터 구동"
    )
    parser.add_argument(
        "--trigger-h-out-cm",
        type=float,
        default=DEFAULT_TRIGGER_H_OUT_CM,
        help="하강 중 모터 시도를 시작할 외부 수위(cm, 기본값: 16.0)",
    )
    parser.add_argument(
        "--arm-above-h-out-cm",
        type=float,
        default=DEFAULT_ARM_ABOVE_H_OUT_CM,
        help="하강 실험을 활성화할 초과 수위(cm, 기본값: 16.0)",
    )
    parser.add_argument(
        "--csv",
        default=None,
        help="기록할 CSV 경로 (기본값: 목표 수위별 새 파일)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=LOOP_INTERVAL_S,
        help="기록 간격(초, 기본값: 0.1)",
    )
    parser.add_argument(
        "--use-imu",
        action="store_true",
        help="IMU roll/pitch도 함께 기록합니다.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.interval <= 0:
        raise SystemExit("--interval은 0보다 커야 합니다.")
    if args.trigger_h_out_cm < 0 or args.arm_above_h_out_cm < 0:
        raise SystemExit("수위 기준은 0 이상이어야 합니다.")
    if args.trigger_h_out_cm > args.arm_above_h_out_cm:
        raise SystemExit("하강 목표 수위는 활성화 수위보다 클 수 없습니다.")
    if args.csv is None:
        arm_label = f"{args.arm_above_h_out_cm:g}".replace(".", "p")
        target_label = f"{args.trigger_h_out_cm:g}".replace(".", "p")
        args.csv = (
            f"floodguard_test6_descending_from_{arm_label}_to_{target_label}_motor.csv"
        )
    if MOTOR_PULSE_S > relay_controller.MAX_RUN_S:
        raise RuntimeError("Test6 모터 펄스 시간이 릴레이 안전 상한을 초과합니다.")
    if not os.path.isfile(sensor_input.CALIBRATION_FILE):
        raise SystemExit(
            f"캘리브레이션 파일이 없습니다: {sensor_input.CALIBRATION_FILE}\n"
            "먼저 빈 수조에서 test2의 --calibrate-empty-tank를 실행하세요."
        )

    reader = sensor_input.SensorReader(use_imu=args.use_imu)
    reader_initialized = False
    relay_initialized = False
    armed_at_iso = ""
    target_detected_at_monotonic = None
    target_detected_at_iso = ""
    motor_commanded_at_monotonic = None
    motor_commanded_at_iso = ""
    pulse_thread = None
    pulse_result = {"state": "not_started"}

    def run_pulse():
        try:
            pulse_result["state"] = "running"
            pulse_result["result"] = relay_controller.run_trial_pulse(MOTOR_PULSE_S)
            pulse_result["state"] = "complete"
        except Exception as exc:
            pulse_result["state"] = "error"
            pulse_result["error"] = str(exc)

    try:
        reader.init()
        reader_initialized = True
        relay_controller.init()
        relay_initialized = True
        write_header = csv_needs_header(args.csv)

        import csv

        with open(args.csv, "w" if write_header else "a", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDNAMES)
            if write_header:
                writer.writeheader()

            started_at = time.monotonic()
            print(f"Test6 기록 시작: {args.csv}")
            print(
                f"먼저 h_out > {args.arm_above_h_out_cm:.1f}cm를 기다립니다. "
                f"그 뒤 하강하여 h_out <= {args.trigger_h_out_cm:.1f}cm가 되면 "
                f"{DELAY_AFTER_DETECTION_S:.1f}초 후 모터를 {MOTOR_PULSE_S:.1f}초 구동합니다."
            )

            while True:
                loop_start = time.monotonic()
                elapsed_s = loop_start - started_at
                data = reader.read_all()
                h_out_cm = data["h_out_cm"]
                event = ""

                if (
                    not armed_at_iso
                    and data["outside_valid"]
                    and h_out_cm is not None
                    and h_out_cm > args.arm_above_h_out_cm
                ):
                    armed_at_iso = datetime.now().isoformat(timespec="milliseconds")
                    event = f"h_out_above_{args.arm_above_h_out_cm:g}_armed"

                if (
                    armed_at_iso
                    and target_detected_at_monotonic is None
                    and data["outside_valid"]
                    and h_out_cm is not None
                    and h_out_cm <= args.trigger_h_out_cm
                ):
                    target_detected_at_monotonic = time.monotonic()
                    target_detected_at_iso = datetime.now().isoformat(timespec="milliseconds")
                    event = f"descending_h_out_{args.trigger_h_out_cm:g}_detected"

                if (
                    target_detected_at_monotonic is not None
                    and motor_commanded_at_monotonic is None
                    and time.monotonic() - target_detected_at_monotonic >= DELAY_AFTER_DETECTION_S
                ):
                    motor_commanded_at_monotonic = time.monotonic()
                    motor_commanded_at_iso = datetime.now().isoformat(timespec="milliseconds")
                    pulse_thread = threading.Thread(target=run_pulse, daemon=True)
                    pulse_thread.start()
                    event = "motor_command_requested"

                if not armed_at_iso:
                    phase = "waiting_for_h_out_arm"
                    motor_command = "off"
                elif target_detected_at_monotonic is None:
                    phase = "waiting_for_descending_target"
                    motor_command = "off"
                elif motor_commanded_at_monotonic is None:
                    phase = "delay_after_descending_target"
                    motor_command = "off"
                elif pulse_thread is not None and pulse_thread.is_alive():
                    phase = "motor_pulse"
                    motor_command = "on"
                else:
                    phase = "post_motor_recording"
                    motor_command = pulse_result["state"]

                # Test5-compatible columns preserve the actual selected target,
                # target-detection time, and exact software command-request time.
                write_sample(
                    writer, data, elapsed_s, phase, event,
                    args.trigger_h_out_cm, target_detected_at_iso,
                    motor_commanded_at_iso, motor_command,
                )
                file.flush()
                print(
                    f"t={elapsed_s:.2f}s | phase={phase} | event={event or '-'} | "
                    f"h_out={h_out_cm} cm | motor={motor_command}"
                )

                if (
                    motor_commanded_at_monotonic is not None
                    and pulse_thread is not None
                    and not pulse_thread.is_alive()
                    and time.monotonic() - motor_commanded_at_monotonic
                    >= MOTOR_PULSE_S + POST_MOTOR_RECORD_S
                ):
                    break

                remaining = args.interval - (time.monotonic() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)

            pulse_thread.join()
            print(f"Test6 완료: {args.csv}")
    except KeyboardInterrupt:
        print("Test6를 사용자가 종료했습니다.")
    finally:
        if pulse_thread is not None and pulse_thread.is_alive():
            pulse_thread.join()
        if relay_initialized:
            relay_controller.close()
        if reader_initialized:
            reader.shutdown()


if __name__ == "__main__":
    main()
