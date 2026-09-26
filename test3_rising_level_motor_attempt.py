"""Test 3: rising outside-water-level motor-pulse experiment.

With a calibrated empty tank, record both water levels.  When motor actuation
is explicitly enabled, issue one supervised 4-second relay pulse at the first
valid crossing of h_out = 4cm, then 5cm, 6cm, and so on.  Water is supplied by
the experimenter; this script only measures and records it.
"""

import argparse
import csv
import os
import time
from datetime import datetime

import fsm_controller
import relay_controller
import sensor_input
from test2_threshold_measurement import (
    FIELDNAMES,
    append_sample,
    csv_needs_header,
    pressure_snapshot,
)


DEFAULT_CSV = "floodguard_test3_rising_motor_log.csv"
LOOP_INTERVAL_S = 0.2
RISING_MOTOR_FIRST_TRIGGER_H_OUT_CM = 4.0
RISING_MOTOR_STEP_CM = 1.0
RISING_MOTOR_PULSE_S = 4.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test 3 외부 수위 상승별 모터 개방 시도 실험"
    )
    parser.add_argument(
        "--csv",
        default=DEFAULT_CSV,
        help=f"기록할 CSV 경로 (기본값: {DEFAULT_CSV})",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=LOOP_INTERVAL_S,
        help="센서 기록 간격(초, 기본값: 0.2)",
    )
    parser.add_argument(
        "--actuate-motor",
        action="store_true",
        help=(
            "h_out가 4.0cm, 5.0cm, 6.0cm, ...에 최초 도달할 때마다 "
            "릴레이를 4초간 1회 구동합니다. 기본값은 모터 미구동입니다."
        ),
    )
    parser.add_argument(
        "--use-imu",
        action="store_true",
        help="IMU를 활성화해 roll/pitch 품질값과 로그를 기록합니다.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.interval <= 0:
        raise SystemExit("--interval은 0보다 커야 합니다.")
    if RISING_MOTOR_PULSE_S > relay_controller.MAX_RUN_S:
        raise RuntimeError("Test3 모터 펄스 시간이 릴레이 안전 상한을 초과합니다.")
    if not os.path.isfile(sensor_input.CALIBRATION_FILE):
        raise SystemExit(
            f"캘리브레이션 파일이 없습니다: {sensor_input.CALIBRATION_FILE}\n"
            "먼저 빈 수조에서 test2의 --calibrate-empty-tank를 실행하세요."
        )

    reader = sensor_input.SensorReader(use_imu=args.use_imu)
    reader_initialized = False
    relay_initialized = False
    first_threshold_reached_at = ""
    next_motor_trigger_h_out_cm = RISING_MOTOR_FIRST_TRIGGER_H_OUT_CM
    motor_attempt_count = 0

    try:
        reader.init()
        reader_initialized = True
        write_header = csv_needs_header(args.csv)
        if args.actuate_motor:
            relay_controller.init()
            relay_initialized = True

        with open(args.csv, "a", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDNAMES)
            if write_header:
                writer.writeheader()
                file.flush()

            print(f"Test3 센서 기록 시작: {args.csv}")
            print(
                "물을 0cm부터 올리세요. "
                + (
                    f"h_out가 {RISING_MOTOR_FIRST_TRIGGER_H_OUT_CM:.1f}cm, "
                    "5.0cm, 6.0cm, ...에 최초 도달할 때마다 "
                    f"릴레이를 {RISING_MOTOR_PULSE_S:.1f}초 구동합니다. "
                    if args.actuate_motor
                    else "모터는 구동하지 않습니다. "
                )
                + "종료: Ctrl+C"
            )

            while True:
                loop_start = time.monotonic()
                data = reader.read_all()
                sample_timestamp = datetime.now().isoformat(timespec="milliseconds")
                state, reason, sensor_valid = fsm_controller.update(
                    h_out_cm=data["h_out_cm"],
                    h_in_cm=data["h_in_cm"],
                    rise_rate_cm_s=data["rise_rate_out_cm_s"],
                    rise_rate_in_cm_s=data["rise_rate_in_cm_s"],
                    roll_deg=data["roll_deg"],
                    pitch_deg=data["pitch_deg"],
                    imu_valid=data["imu_valid"],
                    sonar_valid=data["sonar_valid"],
                    severe_tilt=data["severe_tilt"],
                )
                f_net_n, pressure_reason, threshold_passed, _ = pressure_snapshot(
                    state, data["h_out_cm"], data["h_in_cm"]
                )
                # test 브랜치의 기존 Test2 helper는 센서값 누락 시 빈 문자열을
                # 반환한다. CSV 기록에서 round() 오류가 나지 않도록 None으로
                # 정규화한다.
                if f_net_n == "":
                    f_net_n = None
                threshold_reached_at = (
                    sample_timestamp if threshold_passed == 1 else ""
                )
                if threshold_reached_at and not first_threshold_reached_at:
                    first_threshold_reached_at = threshold_reached_at

                h_out_cm = data["h_out_cm"]
                actuation = "not_triggered"
                if args.actuate_motor and not data["outside_valid"]:
                    actuation = "waiting_for_valid_h_out"
                elif (
                    args.actuate_motor
                    and h_out_cm is not None
                    and h_out_cm >= next_motor_trigger_h_out_cm
                ):
                    target_h_out_cm = next_motor_trigger_h_out_cm
                    motor_attempt_count += 1
                    relay_result = relay_controller.run_trial_pulse(
                        RISING_MOTOR_PULSE_S
                    )
                    actuation = (
                        f"rising_h_out_{target_h_out_cm:.1f}cm_"
                        f"attempt_{motor_attempt_count}_{relay_result['reason']}"
                    )
                    next_motor_trigger_h_out_cm += RISING_MOTOR_STEP_CM
                elif args.actuate_motor:
                    actuation = (
                        f"waiting_for_rising_h_out_{next_motor_trigger_h_out_cm:.1f}cm"
                    )
                elif threshold_reached_at:
                    actuation = "motor_disabled"

                append_sample(
                    writer, data, state, reason, sensor_valid, f_net_n,
                    pressure_reason, threshold_passed, threshold_reached_at,
                    first_threshold_reached_at, sample_timestamp, actuation,
                )
                file.flush()
                print(
                    f"state={state} ({reason}) | h_out={data['h_out_cm']} cm | "
                    f"h_in={data['h_in_cm']} cm | F_net={f_net_n} N | "
                    f"pressure={pressure_reason} | sensor_valid={sensor_valid} | "
                    f"actuation={actuation}"
                )
                remaining = args.interval - (time.monotonic() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        print("Test3 센서 기록을 종료합니다.")
    finally:
        if relay_initialized:
            relay_controller.close()
        if reader_initialized:
            reader.shutdown()


if __name__ == "__main__":
    main()
