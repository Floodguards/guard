"""Test 2: measure the pressure threshold and optionally actuate the motor.

This test reads the two water-level sensors and records the threshold
measurement in a dedicated CSV. IMU is optional and is used only for
calibration/logging. It checks the pressure threshold independently of
the FSM MID/HIGH state. Motor actuation is disabled by default; pass
``--actuate-motor`` only for the separate motor run.
"""

import argparse
import csv
import os
import time
from datetime import datetime

import fsm_controller
import pressure_balance
import relay_controller
import sensor_input


DEFAULT_CSV = "floodguard_test2_threshold_motor_log.csv"
LOOP_INTERVAL_S = 0.2

FIELDNAMES = [
    "timestamp",
    "state",
    "fsm_reason",
    "sensor_valid",
    "outside_raw_distance_cm",
    "outside_distance_cm",
    "inside_raw_distance_cm",
    "inside_distance_cm",
    "h_out_cm",
    "h_in_cm",
    "level_difference_cm",
    "rise_rate_out_cm_s",
    "rise_rate_in_cm_s",
    "roll_deg",
    "pitch_deg",
    "outside_valid",
    "inside_valid",
    "imu_valid",
    "sonar_valid",
    "severe_tilt",
    "f_net_n",
    "pressure_threshold_n",
    "pressure_threshold_passed",
    "threshold_reached_at",
    "first_threshold_reached_at",
    "pressure_reason",
    "actuation",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test 2 임계값 측정 실험 (기본: 모터 미구동)"
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
        help="임계값 도달 시 릴레이를 1회 구동합니다. 기본값은 모터 미구동입니다.",
    )
    parser.add_argument(
        "--use-imu",
        action="store_true",
        help="IMU를 활성화해 roll/pitch 품질값과 로그를 기록합니다. 기본값은 비활성화입니다.",
    )
    parser.add_argument(
        "--calibrate-empty-tank",
        action="store_true",
        help="빈 수조에서 외부·내부 센서 기준값을 측정해 공용 캘리브레이션 파일에 저장합니다.",
    )
    return parser.parse_args()


def pressure_snapshot(state, h_out_cm, h_in_cm):
    """Return force and threshold status without requiring MID/HIGH state."""
    if h_out_cm is None or h_in_cm is None:
        return "", "sensor_value_unavailable", "", False

    # Reverse pressure is not a valid threshold-measurement condition.
    if h_out_cm < h_in_cm:
        return (
            pressure_balance.compute_f_net_n(h_out_cm, h_in_cm),
            "reverse_pressure_wait",
            0,
            False,
        )

    f_net_n = pressure_balance.compute_f_net_n(h_out_cm, h_in_cm)

    # This is a threshold measurement test, so do not gate it on the FSM
    # state. Any net pressure at or below the threshold is a pass.
    threshold_n = pressure_balance.PRESSURE_THRESHOLD_N
    threshold_passed = int(f_net_n <= threshold_n)
    can_open = bool(threshold_passed)
    pressure_reason = (
        "pressure_threshold_reached"
        if can_open
        else "pressure_threshold_wait"
    )
    return f_net_n, pressure_reason, threshold_passed, can_open


def append_sample(
    writer,
    data,
    state,
    reason,
    sensor_valid,
    f_net_n,
    pressure_reason,
    threshold_passed,
    threshold_reached_at,
    first_threshold_reached_at,
    sample_timestamp,
    actuation,
):
    writer.writerow(
        {
            "timestamp": sample_timestamp,
            "state": state,
            "fsm_reason": reason,
            "sensor_valid": int(sensor_valid),
            "outside_raw_distance_cm": data["outside_raw_distance_cm"],
            "outside_distance_cm": data["outside_distance_cm"],
            "inside_raw_distance_cm": data["inside_raw_distance_cm"],
            "inside_distance_cm": data["inside_distance_cm"],
            "h_out_cm": data["h_out_cm"],
            "h_in_cm": data["h_in_cm"],
            "level_difference_cm": data["level_difference_cm"],
            "rise_rate_out_cm_s": round(data["rise_rate_out_cm_s"], 3),
            "rise_rate_in_cm_s": round(data["rise_rate_in_cm_s"], 3),
            "roll_deg": round(data["roll_deg"], 2),
            "pitch_deg": round(data["pitch_deg"], 2),
            "outside_valid": int(data["outside_valid"]),
            "inside_valid": int(data["inside_valid"]),
            "imu_valid": int(data["imu_valid"]),
            "sonar_valid": int(data["sonar_valid"]),
            "severe_tilt": int(data["severe_tilt"]),
            "f_net_n": round(f_net_n, 3) if f_net_n != "" else "",
            "pressure_threshold_n": pressure_balance.PRESSURE_THRESHOLD_N,
            "pressure_threshold_passed": threshold_passed,
            "threshold_reached_at": threshold_reached_at,
            "first_threshold_reached_at": first_threshold_reached_at,
            "pressure_reason": pressure_reason,
            "actuation": actuation,
        }
    )


def csv_needs_header(csv_path):
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        return True

    with open(csv_path, newline="", encoding="utf-8") as file:
        existing_header = next(csv.reader(file), None)
    if existing_header != FIELDNAMES:
        raise SystemExit(
            f"CSV 헤더가 현재 형식과 다릅니다: {csv_path}\n"
            "기존 파일을 보존하려면 새 경로로 실행하세요: --csv 새파일.csv"
        )
    return False


def main():
    args = parse_args()
    if args.interval <= 0:
        raise SystemExit("--interval은 0보다 커야 합니다.")

    calibration_path = sensor_input.CALIBRATION_FILE
    if not os.path.isfile(calibration_path) and not args.calibrate_empty_tank:
        raise SystemExit(
            f"캘리브레이션 파일이 없습니다: {calibration_path}\n"
            "먼저 빈 수조에서 두 수위 센서의 기준값을 보정하세요. IMU 보정은 사용하지 않습니다."
        )

    write_header = csv_needs_header(args.csv)
    reader = sensor_input.SensorReader(use_imu=args.use_imu)
    reader_initialized = False
    relay_initialized = False
    first_threshold_reached_at = ""
    try:
        reader.init()
        reader_initialized = True
        if args.calibrate_empty_tank:
            calibration = reader.calibrate_empty_tank()
            print(f"공용 캘리브레이션 저장 완료: {sensor_input.CALIBRATION_FILE}")
            print(calibration)
        if args.actuate_motor:
            relay_controller.init()
            relay_initialized = True

        with open(args.csv, "a", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDNAMES)
            if write_header:
                writer.writeheader()
                file.flush()

            print(f"센서 기록 시작: {args.csv}")
            print(
                "FSM 상태와 관계없이 외부 수위가 내부 수위 이상이고 "
                f"F_net = {pressure_balance.PRESSURE_THRESHOLD_N:.1f}N인지 기록합니다. "
                + ("임계값 도달 시 릴레이를 1회 구동합니다. "
                   if args.actuate_motor else "모터는 구동하지 않습니다. ")
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
                f_net_n, pressure_reason, threshold_passed, can_open = (
                    pressure_snapshot(state, data["h_out_cm"], data["h_in_cm"])
                )

                threshold_reached_at = (
                    sample_timestamp if threshold_passed == 1 else ""
                )
                if threshold_reached_at and not first_threshold_reached_at:
                    first_threshold_reached_at = threshold_reached_at

                actuation = "not_triggered"
                if args.actuate_motor and can_open:
                    relay_result = relay_controller.run(can_open)
                    actuation = relay_result["reason"]
                elif threshold_reached_at:
                    actuation = (
                        "motor_disabled"
                        if not args.actuate_motor
                        else "already_opened_or_gate_closed"
                    )

                append_sample(
                    writer,
                    data,
                    state,
                    reason,
                    sensor_valid,
                    f_net_n,
                    pressure_reason,
                    threshold_passed,
                    threshold_reached_at,
                    first_threshold_reached_at,
                    sample_timestamp,
                    actuation,
                )
                file.flush()

                print(
                    f"state={state} ({reason}) | h_out={data['h_out_cm']} cm | "
                    f"h_in={data['h_in_cm']} cm | F_net={f_net_n} N | "
                    f"pressure={pressure_reason} | "
                    f"threshold_passed={threshold_passed} | "
                    f"threshold_reached_at={threshold_reached_at or 'N/A'} | "
                    f"first_threshold_reached_at={first_threshold_reached_at or 'N/A'} | "
                    f"sensor_valid={sensor_valid} | actuation={actuation}"
                )

                remaining = args.interval - (time.monotonic() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        print("센서 기록을 종료합니다.")
    finally:
        if relay_initialized:
            relay_controller.close()
        if reader_initialized:
            reader.shutdown()


if __name__ == "__main__":
    main()
