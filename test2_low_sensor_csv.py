"""Record sensor readings and actuate once at the MID/HIGH pressure threshold.

This test reads the two water-level sensors with IMU disabled and appends each
sample to a dedicated CSV. In MID/HIGH, it runs the relay once only when the
rounded force equals the pressure threshold and the outside level is not below
the inside level.
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
        description="센서 기록 및 MID/HIGH 임계값 일치 시 모터 1회 구동 테스트"
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
    return parser.parse_args()


def pressure_snapshot(state, h_out_cm, h_in_cm):
    """Return force, reason, threshold flag, and one-shot open decision."""
    if h_out_cm is None or h_in_cm is None:
        return "", "sensor_value_unavailable", "", False

    # The shared force model clamps negative values to zero. That must not
    # turn reversed water pressure into a false threshold pass in this test.
    if h_out_cm < h_in_cm:
        return (
            pressure_balance.compute_f_net_n(h_out_cm, h_in_cm),
            "reverse_pressure_wait",
            0 if state in ("MID", "HIGH") else "",
            False,
        )

    f_net_n = pressure_balance.compute_f_net_n(h_out_cm, h_in_cm)

    if state in ("MID", "HIGH"):
        # test2 is a calibration point test: only the threshold value itself
        # opens the relay. Values below it must not be treated as an opening
        # condition, unlike the normal main/test1 <= gate.
        threshold_n = pressure_balance.PRESSURE_THRESHOLD_N
        threshold_passed = int(round(f_net_n, 1) == round(threshold_n, 1))
        can_open = bool(threshold_passed)
        pressure_reason = (
            "pressure_threshold_exact_match"
            if can_open
            else "pressure_threshold_exact_wait"
        )
    else:
        threshold_passed = ""
        can_open = False
        pressure_reason = "test2_non_mid_high_wait"
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
    if not os.path.isfile(calibration_path):
        raise SystemExit(
            f"캘리브레이션 파일이 없습니다: {calibration_path}\n"
            "먼저 빈 수조에서 두 수위 센서의 기준값을 보정하세요. IMU 보정은 사용하지 않습니다."
        )

    write_header = csv_needs_header(args.csv)
    reader = sensor_input.SensorReader(use_imu=False)
    reader_initialized = False
    relay_initialized = False
    first_threshold_reached_at = ""
    try:
        reader.init()
        reader_initialized = True
        relay_controller.init()
        relay_initialized = True

        with open(args.csv, "a", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDNAMES)
            if write_header:
                writer.writeheader()
                file.flush()

            print(f"센서 기록 시작: {args.csv}")
            print(
                "MID/HIGH에서 외부 수위가 내부 수위 이상이고 "
                f"F_net = {pressure_balance.PRESSURE_THRESHOLD_N:.1f}N이면 "
                "릴레이를 1회 구동합니다. 종료: Ctrl+C"
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
                if state in ("MID", "HIGH") and can_open:
                    relay_result = relay_controller.run(can_open)
                    actuation = relay_result["reason"]
                elif threshold_reached_at:
                    actuation = "already_opened_or_gate_closed"

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
