"""Record live sensor readings and FSM state for experiment 2.

This standalone recorder reads the two water-level sensors (IMU disabled),
then appends readings to a dedicated CSV. It never initializes or calls the
Arduino/output controller or the relay controller, so LOW cannot actuate
the window during this test.
"""

import argparse
import csv
import os
import time
from datetime import datetime

import fsm_controller
import pressure_balance
import sensor_input


DEFAULT_CSV = "floodguard_test2_low_sensor_log.csv"
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
    "pressure_reason",
    "actuation",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="LOW 수조 센서 기록 전용 테스트 (릴레이/출력 구동 없음)"
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
    """Report the pressure gate without initializing or actuating outputs."""
    if h_out_cm is None or h_in_cm is None:
        return "", "sensor_value_unavailable", ""

    _can_open, f_net_n, pressure_reason = pressure_balance.can_open(
        state, h_out_cm, h_in_cm
    )
    if state in ("MID", "HIGH"):
        threshold_passed = int(pressure_reason == "pressure_balanced_open")
    else:
        threshold_passed = ""
    return f_net_n, pressure_reason, threshold_passed


def append_sample(
    writer, data, state, reason, sensor_valid,
    f_net_n, pressure_reason, threshold_passed,
):
    h_out_cm = data["h_out_cm"]
    h_in_cm = data["h_in_cm"]
    writer.writerow({
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "state": state,
        "fsm_reason": reason,
        "sensor_valid": int(sensor_valid),
        "outside_raw_distance_cm": data["outside_raw_distance_cm"],
        "outside_distance_cm": data["outside_distance_cm"],
        "inside_raw_distance_cm": data["inside_raw_distance_cm"],
        "inside_distance_cm": data["inside_distance_cm"],
        "h_out_cm": h_out_cm,
        "h_in_cm": h_in_cm,
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
        "pressure_reason": pressure_reason,
        "actuation": "disabled",
    })


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
    initialized = True
    try:
        reader.init()
        with open(args.csv, "a", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDNAMES)
            if write_header:
                writer.writeheader()
                file.flush()

            print(f"센서 기록 시작: {args.csv}")
            print("출력·릴레이는 연결하거나 작동하지 않습니다. 종료: Ctrl+C")
            while True:
                loop_start = time.monotonic()
                data = reader.read_all()
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
                f_net_n, pressure_reason, threshold_passed = pressure_snapshot(
                    state, data["h_out_cm"], data["h_in_cm"]
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
                )
                file.flush()

                print(
                    f"state={state} ({reason}) | h_out={data['h_out_cm']} cm | "
                    f"h_in={data['h_in_cm']} cm | "
                    f"F_net={f_net_n} N | pressure={pressure_reason} | "
                    f"threshold_passed={threshold_passed} | "
                    f"sensor_valid={sensor_valid} | actuation=disabled"
                )

                remaining = args.interval - (time.monotonic() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        print("센서 기록을 종료합니다.")
    finally:
        if initialized:
            reader.shutdown()


if __name__ == "__main__":
    main()
