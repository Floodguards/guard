"""Test 4: record a no-pressure-difference opening attempt.

The operator first makes the outside and inside water levels equal.  This
script records from the start, waits 3 seconds, drives the relay for 2 seconds,
then records a further 3 seconds and exits.  The resulting CSV opens directly
in Excel.  It records the command timing; physical panel-open completion still
needs to be observed by video or an independent position sensor.
"""

import argparse
import csv
import os
import threading
import time
from datetime import datetime

import pressure_balance
import relay_controller
import sensor_input


DEFAULT_CSV = "floodguard_test4_equal_level_opening_time.csv"
LOOP_INTERVAL_S = 0.1
PRE_MOTOR_RECORD_S = 3.0
MOTOR_PULSE_S = 2.0
POST_MOTOR_RECORD_S = 3.0

FIELDNAMES = [
    "timestamp",
    "elapsed_s",
    "phase",
    "motor_command",
    "h_out_cm",
    "h_in_cm",
    "level_difference_cm",
    "f_net_n",
    "outside_raw_distance_cm",
    "outside_distance_cm",
    "inside_raw_distance_cm",
    "inside_distance_cm",
    "outside_valid",
    "inside_valid",
    "sensor_valid",
    "roll_deg",
    "pitch_deg",
    "imu_valid",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test 4 수압차 없는 조건의 2초 창문 개방시간 기록"
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
        help="기록 간격(초, 기본값: 0.1)",
    )
    parser.add_argument(
        "--use-imu",
        action="store_true",
        help="IMU roll/pitch도 함께 기록합니다.",
    )
    return parser.parse_args()


def csv_needs_header(csv_path):
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        return True
    with open(csv_path, newline="", encoding="utf-8") as file:
        existing_header = next(csv.reader(file), None)
    if existing_header != FIELDNAMES:
        raise SystemExit(
            f"CSV 헤더가 현재 Test4 형식과 다릅니다: {csv_path}\n"
            "기존 파일을 보존하려면 --csv 새파일.csv로 실행하세요."
        )
    return False


def rounded_or_blank(value, digits=3):
    return round(value, digits) if isinstance(value, (int, float)) else ""


def pressure_snapshot(h_out_cm, h_in_cm):
    if h_out_cm is None or h_in_cm is None:
        return None
    return pressure_balance.compute_f_net_n(h_out_cm, h_in_cm)


def write_sample(writer, data, elapsed_s, phase, motor_command):
    h_out_cm = data["h_out_cm"]
    h_in_cm = data["h_in_cm"]
    writer.writerow(
        {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "elapsed_s": rounded_or_blank(elapsed_s),
            "phase": phase,
            "motor_command": motor_command,
            "h_out_cm": h_out_cm if h_out_cm is not None else "",
            "h_in_cm": h_in_cm if h_in_cm is not None else "",
            "level_difference_cm": rounded_or_blank(data["level_difference_cm"]),
            "f_net_n": rounded_or_blank(pressure_snapshot(h_out_cm, h_in_cm)),
            "outside_raw_distance_cm": data["outside_raw_distance_cm"],
            "outside_distance_cm": data["outside_distance_cm"],
            "inside_raw_distance_cm": data["inside_raw_distance_cm"],
            "inside_distance_cm": data["inside_distance_cm"],
            "outside_valid": int(data["outside_valid"]),
            "inside_valid": int(data["inside_valid"]),
            "sensor_valid": int(data["outside_valid"] and data["inside_valid"]),
            "roll_deg": rounded_or_blank(data["roll_deg"], 2),
            "pitch_deg": rounded_or_blank(data["pitch_deg"], 2),
            "imu_valid": int(data["imu_valid"]),
        }
    )


def main():
    args = parse_args()
    if args.interval <= 0:
        raise SystemExit("--interval은 0보다 커야 합니다.")
    if MOTOR_PULSE_S > relay_controller.MAX_RUN_S:
        raise RuntimeError("Test4 모터 펄스 시간이 릴레이 안전 상한을 초과합니다.")
    if not os.path.isfile(sensor_input.CALIBRATION_FILE):
        raise SystemExit(
            f"캘리브레이션 파일이 없습니다: {sensor_input.CALIBRATION_FILE}\n"
            "먼저 빈 수조에서 test2의 --calibrate-empty-tank를 실행하세요."
        )

    reader = sensor_input.SensorReader(use_imu=args.use_imu)
    reader_initialized = False
    relay_initialized = False
    pulse_started = False
    pulse_result = {"state": "not_started"}
    pulse_thread = None

    def run_pulse():
        try:
            pulse_result["state"] = "running"
            pulse_result["result"] = relay_controller.run_trial_pulse(MOTOR_PULSE_S)
            pulse_result["state"] = "complete"
        except Exception as exc:  # Keep a sensor log even if relay control fails.
            pulse_result["state"] = "error"
            pulse_result["error"] = str(exc)

    try:
        reader.init()
        reader_initialized = True
        relay_controller.init()
        relay_initialized = True
        write_header = csv_needs_header(args.csv)

        with open(args.csv, "w" if write_header else "a", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=FIELDNAMES)
            if write_header:
                writer.writeheader()

            started_at = time.monotonic()
            total_duration_s = PRE_MOTOR_RECORD_S + MOTOR_PULSE_S + POST_MOTOR_RECORD_S
            print(f"Test4 기록 시작: {args.csv}")
            print(
                "양쪽 수위를 맞춘 뒤 그대로 두세요. "
                f"{PRE_MOTOR_RECORD_S:.0f}초 후 모터를 {MOTOR_PULSE_S:.0f}초 구동하고 "
                f"{POST_MOTOR_RECORD_S:.0f}초 더 기록합니다."
            )

            while True:
                loop_start = time.monotonic()
                elapsed_s = loop_start - started_at
                if not pulse_started and elapsed_s >= PRE_MOTOR_RECORD_S:
                    pulse_started = True
                    pulse_thread = threading.Thread(target=run_pulse, daemon=True)
                    pulse_thread.start()

                if not pulse_started:
                    phase = "pre_motor_recording"
                    motor_command = "off"
                elif pulse_thread is not None and pulse_thread.is_alive():
                    phase = "motor_pulse"
                    motor_command = "on"
                else:
                    phase = "post_motor_recording"
                    motor_command = pulse_result["state"]

                data = reader.read_all()
                write_sample(writer, data, elapsed_s, phase, motor_command)
                file.flush()
                print(
                    f"t={elapsed_s:.2f}s | phase={phase} | motor={motor_command} | "
                    f"h_out={data['h_out_cm']} cm | h_in={data['h_in_cm']} cm | "
                    f"F_net={pressure_snapshot(data['h_out_cm'], data['h_in_cm'])} N"
                )

                if elapsed_s >= total_duration_s:
                    break
                remaining = args.interval - (time.monotonic() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)

            if pulse_thread is not None:
                pulse_thread.join()
            print(f"Test4 완료: {args.csv}")
    except KeyboardInterrupt:
        print("Test4를 사용자가 종료했습니다.")
    finally:
        if pulse_thread is not None and pulse_thread.is_alive():
            pulse_thread.join()
        if relay_initialized:
            relay_controller.close()
        if reader_initialized:
            reader.shutdown()


if __name__ == "__main__":
    main()
