"""Test 5: delayed motor pulse after detecting a selected outside water level.

The script records continuously. After a valid h_out measurement reaches the
selected target, it records the detection time, waits 3 seconds while
continuing to log, requests a 2-second relay pulse, records the command time,
then records 3 more seconds. The CSV is directly usable in Excel.
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


DEFAULT_TRIGGER_H_OUT_CM = 16.0
LOOP_INTERVAL_S = 0.1
DELAY_AFTER_DETECTION_S = 3.0
MOTOR_PULSE_S = 2.0
POST_MOTOR_RECORD_S = 3.0

FIELDNAMES = [
    "timestamp",
    "elapsed_s",
    "phase",
    "event",
    "trigger_h_out_cm",
    "h_out_detected_at",
    "motor_command_requested_at",
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
        description="Test 5: 선택한 h_out 감지 후 3초 대기, 2초 모터 구동"
    )
    parser.add_argument(
        "--csv",
        default=None,
        help="기록할 CSV 경로 (기본값: 목표 수위별 새 파일)",
    )
    parser.add_argument(
        "--trigger-h-out-cm",
        type=float,
        default=DEFAULT_TRIGGER_H_OUT_CM,
        help=(
            "모터 시도를 시작할 외부 수위(cm, 기본값: "
            f"{DEFAULT_TRIGGER_H_OUT_CM:.1f})"
        ),
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
            f"CSV 헤더가 현재 Test5 형식과 다릅니다: {csv_path}\n"
            "기존 파일을 보존하려면 --csv 새파일.csv로 실행하세요."
        )
    return False


def rounded_or_blank(value, digits=3):
    return round(value, digits) if isinstance(value, (int, float)) else ""


def pressure_snapshot(h_out_cm, h_in_cm):
    if h_out_cm is None or h_in_cm is None:
        return None
    return pressure_balance.compute_f_net_n(h_out_cm, h_in_cm)


def write_sample(
    writer,
    data,
    elapsed_s,
    phase,
    event,
    trigger_h_out_cm,
    h_out_detected_at,
    motor_command_requested_at,
    motor_command,
):
    h_out_cm = data["h_out_cm"]
    h_in_cm = data["h_in_cm"]
    writer.writerow(
        {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "elapsed_s": rounded_or_blank(elapsed_s),
            "phase": phase,
            "event": event,
            "trigger_h_out_cm": rounded_or_blank(trigger_h_out_cm),
            "h_out_detected_at": h_out_detected_at,
            "motor_command_requested_at": motor_command_requested_at,
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
    if args.trigger_h_out_cm < 0:
        raise SystemExit("--trigger-h-out-cm은 0 이상이어야 합니다.")
    if args.csv is None:
        target_label = f"{args.trigger_h_out_cm:g}".replace(".", "p")
        args.csv = f"floodguard_test5_h_out_{target_label}_delayed_motor.csv"
    if MOTOR_PULSE_S > relay_controller.MAX_RUN_S:
        raise RuntimeError("Test5 모터 펄스 시간이 릴레이 안전 상한을 초과합니다.")
    if not os.path.isfile(sensor_input.CALIBRATION_FILE):
        raise SystemExit(
            f"캘리브레이션 파일이 없습니다: {sensor_input.CALIBRATION_FILE}\n"
            "먼저 빈 수조에서 test2의 --calibrate-empty-tank를 실행하세요."
        )

    reader = sensor_input.SensorReader(use_imu=args.use_imu)
    reader_initialized = False
    relay_initialized = False
    detected_at_monotonic = None
    detected_at_iso = ""
    motor_commanded_at_monotonic = None
    motor_commanded_at_iso = ""
    pulse_thread = None
    pulse_result = {"state": "not_started"}

    def run_pulse():
        try:
            pulse_result["state"] = "running"
            pulse_result["result"] = relay_controller.run_trial_pulse(MOTOR_PULSE_S)
            pulse_result["state"] = "complete"
        except Exception as exc:  # Preserve logs even if relay control fails.
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
            print(f"Test5 기록 시작: {args.csv}")
            print(
                f"유효한 h_out >= {args.trigger_h_out_cm:.1f}cm를 기다립니다. "
                f"감지 후 {DELAY_AFTER_DETECTION_S:.1f}초 뒤 모터를 "
                f"{MOTOR_PULSE_S:.1f}초 구동합니다. 종료: Ctrl+C"
            )

            while True:
                loop_start = time.monotonic()
                elapsed_s = loop_start - started_at
                data = reader.read_all()
                event = ""
                h_out_cm = data["h_out_cm"]

                if (
                    detected_at_monotonic is None
                    and data["outside_valid"]
                    and h_out_cm is not None
                    and h_out_cm >= args.trigger_h_out_cm
                ):
                    detected_at_monotonic = time.monotonic()
                    detected_at_iso = datetime.now().isoformat(timespec="milliseconds")
                    event = f"h_out_{args.trigger_h_out_cm:g}_detected"

                if (
                    detected_at_monotonic is not None
                    and motor_commanded_at_monotonic is None
                    and time.monotonic() - detected_at_monotonic >= DELAY_AFTER_DETECTION_S
                ):
                    motor_commanded_at_monotonic = time.monotonic()
                    motor_commanded_at_iso = datetime.now().isoformat(timespec="milliseconds")
                    pulse_thread = threading.Thread(target=run_pulse, daemon=True)
                    pulse_thread.start()
                    event = "motor_command_requested"

                if detected_at_monotonic is None:
                    phase = "waiting_for_h_out_target"
                    motor_command = "off"
                elif motor_commanded_at_monotonic is None:
                    phase = "delay_after_h_out_target"
                    motor_command = "off"
                elif pulse_thread is not None and pulse_thread.is_alive():
                    phase = "motor_pulse"
                    motor_command = "on"
                else:
                    phase = "post_motor_recording"
                    motor_command = pulse_result["state"]

                write_sample(
                    writer, data, elapsed_s, phase, event,
                    args.trigger_h_out_cm, detected_at_iso,
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
            print(f"Test5 완료: {args.csv}")
    except KeyboardInterrupt:
        print("Test5를 사용자가 종료했습니다.")
    finally:
        if pulse_thread is not None and pulse_thread.is_alive():
            pulse_thread.join()
        if relay_initialized:
            relay_controller.close()
        if reader_initialized:
            reader.shutdown()


if __name__ == "__main__":
    main()
