"""Supervised real-motor trials used to estimate the opening-load boundary.

This is intentionally separate from main.py.  It reads only the two water
levels, never runs the FSM, Arduino, LCD, or automatic-opening path, and makes
one relay pulse only after an operator types OPEN.  The physical target-open
result is recorded by the operator because no position sensor exists.
"""

import argparse
import csv
import os
import time
from datetime import datetime

import relay_controller
import sensor_input


RHO_WATER_KG_M3 = 1000.0
GRAVITY_M_S2 = 9.81

# Fixed pilot apparatus and operating procedure.
BOTTOM_WIDTH_CM = 36.4
TOP_WIDTH_CM = 33.3
PANEL_HEIGHT_CM = 21.6
LEVEL_SAMPLE_COUNT = 5
LEVEL_SAMPLE_INTERVAL_S = 0.2
TRIAL_RELAY_RUN_S = 5.5
CSV_FILE_NAME = "floodguard_motor_opening_trials.csv"


def one_side_force_n(level_cm):
    """Hydrostatic force on one side of the actual vertical trapezoid panel."""
    if level_cm < 0 or level_cm > PANEL_HEIGHT_CM:
        raise ValueError("수위가 이번 부분 침수 모델 범위(0~21.6cm)를 벗어났습니다.")
    h_m = level_cm / 100.0
    bottom_width_m = BOTTOM_WIDTH_CM / 100.0
    panel_height_m = PANEL_HEIGHT_CM / 100.0
    width_slope = ((TOP_WIDTH_CM - BOTTOM_WIDTH_CM) / 100.0) / panel_height_m
    return RHO_WATER_KG_M3 * GRAVITY_M_S2 * (
        bottom_width_m * h_m ** 2 / 2.0 + width_slope * h_m ** 3 / 6.0
    )


def read_level_samples(reader):
    samples = []
    for _ in range(LEVEL_SAMPLE_COUNT):
        data = reader.read_all()
        h_out_cm = data.get("h_out_cm")
        h_in_cm = data.get("h_in_cm")
        if h_out_cm is None or h_in_cm is None:
            raise RuntimeError("외부·내부 수위가 모두 유효할 때만 모터 시험을 시작할 수 있습니다.")
        samples.append((float(h_out_cm), float(h_in_cm)))
        time.sleep(LEVEL_SAMPLE_INTERVAL_S)
    return samples


def _mean(values):
    return sum(values) / len(values)


def _append_row(row, csv_path=CSV_FILE_NAME):
    write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
    with open(csv_path, "a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def parse_args():
    parser = argparse.ArgumentParser(description="실제 모터 개방 한계 추정용 감독하 시험 1회")
    parser.add_argument("--arm-motor", action="store_true",
                        help="없으면 릴레이를 절대 켜지 않는 안전 잠금")
    parser.add_argument("--trial-id", help="비우면 실행 시각 기반 ID를 자동 생성")
    parser.add_argument("--note", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.arm_motor:
        raise SystemExit("실제 모터 시험은 --arm-motor를 명시해야 합니다.")
    if TRIAL_RELAY_RUN_S > relay_controller.MAX_RUN_S:
        raise RuntimeError("고정 시험 통전 시간이 릴레이 안전 상한을 넘습니다.")

    trial_id = args.trial_id or "motor-" + datetime.now().strftime("%Y%m%d-%H%M%S")
    reader = sensor_input.SensorReader(dry_run=False, use_imu=False)
    relay_ready = False
    try:
        reader.init()
        relay_controller.init(dry_run=False)
        relay_ready = True

        samples = read_level_samples(reader)
        outside_levels = [sample[0] for sample in samples]
        inside_levels = [sample[1] for sample in samples]
        h_out_cm = _mean(outside_levels)
        h_in_cm = _mean(inside_levels)
        outside_force_n = one_side_force_n(h_out_cm)
        inside_force_n = one_side_force_n(h_in_cm)
        f_net_signed_n = outside_force_n - inside_force_n

        print(
            f"trial={trial_id} | h_out={h_out_cm:.2f}cm (range {min(outside_levels):.2f}-"
            f"{max(outside_levels):.2f}) | h_in={h_in_cm:.2f}cm (range "
            f"{min(inside_levels):.2f}-{max(inside_levels):.2f})"
        )
        print(f"F_net_signed={f_net_signed_n:.2f}N | relay={TRIAL_RELAY_RUN_S:.2f}s")
        confirmation = input("손·도구를 구동부에서 치운 뒤 OPEN을 입력하면 1회만 구동합니다: ").strip()
        if confirmation != "OPEN":
            print("구동을 취소했습니다. CSV에 기록하지 않았습니다.")
            return

        relay_result = relay_controller.run_trial_pulse(TRIAL_RELAY_RUN_S, dry_run=False)
        opened_to_target = input("판이 목표 위치까지 실제로 열렸습니까? (yes/no): ").strip().lower()
        if opened_to_target not in ("yes", "no"):
            raise ValueError("결과는 yes 또는 no여야 합니다.")

        _append_row({
            "timestamp_s": datetime.now().isoformat(timespec="seconds"),
            "trial_id": trial_id,
            "h_out_samples_cm": ";".join(f"{value:.3f}" for value in outside_levels),
            "h_in_samples_cm": ";".join(f"{value:.3f}" for value in inside_levels),
            "h_out_cm_mean": round(h_out_cm, 3),
            "h_in_cm_mean": round(h_in_cm, 3),
            "h_out_cm_range": round(max(outside_levels) - min(outside_levels), 3),
            "h_in_cm_range": round(max(inside_levels) - min(inside_levels), 3),
            "bottom_width_cm": BOTTOM_WIDTH_CM,
            "top_width_cm": TOP_WIDTH_CM,
            "panel_height_cm": PANEL_HEIGHT_CM,
            "outside_hydrostatic_force_n": round(outside_force_n, 3),
            "inside_hydrostatic_force_n": round(inside_force_n, 3),
            "f_net_signed_n": round(f_net_signed_n, 3),
            "relay_run_s": TRIAL_RELAY_RUN_S,
            "opened_to_target": opened_to_target,
            "relay_reason": relay_result["reason"],
            "note": args.note,
        })
        print(f"시험 결과를 {CSV_FILE_NAME}에 기록했습니다.")
    finally:
        if relay_ready:
            relay_controller.close()
        reader.shutdown()


if __name__ == "__main__":
    main()
