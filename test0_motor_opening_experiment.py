"""Run one supervised motor-only opening pulse.

This test does not read sensors, calculate pressure, run the FSM, or use the
LCD/Arduino path. It drives the relay once after explicit confirmation and
records the operator's observed result.
"""

import argparse
import csv
import os
from datetime import datetime

import relay_controller


CSV_FILE_NAME = "floodguard_test0_motor_log.csv"


def parse_args():
    parser = argparse.ArgumentParser(description="감독하 모터 단독 구동 시험")
    parser.add_argument(
        "--arm-motor", action="store_true",
        help="이 옵션이 없으면 릴레이를 구동하지 않습니다.",
    )
    parser.add_argument(
        "--duration", type=float, default=relay_controller.MAX_RUN_S,
        help=f"릴레이 통전 시간(기본 {relay_controller.MAX_RUN_S:.1f}초)",
    )
    parser.add_argument("--note", default="")
    return parser.parse_args()


def append_result(opened_to_target, duration_s, note, csv_path=CSV_FILE_NAME):
    row = {
        "timestamp_s": datetime.now().isoformat(timespec="seconds"),
        "duration_s": duration_s,
        "opened_to_target": opened_to_target,
        "note": note,
    }
    write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
    with open(csv_path, "a", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def main():
    args = parse_args()
    if not args.arm_motor:
        raise SystemExit("실제 모터 시험은 --arm-motor를 명시해야 합니다.")
    if args.duration <= 0 or args.duration > relay_controller.MAX_RUN_S:
        raise SystemExit(
            f"통전 시간은 0초 초과 {relay_controller.MAX_RUN_S:.1f}초 이하여야 합니다."
        )

    confirmation = input(
        "주변에서 손·도구를 치운 뒤 OPEN을 입력하면 모터를 1회 구동합니다: "
    ).strip()
    if confirmation != "OPEN":
        print("구동을 취소했습니다.")
        return

    relay_controller.init()
    try:
        result = relay_controller.run_trial_pulse(args.duration)
        print(f"모터 구동 완료: {args.duration:.2f}초 ({result['reason']})")
        opened_to_target = input("판이 목표 위치까지 열렸습니까? (yes/no): ").strip().lower()
        if opened_to_target not in ("yes", "no"):
            raise SystemExit("결과는 yes 또는 no여야 합니다.")
        append_result(opened_to_target, args.duration, args.note)
        print(f"시험 결과를 {CSV_FILE_NAME}에 기록했습니다.")
    finally:
        relay_controller.close()


if __name__ == "__main__":
    main()
