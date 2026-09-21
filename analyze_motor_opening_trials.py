"""Summarize the observed bracket for the real motor opening boundary."""

import csv
from pathlib import Path


CSV_FILE_NAME = "floodguard_motor_opening_trials.csv"


def main():
    path = Path(CSV_FILE_NAME)
    if not path.exists():
        raise SystemExit(f"{CSV_FILE_NAME}이 없습니다.")

    successes = []
    failures = []
    with path.open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            force_n = float(row["f_net_signed_n"])
            if row["opened_to_target"] == "yes":
                successes.append(force_n)
            elif row["opened_to_target"] == "no":
                failures.append(force_n)

    print(f"성공 {len(successes)}회 / 실패 {len(failures)}회")
    if successes:
        print("가장 큰 성공 순수압: %.2f N" % max(successes))
    if failures:
        print("가장 작은 실패 순수압: %.2f N" % min(failures))

    if successes and failures:
        lower_n = max(successes)
        upper_n = min(failures)
        if lower_n < upper_n:
            print("관측된 개방 한계 후보 구간: %.2f N <= F_cap,actual < %.2f N" % (lower_n, upper_n))
        else:
            print("성공·실패 순서가 겹칩니다. 같은 수위차 조건을 3회 이상 재시험해야 합니다.")
    elif successes:
        print("현재 결과는 개방 한계의 하한만 보여 줍니다. 실패 조건이 아직 없습니다.")
    else:
        print("성공 조건이 아직 없습니다. 센서값·기구 걸림·통전시간을 먼저 확인하세요.")


if __name__ == "__main__":
    main()
