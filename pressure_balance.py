# ============================================================
# pressure_balance.py
# 2026-08-25: 정연(B)의 실제 실행 로그(터미널)로 확인함.
# f_net_n은 상태와 무관하게 항상 계산됨(IDLE도 f_net=0.00N으로
# 찍힘). window_width_m 역산 결과 정연 코드는 아직 0.152 사용중
# 인 게 확인됨. 병합 코드는 현재 잠정 패널 폭 0.4를 사용한다.
#
# F_net ≤ 57.7N 순간 개방 로직 및 57.7N 도출 근거(모터 정격토크
# 1.8kgf·cm 기준 역산)는 FLOODGUARD_실험흐름.docx로 재확인함 - 일치.
#
# 2026-08-25 추가 수정 1 (정연의 "개방 판단" 로직 문서/스크린샷 기준):
# 1) "안쪽 수위가 바깥쪽보다 높게 들어오는 이상 상황에서는 음수 힘이
#    나오지 않도록 처리" - h_out_m**2 - h_in_m**2가 음수가 되면 0으로
#    클램프하도록 compute_f_net_n()을 수정함.
#
#    WINDOW_WIDTH_M=0.4m와 PRESSURE_THRESHOLD_N=57.7N은 현재 병합 코드의
#    잠정값이며, 최종 실험 기준으로는 아직 확정하지 않았다.
# 2) "알 수 없는 상태: 열지 않음"이 개방 판단표에 명시되어 있어서,
#    should_open_window()에 IDLE/LOW/MID/HIGH/ESCAPE가 아닌 상태가
#    들어오면 명시적으로 False를 반환하도록 unknown-state 분기를
#    추가함.
#
# 2026-08-30 최종 통합 기준: ESCAPE가 fsm_controller의 정식 FSM
# 상태로 유지되며, should_open_window()도 ESCAPE를 명시적으로
# 처리한다. ESCAPE는 창문이 이미 열린 뒤의 상태이므로 can_open=False를
# 반환한다. can_open은 "지금 릴레이를 작동할지"를 뜻하므로, ESCAPE에서
# 릴레이를 다시 작동시키지 않는다.
# ============================================================

RHO_WATER = 1000.0
G = 9.8

# !! TODO(잠정치 - 포맥스 판 실물 입수 후 교체 필요) !!
# 아래 두 값은 확정 물리상수가 아니다. 정연 본인이 "메인" 페이지의
# "남은 실제 연결 작업" 체크리스트에 "수조 실험 후 LOW/MID/HIGH
# 기준값 및 57.7N 임계값 보정"이라고 직접 명시해뒀고, 서연도 정연
# 페이지에 "포맥스 판 실물 보고 계산값 변경 가능"이라고 코멘트를
# 남긴 항목이다 (2026-08-25). 포맥스 판 실물을 받으면 폭을 다시
# 실측하고, 임계값도 그 실측값 + 수조 실험 데이터로 재계산해야
# 한다. 코드에서 이 값들을 바꿔야 할 때는 이 TODO 블록만 찾으면 됨.
WINDOW_WIDTH_M = 0.4         # TODO(잠정치): 포맥스 판 실측 후 교체
PRESSURE_THRESHOLD_N = 57.7  # TODO(잠정치): 포맥스 판 실측 + 수조실험 후 재계산 필요


def compute_f_net_n(h_out_cm, h_in_cm):
    """F_net = 1/2 * rho * g * width * (h_out^2 - h_in^2), 단위: N.
    안쪽 수위가 바깥쪽보다 높은 이상 상황에서는 0으로 클램프한다."""
    h_out_m = (h_out_cm or 0.0) / 100.0
    h_in_m = (h_in_cm or 0.0) / 100.0
    diff = h_out_m ** 2 - h_in_m ** 2
    if diff < 0:
        diff = 0.0
    return 0.5 * RHO_WATER * G * WINDOW_WIDTH_M * diff


def should_open_window(state, h_out_cm, h_in_cm):
    f_net_n = compute_f_net_n(h_out_cm, h_in_cm)

    if state == "IDLE":
        return False, f_net_n, "idle_no_open"

    if state == "LOW":
        return True, f_net_n, "low_immediate_open"

    if state == "ESCAPE":
        # 창문이 이미 열린 뒤의 상태. can_open은 릴레이 재작동을 막기 위해 False.
        return False, f_net_n, "escape_already_open"

    if state not in ("MID", "HIGH"):
        # "알 수 없는 상태: 열지 않음" (정연의 개방판단 문서 기준,
        # 2026-08-25 추가) - IDLE/LOW/MID/HIGH/ESCAPE가 아닌 상태가
        # 실수로 들어와도 안전하게 열지 않는다.
        return False, f_net_n, "unknown_state_no_open"

    if h_in_cm is None:
        return False, f_net_n, "h_in_unavailable_wait"

    if f_net_n <= PRESSURE_THRESHOLD_N:
        return True, f_net_n, "pressure_balanced_open"

    return False, f_net_n, "pressure_too_high_wait"


can_open = should_open_window
