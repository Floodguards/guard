# ============================================================
# pressure_balance.py
# 2026-08-25 historical note: 당시 정연(B)의 실제 실행 로그에서
# f_net_n 계산을 확인했으며, 그때의 37cm 사각판/폭 가정은 아래
# 2026-09-21 trapezoid remeasurement update로 대체됐다.
#
# 기존 F_net ≤ 57.7N은 μ=0.83 가정에 따른 계산 추정치였다.
# 2026-09-25 문서 확정 입력값: 수압 작용 폭 0.19m, 드럼 유효 반지름
# 0.80cm, 포맥스판과 철사를 합친 질량 0.120kg, 정지마찰계수 μ_s=0.80.
# 정격토크 1.8kgf·cm를 0.80cm 반지름에 적용한 이상 구동력은 약 22.05N이고,
# (22.05 - 0.120×9.8)/0.80 ≈ 26.1N을 조건부 개방 임계값으로 사용한다.
#
# 2026-08-25 historical model (superseded 2026-09-21): rectangular width
# 0.37m and PRESSURE_THRESHOLD_N=108.2N. Current dimensions/model are the
# average-width approximation and pressure expression below.
# Negative net pressure remains clamped to zero, per the original policy.
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

# 판 치수 실측값. 수압 작용 폭은 문서 확정값 19cm로 고정한다.
PANEL_HEIGHT_M = 0.217
PANEL_BOTTOM_WIDTH_M = 0.333
PANEL_TOP_WIDTH_M = 0.364
WINDOW_WIDTH_M = 0.190  # 문서 확정 수압 작용 폭 19cm
PANEL_THICKNESS_M = 0.003
PANEL_DENSITY_KG_M3 = 550.0  # 0.55g/cm³ 가정, 실측 전
PANEL_AREA_M2 = PANEL_HEIGHT_M * (PANEL_BOTTOM_WIDTH_M + PANEL_TOP_WIDTH_M) / 2.0
PANEL_VOLUME_M3 = PANEL_AREA_M2 * PANEL_THICKNESS_M
PANEL_MASS_KG = 0.120  # 포맥스판과 고정철사를 합친 실측 질량

# 문서 확정 입력값: μ_s=0.80, 드럼 유효 반지름 0.80cm.
MAX_LINEAR_DRIVE_FORCE_N = 22.05
STATIC_FRICTION_COEFF = 0.80
KINETIC_FRICTION_COEFF = None  # 문서에서 운동마찰계수는 정의하지 않음
PRESSURE_THRESHOLD_N = 26.1  # 조건부 개방 임계값(N)


def compute_f_net_n(h_out_cm, h_in_cm):
    """평균 폭 근사로 계산한 순수압력(N); 수위 단위는 cm.

    외부 수위가 내부 수위보다 낮은 역수압 조건은 실험 판정값으로
    취급하지 않고 None을 반환한다.
    """
    h_out_m = (h_out_cm or 0.0) / 100.0
    h_in_m = (h_in_cm or 0.0) / 100.0
    diff = 0.5 * RHO_WATER * G * WINDOW_WIDTH_M * (h_out_m ** 2 - h_in_m ** 2)
    if diff < 0:
        return None
    return diff


def should_open_window(state, h_out_cm, h_in_cm):
    if state == "ESCAPE":
        # 창문이 이미 열린 뒤의 상태. can_open은 릴레이 재작동을 막기 위해 False.
        f_net_n = (
            compute_f_net_n(h_out_cm, h_in_cm)
            if h_out_cm is not None and h_in_cm is not None
            else None
        )
        return False, f_net_n, "escape_already_open"

    if h_out_cm is None:
        return False, None, "h_out_unavailable_wait"

    if state in ("MID", "HIGH") and h_in_cm is None:
        return False, None, "h_in_unavailable_wait"

    f_net_n = compute_f_net_n(h_out_cm, h_in_cm)
    if f_net_n is None:
        return False, None, "reverse_pressure_wait"

    if state == "IDLE":
        return False, f_net_n, "idle_no_open"

    if state == "LOW":
        return True, f_net_n, "low_immediate_open"

    if state not in ("MID", "HIGH"):
        # "알 수 없는 상태: 열지 않음" (정연의 개방판단 문서 기준,
        # 2026-08-25 추가) - IDLE/LOW/MID/HIGH/ESCAPE가 아닌 상태가
        # 실수로 들어와도 안전하게 열지 않는다.
        return False, f_net_n, "unknown_state_no_open"

    if f_net_n <= PRESSURE_THRESHOLD_N:
        return True, f_net_n, "pressure_balanced_open"

    return False, f_net_n, "pressure_too_high_wait"


can_open = should_open_window
