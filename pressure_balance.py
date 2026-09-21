# ============================================================
# pressure_balance.py
# 2026-08-25 historical note: 당시 정연(B)의 실제 실행 로그에서
# f_net_n 계산을 확인했으며, 그때의 37cm 사각판/폭 가정은 아래
# 2026-09-21 trapezoid remeasurement update로 대체됐다.
#
# 기존 F_net ≤ 57.7N은 μ=0.83 가정에 따른 계산 추정치였다.
# 2026-09-21 재측정: 패널 아래폭 33.3cm, 위폭 36.4cm,
# 높이 21.7cm, 두께 3mm다. 요청한 단순 근사로 평균 폭을 사용한다:
# (33.3+36.4)/2 = 34.85cm. 사다리꼴 폭 변화 적분은 사용하지 않는다.
# 판 밀도 0.55g/cm³, 드럼 유효 반지름 0.50cm, μ_s=0.60은 설계 가정이다.
# 평균 폭 기준 부피 약 226.87cm³, 추정 질량 약 0.1248kg,
# 정격토크 1.8kgf·cm를 0.50cm 반지름에 적용한 이상 구동력은 약 35.3N이다.
# (35.3 - 0.1248×9.8)/0.60 ≈ 56.8N으로 재계산해 임시 임계값에 적용했다.
# μ_k=0.30으로 구한 약 163.9N은 운동 시작 뒤 참고값일 뿐 개방 기준이 아니다.
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

# 판 치수 실측값. 수압 계산은 요청에 따라 평균 폭 근사를 쓴다.
PANEL_HEIGHT_M = 0.217
PANEL_BOTTOM_WIDTH_M = 0.333
PANEL_TOP_WIDTH_M = 0.364
WINDOW_WIDTH_M = (PANEL_BOTTOM_WIDTH_M + PANEL_TOP_WIDTH_M) / 2.0
PANEL_THICKNESS_M = 0.003
PANEL_DENSITY_KG_M3 = 550.0  # 0.55g/cm³ 가정, 실측 전
PANEL_AREA_M2 = PANEL_HEIGHT_M * (PANEL_BOTTOM_WIDTH_M + PANEL_TOP_WIDTH_M) / 2.0
PANEL_VOLUME_M3 = PANEL_AREA_M2 * PANEL_THICKNESS_M
PANEL_MASS_KG = PANEL_VOLUME_M3 * PANEL_DENSITY_KG_M3

# 설계 추정 입력값: μ_s=0.60, μ_k=0.30. 실제 접촉 조합·젖은 조건의 실측값은 아니다.
# 토크 1.8kgf·cm / 드럼 유효 반지름 0.50cm, 효율 100% 가정의 이상값.
MAX_LINEAR_DRIVE_FORCE_N = 35.3
STATIC_FRICTION_COEFF = 0.60   # 사용자 지정 설계 추정값; 실제 조합 실측 전
KINETIC_FRICTION_COEFF = 0.30  # PVC 접촉 실험값을 참고한 설계 추정값
PRESSURE_THRESHOLD_N = (
    56.8
)  # 사용자 지정 임시 임계값(N); 구조 및 실험 검증 필요


def compute_f_net_n(h_out_cm, h_in_cm):
    """평균 폭 근사로 계산한 순수압력(N); 수위 단위는 cm."""
    h_out_m = (h_out_cm or 0.0) / 100.0
    h_in_m = (h_in_cm or 0.0) / 100.0
    diff = 0.5 * RHO_WATER * G * WINDOW_WIDTH_M * (h_out_m ** 2 - h_in_m ** 2)
    if diff < 0:
        diff = 0.0
    return diff


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
