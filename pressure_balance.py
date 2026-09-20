# ============================================================
# pressure_balance.py
# 2026-08-25: 정연(B)의 실제 실행 로그(터미널)로 확인함.
# f_net_n은 상태와 무관하게 항상 계산됨(IDLE도 f_net=0.00N으로
# 찍힘). window_width_m 역산 결과 정연 코드는 아직 0.152 사용중
# 인 게 확인됨. 포맥스판의 수압 작용 면 가로 길이는 37cm로 확정했다.
#
# 기존 F_net ≤ 57.7N은 μ=0.83 가정에 따른 계산 추정치였다.
# 2026-09-20 사용자가 제공한 참고 이미지에 따라 포맥스-PVC/철판 접촉의
# 설계 입력값을 μ_s=0.45, μ_k=0.40으로 채택했다. 판 질량은
# 37×21.7×0.4cm, 밀도 0.55g/cm³ 가정으로 약 0.1766kg 추정했다.
# (50.4N - 0.1766kg×9.8m/s²) / 0.45 ≈ 108.2N으로 재산정했다.
# 마찰계수는 이미지 참고값이며 제조사 공식값/젖은 실측값이 아니다.
# 실제 가이드 법선반력·모터 전달효율·판 질량을 확인 전 계산 추정치다.
#
# 2026-08-25 추가 수정 1 (정연의 "개방 판단" 로직 문서/스크린샷 기준):
# 1) "안쪽 수위가 바깥쪽보다 높게 들어오는 이상 상황에서는 음수 힘이
#    나오지 않도록 처리" - h_out_m**2 - h_in_m**2가 음수가 되면 0으로
#    클램프하도록 compute_f_net_n()을 수정함.
#
#    WINDOW_WIDTH_M=0.37m(포맥스판 수압 작용 면 가로 길이 37cm)는 확정값이다.
#    μ_s=0.45, μ_k=0.40은 제공 이미지의 포맥스-PVC/철판 참고값을 채택한
#    설계 입력값이다. 공식 제품별 마찰자료나 젖은 실측값은 아니므로 검증 전이다.
#    PRESSURE_THRESHOLD_N=108.2N은 판 질량 0.1766kg 추정 및 종전 구동력 가정으로
#    재산정한 잠정값이다.
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

# 포맥스판 수압 작용 면 가로 길이는 37cm로 확정되었다.
# 이미지 참고 설계 입력값: 포맥스(PVC)-철판 μ_s=0.45, μ_k=0.40.
# 실제 가이드가 철판인지, 젖은 조건에서 유효한지는 실물 확인/실험 전이다.
# 최대 선형 구동력 50.4N, 판 질량 0.1766kg(밀도 0.55g/cm³ 가정)을 적용하면
# (50.4 - 0.1766×9.8) / 0.45 ≈ 108.2N. 이는 F_net이 가이드 법선반력에
# 직접 대응한다는 가정의 계산 추정치이며, 수조/구동부 검증 전 잠정값이다.
WINDOW_WIDTH_M = 0.37
STATIC_FRICTION_COEFF = 0.45   # 제공 이미지의 포맥스-PVC/철판 참고값 (가정)
KINETIC_FRICTION_COEFF = 0.40  # 제공 이미지의 포맥스-PVC/철판 참고값 (가정)
PRESSURE_THRESHOLD_N = 108.2   # 계산 추정치: 실제 구조 및 실험 검증 필요


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
