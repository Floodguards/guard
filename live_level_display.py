"""Terminal-only water-level display for supervised experiments."""


class LiveLevelDisplay:
    def __init__(self, jump_rate_cm_s=10.0):
        self.jump_rate_cm_s = jump_rate_cm_s
        self.previous_inside = None
        self.previous_time = None

    def format(self, data):
        outside = data.get("h_out_cm")
        inside = data.get("h_in_cm")
        outside_valid = data.get("outside_valid", False) and outside is not None
        inside_valid = data.get("inside_valid", False) and inside is not None
        outside_text = f"{outside:.2f}cm" if outside_valid else "측정 실패"
        inside_text = f"{inside:.2f}cm" if inside_valid else "측정 실패"
        warning = ""

        if inside_valid:
            now = data.get("loop_time")
            if (
                now is not None
                and self.previous_time is not None
                and now > self.previous_time
                and abs(inside - self.previous_inside) / (now - self.previous_time)
                >= self.jump_rate_cm_s
            ):
                warning = (
                    f" | 내부 수위 급변 {inside - self.previous_inside:+.2f}cm"
                    " (센서/수면 확인)"
                )
            self.previous_inside = inside
            self.previous_time = now
        else:
            self.previous_inside = None
            self.previous_time = None
            warning = " | 내부 센서 확인 필요"

        return f"h_out={outside_text} | h_in={inside_text}{warning}"
