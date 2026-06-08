"""그리퍼 더블-핀치 제스처 검출.

'쥐었다 폄(close->open)' 1회 = 핀치. double_window 안에 핀치 2회 = 토글 이벤트(True).
hysteresis(enter_closed/enter_open)로 채터링 방지. 신호는 그리퍼 각도 또는 거리 등 단조 스칼라.
"""


def calibrate_open_closed(samples, rest=None):
    """캘리브 샘플(list of float)에서 (open_val, closed_val) 추정.

    open_val = rest(기본 첫 샘플)에 가까운 극단, closed_val = 반대 극단.
    """
    if not samples:
        raise ValueError("빈 캘리브 샘플")
    lo, hi = min(samples), max(samples)
    if rest is None:
        rest = samples[0]
    open_val = lo if abs(lo - rest) <= abs(hi - rest) else hi
    closed_val = hi if open_val == lo else lo
    return open_val, closed_val


class GripperGestureDetector:
    def __init__(self, open_val, closed_val, double_window=1.5,
                 enter_closed_frac=0.6, enter_open_frac=0.35, min_pinch_gap=0.08):
        self.open_val = float(open_val)
        self.closed_val = float(closed_val)
        d = self.closed_val - self.open_val
        if d == 0:
            d = 1e-6
        self.dir = 1.0 if d > 0 else -1.0          # 닫힘 방향 부호
        self.enter_closed = self.open_val + enter_closed_frac * d
        self.enter_open = self.open_val + enter_open_frac * d
        self.double_window = double_window
        self.min_pinch_gap = min_pinch_gap
        self.state = "open"
        self.pinch_times = []
        self._last_pinch_t = -1e9
        # ---- 진단 계측 ----
        self.last_event = None          # 이번 update()에서 발생한 이벤트
        self.n_close = self.n_open = 0  # close/open 전이 누적
        self.n_pinch = self.n_toggle = 0
        self.nan_count = 0              # None/NaN 신호 누적

    def update(self, val, t):
        """현재 그리퍼 신호 val(시각 t)로 상태 갱신. 더블-핀치 완성 시 True.

        진단용으로 self.last_event 에 이번 호출의 이벤트를 기록한다
        (None | 'close' | 'open' | 'pinch' | 'toggle').
        """
        self.last_event = None
        if val is None or val != val:        # None 또는 NaN → 무시(채터 유발 X)
            self.nan_count += 1
            return False
        toward_closed = (val - self.enter_closed) * self.dir >= 0
        toward_open = (val - self.enter_open) * self.dir <= 0
        if self.state == "open" and toward_closed:
            self.state = "closed"
            self.n_close += 1
            self.last_event = "close"
        elif self.state == "closed" and toward_open:
            self.state = "open"
            self.n_open += 1
            self.last_event = "open"
            if t - self._last_pinch_t >= self.min_pinch_gap:
                self._last_pinch_t = t
                self.pinch_times = [pt for pt in self.pinch_times if t - pt <= self.double_window]
                self.pinch_times.append(t)
                self.n_pinch += 1
                self.last_event = "pinch"
                if len(self.pinch_times) >= 2:
                    self.pinch_times = []
                    self.n_toggle += 1
                    self.last_event = "toggle"
                    return True
        return False

    @property
    def is_closed(self):
        return self.state == "closed"

    def pinch_progress(self, now):
        """현재 double_window 내 누적 핀치 수(0/1) — UI 피드백용."""
        return len([pt for pt in self.pinch_times if now - pt <= self.double_window])
