"""좁은 통로에서 청소 로봇이 한쪽으로 붙어 설 자리와 그리로 가는 길(통로 교차).

운반 로봇의 Nav2가 선 청소 로봇 옆으로 길을 짜려면 global costmap에 지나갈 칸이 남아야 한다.
남는 칸은 통로를 가로지르는 한 줄로 센다. NavFn은 길을 칸 중심보다 반 칸 원점 쪽에 그린다. 그래서
청소 로봇은 좌표가 큰 쪽 면에 붙는다: 작은 쪽에 붙으면 길이 청소 로봇 쪽으로 반 칸 다가와 충돌 판정에 선다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

Point = tuple[float, float]

# ---- 운반 로봇 Nav2 global costmap을 본뜬 값 ----
NAV_GRID_M = 0.025  # 칸 크기
NAV_ORIGIN_M = -0.15  # 격자 원점(두 축 같음)
NAV_INSCRIBED_M = 0.07  # 막힌 칸 중심에서 이 안의 칸 중심은 계획이 지나지 않는다
BODY_HALF_M = 0.056  # 몸 반폭
MARK_HALF_M = 0.075  # 선 몸이 costmap에 찍히는 반폭(pode_peer_obstacles가 찍는 원 반지름)
# ---- 선 몸에서 보는 로봇 쪽으로 늘여 찍히는 띠(pode_peer_obstacles body.lane_span과 같은 값) ----
BODY_FRONT_M, BODY_REAR_M = 0.040, -0.076  # 바퀴 축에서 몸 앞끝과 뒤끝
LANE_KEEP_M = 0.20  # 띠는 보는 로봇에서 이만큼 앞에서 끝난다
LANE_SIDE_M = 0.30  # 보는 로봇이 몸 방향 축에서 옆으로 이보다 멀면 띠가 없다

AISLE_MAX_M = 0.45  # 양쪽 면 사이가 이보다 넓으면 통로로 보지 않는다
AISLE_LOOK_M = 0.30  # 통로의 면은 통로 방향으로 이만큼 앞뒤에 걸친 것까지 본다(진열대 사이 틈도 같은 통로)
LEAD_M = 0.08  # 옆으로 붙으며 나아가는 거리의 하한
SLOPE = 1.5  # 나아가는 거리 / 옆으로 가는 거리
STRAIGHT_M = 0.05  # 붙은 뒤 통로와 나란히 서려고 더 가는 거리
STEP_M = 0.01  # 길을 검사하는 간격
_EPS = 1e-9


# ---- 붙어 설 자리 하나 ----
@dataclass(frozen=True)
class Tuck:
    spot: Point  # 설 자리(중심)
    yaw: float  # 설 방향(통로 방향)
    path: tuple  # 지금 자리에서 설 자리까지 꺾은선
    back: Point  # 떠난 줄로 돌아갈 점(같은 방향으로 더 간 곳)
    rows: int  # 옆에 남는 칸 수


def pass_rows(lo: float, hi: float, body_lo: float, body_hi: float, grid: float = NAV_GRID_M,
              origin: float = NAV_ORIGIN_M, inscribed: float = NAV_INSCRIBED_M) -> int:
    """막힌 면 lo, hi 사이를 가로지르는 한 줄에서 body_lo~body_hi에 몸이 찍혔을 때 계획이 지날 칸 수.

    막힌 것이 걸친 칸은 막힌 칸이다. 막힌 칸 중심에서 inscribed 안의 칸도 지나지 못한다.
    """
    below = math.ceil((lo - origin) / grid - _EPS) - 1  # 아래 면이 막는 마지막 칸
    above = math.floor((hi - origin) / grid + _EPS)  # 위 면이 막는 첫 칸
    first = math.floor((body_lo - origin) / grid + _EPS)
    last = math.ceil((body_hi - origin) / grid - _EPS) - 1
    shut = [below, above, *range(first, last + 1)]
    return sum(1 for k in range(below + 1, above)
               if all(abs(k - s) * grid > inscribed + _EPS for s in shut))


def faces(solids, box, p: Point, axis: int, look: float = 0.0) -> tuple[float, float]:
    """p에서 통로 방향 axis(0: x, 1: y)에 수직으로 본 양쪽 막힌 면의 좌표(작은 쪽, 큰 쪽).
    통로 방향으로 look 안에 걸친 것까지 본다."""
    along, across = p[axis], p[1 - axis]
    lo, hi = box[1 - axis], box[3 - axis]
    for rect in solids:
        if rect[axis + 2] < along - look - _EPS or rect[axis] > along + look + _EPS:
            continue
        r_lo, r_hi = rect[1 - axis], rect[3 - axis]
        if r_hi <= across + _EPS:
            lo = max(lo, r_hi)
        elif r_lo >= across - _EPS:
            hi = min(hi, r_lo)
        else:
            return across, across
    return lo, hi


def _gap(p: Point, rect) -> float:
    return math.hypot(max(rect[0] - p[0], 0.0, p[0] - rect[2]),
                      max(rect[1] - p[1], 0.0, p[1] - rect[3]))


def clear(solids, box, pts, radius: float) -> bool:
    """꺾은선 위 어디서든 몸을 덮는 원(radius)이 벽과 막힌 것에 닿지 않는가."""
    for a, b in zip(pts, pts[1:], strict=False):
        n = max(1, math.ceil(math.dist(a, b) / STEP_M))
        for k in range(n + 1):
            q = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            wall = min(q[0] - box[0], box[2] - q[0], q[1] - box[1], box[3] - q[1])
            if wall < radius - 1e-6 or any(_gap(q, rect) < radius - 1e-6 for rect in solids):
                return False
    return True


def plan(solids, box, here: Point, yaw: float, axis: int, *, clear: float, radius: float,
         rows: int, max_width: float = AISLE_MAX_M) -> Tuck | None:
    """here에서 통로(방향 axis)의 좌표가 큰 쪽 면에 clear만큼 띄워 나란히 설 자리. 없으면 None.

    옆에 남는 칸이 rows보다 적으면 없다. 몸 방향 쪽으로 나아가며 붙는다.
    """
    lo, hi = faces(solids, box, here, axis, AISLE_LOOK_M)
    if not _EPS < hi - lo <= max_width + _EPS:
        return None
    across = here[1 - axis]
    forward = math.cos(yaw) if axis == 0 else math.sin(yaw)
    f = 1.0 if forward >= 0.0 else -1.0
    line = hi - clear - BODY_HALF_M
    left = pass_rows(lo, hi, line - MARK_HALF_M, line + MARK_HALF_M)
    if left < rows:
        return None

    def point(s: float, c: float) -> Point:
        return (s, c) if axis == 0 else (c, s)

    shift = abs(line - across)
    lead = max(LEAD_M, SLOPE * shift) if shift > STEP_M / 2 else 0.0
    s1 = here[axis] + f * lead
    s2 = s1 + f * STRAIGHT_M
    pts = [here, point(s1, line), point(s2, line)] if lead else [here, point(s2, line)]
    if not _clear(solids, box, pts, radius):
        return None
    heading = (0.0 if f > 0 else math.pi) if axis == 0 else f * math.pi / 2
    return Tuck(pts[-1], heading, tuple(pts), point(s2 + f * lead, across), left)


def lane(spot: Point, yaw: float, viewer: Point, length: float) -> tuple[Point, Point] | None:
    """spot에 yaw로 선 몸에서 viewer 쪽으로 늘여 찍히는 띠의 중심선(두 끝). 없으면 None."""
    dx, dy = viewer[0] - spot[0], viewer[1] - spot[1]
    c, s = math.cos(yaw), math.sin(yaw)
    along, aside = dx * c + dy * s, -dx * s + dy * c
    if length <= 0.0 or abs(aside) > LANE_SIDE_M:
        return None
    if along >= BODY_FRONT_M:
        lo, hi = BODY_FRONT_M, min(BODY_FRONT_M + length, along - LANE_KEEP_M)
    elif along <= BODY_REAR_M:
        lo, hi = max(BODY_REAR_M - length, along + LANE_KEEP_M), BODY_REAR_M
    else:
        return None
    if hi - lo <= _EPS:
        return None
    return (spot[0] + c * lo, spot[1] + s * lo), (spot[0] + c * hi, spot[1] + s * hi)


def lane_shuts(spot: Point, yaw: float, viewer: Point, length: float, point: Point) -> bool:
    """그 띠 때문에 viewer의 계획이 point에 닿지 못하는가(띠가 찍힌 칸에서 NAV_INSCRIBED_M 안)."""
    ends = lane(spot, yaw, viewer, length)
    if ends is None:
        return False
    (ax, ay), (bx, by) = ends
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((point[0] - ax) * dx + (point[1] - ay) * dy) / (dx * dx + dy * dy)))
    gap = math.hypot(point[0] - ax - t * dx, point[1] - ay - t * dy)
    return gap <= BODY_HALF_M + NAV_GRID_M + NAV_INSCRIBED_M + _EPS


_clear = clear
