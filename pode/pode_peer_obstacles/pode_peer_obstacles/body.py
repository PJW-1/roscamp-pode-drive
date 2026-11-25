"""다른 Pode의 몸 자리에 찍을 점을 계산한다(ROS 없음): 몸을 덮는 원, 몸 발자국, 선 몸에서 늘인 띠."""
import math

# 몸을 덮는 원 반지름 (m): 몸 가운데에서 가장 먼 꼭짓점 + 2 mm
RADIUS_M = 0.075
# 원 가운데: 바퀴 축(base_footprint)에서 몸 가운데까지 앞쪽 거리 (m)
CENTRE_M = -0.018
# 몸 발자국(base_footprint 틀): nav2_params.yaml local_costmap footprint와 같은 꼭짓점
FOOTPRINT = ((0.040, 0.021), (0.036, 0.031), (0.029, 0.056), (-0.029, 0.056), (-0.049, 0.049),
             (-0.071, 0.042), (-0.076, 0.037), (-0.076, -0.037), (-0.071, -0.042),
             (-0.049, -0.049), (-0.029, -0.056), (0.029, -0.056), (0.036, -0.031),
             (0.040, -0.021))
# 발자국 꼭짓점을 몸 가운데에서 밖으로 미는 여유 (m)
MARGIN_M = 0.002
FOOTPRINT_SHAPE, DISC_SHAPE = "footprint", "disc"
SHAPES = (DISC_SHAPE, FOOTPRINT_SHAPE)
# 띠가 보는 로봇에서 떨어져 끝나는 거리 (m): 보는 로봇 자리가 막힌 칸이 되지 않게 한다
LANE_KEEP_M = 0.20
# 보는 로봇이 선 로봇의 몸 방향 축에서 옆으로 이보다 멀면 띠를 내지 않는다 (m)
LANE_SIDE_M = 0.30


def others(names: str, me: str) -> list[str]:
    """"pode1,pode2,pode3"에서 나를 뺀 이름들."""
    found = []
    for name in (part.strip() for part in names.split(",")):
        if name and name != me and name not in found:
            found.append(name)
    return found


def body_points(x: float, y: float, yaw: float, radius_m: float, step_m: float,
                centre_m: float) -> list[tuple[float, float]]:
    """자세 (x, y, yaw)의 몸을 덮는 원 안의 점들(map 좌표).

    둘레는 격자만으로 칸이 빠질 수 있어 step_m보다 촘촘히 더 찍는다.
    """
    cx, cy = x + centre_m * math.cos(yaw), y + centre_m * math.sin(yaw)
    n = int(radius_m / step_m)
    points = [(cx + i * step_m, cy + j * step_m)
              for i in range(-n, n + 1) for j in range(-n, n + 1)
              if math.hypot(i * step_m, j * step_m) <= radius_m]
    rim = max(8, math.ceil(2.0 * math.pi * radius_m / step_m))
    points += [(cx + radius_m * math.cos(2.0 * math.pi * k / rim),
                cy + radius_m * math.sin(2.0 * math.pi * k / rim)) for k in range(rim)]
    return points


def _inside(q: tuple[float, float], poly) -> bool:
    hit = False
    for (x0, y0), (x1, y1) in zip(poly, [*poly[1:], poly[0]], strict=True):
        if (y0 > q[1]) != (y1 > q[1]) and q[0] < x0 + (q[1] - y0) * (x1 - x0) / (y1 - y0):
            hit = not hit
    return hit


def footprint_points(x: float, y: float, yaw: float, step_m: float, margin_m: float = MARGIN_M,
                     footprint=FOOTPRINT, centre_m: float = CENTRE_M) -> list[tuple[float, float]]:
    """자세 (x, y, yaw)의 몸 발자국 안과 둘레의 점들(map 좌표).

    둘레는 꼭짓점을 몸 가운데에서 margin_m만큼 밖으로 민 다각형이다. 안은 step_m의 반 간격 격자,
    둘레는 그 간격 이하로 찍어 빈 데가 없다.
    """
    grown = []
    for vx, vy in footprint:
        reach = math.hypot(vx - centre_m, vy)
        grown.append((vx + margin_m * (vx - centre_m) / reach, vy + margin_m * vy / reach))
    fine = step_m / 2.0
    xs, ys = [v[0] for v in footprint], [v[1] for v in footprint]
    local = [(i * fine, j * fine)
             for i in range(math.floor(min(xs) / fine), math.ceil(max(xs) / fine) + 1)
             for j in range(math.floor(min(ys) / fine), math.ceil(max(ys) / fine) + 1)
             if _inside((i * fine, j * fine), footprint)]
    for (x0, y0), (x1, y1) in zip(grown, [*grown[1:], grown[0]], strict=True):
        n = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / fine))
        local += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in local]


def lane_span(pose, viewer, length_m: float, keep_m: float = LANE_KEEP_M,
              side_m: float = LANE_SIDE_M, footprint=FOOTPRINT) -> tuple[float, float] | None:
    """선 로봇 pose의 몸에서 보는 로봇 viewer 쪽으로 늘인 띠의 범위(몸 방향 축 좌표, 작은 쪽과 큰 쪽).

    띠는 몸 끝에서 length_m까지이고 보는 로봇에서 keep_m 앞에서 끝난다. 낼 것이 없으면 None.
    """
    x, y, yaw = pose
    dx, dy = viewer[0] - x, viewer[1] - y
    along = dx * math.cos(yaw) + dy * math.sin(yaw)
    aside = -dx * math.sin(yaw) + dy * math.cos(yaw)
    if length_m <= 0.0 or abs(aside) > side_m:
        return None
    front, rear = max(v[0] for v in footprint), min(v[0] for v in footprint)
    if along >= front:
        lo, hi = front, min(front + length_m, along - keep_m)
    elif along <= rear:
        lo, hi = max(rear - length_m, along + keep_m), rear
    else:
        return None
    return (lo, hi) if hi - lo > 1e-9 else None


def lane_points(pose, viewer, length_m: float, half_m: float, step_m: float,
                keep_m: float = LANE_KEEP_M, side_m: float = LANE_SIDE_M
                ) -> list[tuple[float, float]]:
    """선 로봇의 몸 폭(half_m) 그대로 보는 로봇 쪽으로 늘인 띠 안의 점들(map 좌표).

    계획(global costmap)이 선 로봇 옆줄로 일찍 옮겨, 몸 옆을 지날 때는 통로와 나란히 가게 한다.
    """
    span = lane_span(pose, viewer, length_m, keep_m, side_m)
    if span is None:
        return []
    x, y, yaw = pose
    fine = step_m / 2.0
    n, m = max(1, math.ceil((span[1] - span[0]) / fine)), max(1, math.ceil(2.0 * half_m / fine))
    c, s = math.cos(yaw), math.sin(yaw)
    points = []
    for i in range(n + 1):
        px = span[0] + (span[1] - span[0]) * i / n
        for j in range(m + 1):
            py = -half_m + 2.0 * half_m * j / m
            points.append((x + c * px - s * py, y + s * px + c * py))
    return points
