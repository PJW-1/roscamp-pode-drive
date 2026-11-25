"""다른 로봇의 몸을 costmap에 찍을 점으로 바꾸는 계산(ROS 없이 돈다)."""
import math

import pytest
from pathlib import Path

import yaml
from pode_peer_obstacles.body import (
    CENTRE_M,
    FOOTPRINT,
    LANE_KEEP_M,
    MARGIN_M,
    RADIUS_M,
    body_points,
    footprint_points,
    lane_points,
    lane_span,
    others,
)


def test_the_other_robots_are_every_name_but_mine():
    assert others("pode1, pode2,pode3", "pode2") == ["pode1", "pode3"]


def test_an_empty_list_has_no_other_robots():
    assert others("", "pode1") == []


def test_a_name_listed_twice_is_one_robot():
    assert others("pode1,pode3,pode3", "pode1") == ["pode3"]


def test_the_points_fill_a_disc_around_the_body_centre():
    points = body_points(1.0, 0.5, 0.0, radius_m=0.08, step_m=0.02, centre_m=0.0)
    assert all(math.dist(p, (1.0, 0.5)) <= 0.08 + 1e-9 for p in points)
    # 원 안의 어느 자리든 점에서 한 칸(대각선 반) 안이다: 빈 데가 없다
    for angle in range(0, 360, 15):
        for r in (0.0, 0.03, 0.06):
            spot = (1.0 + r * math.cos(math.radians(angle)),
                    0.5 + r * math.sin(math.radians(angle)))
            assert min(math.dist(spot, p) for p in points) <= 0.02 * math.sqrt(0.5) + 1e-9


def test_the_rim_of_the_disc_is_marked_too():
    points = body_points(0.0, 0.0, 0.0, radius_m=0.08, step_m=0.02, centre_m=0.0)
    for angle in range(0, 360, 10):
        rim = (0.08 * math.cos(math.radians(angle)), 0.08 * math.sin(math.radians(angle)))
        assert min(math.dist(rim, p) for p in points) <= 0.02


@pytest.mark.parametrize("yaw, centre", [(0.0, (0.982, 0.5)), (math.pi / 2, (1.0, 0.482)),
                                         (math.pi, (1.018, 0.5))])
def test_the_disc_sits_on_the_body_centre_behind_the_wheel_axis(yaw, centre):
    """base_footprint는 바퀴 축에 있고 몸은 그 뒤로 더 길다: 원의 가운데는 뒤로 0.018 m다."""
    points = body_points(1.0, 0.5, yaw, radius_m=0.08, step_m=0.02, centre_m=-0.018)
    mean = (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))
    assert math.dist(mean, centre) < 1e-6
    assert all(math.dist(p, centre) <= 0.08 + 1e-9 for p in points)


def test_the_default_disc_covers_the_body_with_a_small_margin():
    """몸 가운데에서 가장 먼 꼭짓점(0.073 m)을 덮고 여유는 2 mm만 둔다(0.08은 좁은 틈을 막는다)."""
    farthest = max(math.dist(p, (CENTRE_M, 0.0)) for p in FOOTPRINT)
    assert RADIUS_M == 0.075
    assert farthest < RADIUS_M <= farthest + 0.003


# ---- 발자국 모양으로 찍기 ----
def _inside(q, poly) -> bool:
    hit = False
    for (x0, y0), (x1, y1) in zip(poly, [*poly[1:], poly[0]], strict=True):
        if (y0 > q[1]) != (y1 > q[1]) and q[0] < x0 + (q[1] - y0) * (x1 - x0) / (y1 - y0):
            hit = not hit
    return hit


def _edge_gap(q, poly) -> float:
    best = math.inf
    for (x0, y0), (x1, y1) in zip(poly, [*poly[1:], poly[0]], strict=True):
        dx, dy = x1 - x0, y1 - y0
        t = max(0.0, min(1.0, ((q[0] - x0) * dx + (q[1] - y0) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(q[0] - x0 - t * dx, q[1] - y0 - t * dy))
    return best


def test_the_footprint_is_the_polygon_of_the_local_costmap():
    """nav2_params.yaml local_costmap의 다각형 발자국과 같은 꼭짓점이다."""
    path = Path(__file__).resolve().parents[2] / "pode_nav_config" / "params" / "nav2_params.yaml"
    text = path.read_text(encoding="utf-8")
    rows = [line.split("footprint:", 1)[1] for line in text.splitlines()
            if line.strip().startswith("footprint:")]
    polygons = [yaml.safe_load(yaml.safe_load(row)) for row in rows]
    assert [list(v) for v in FOOTPRINT] in polygons


def test_the_footprint_points_stay_on_the_body_with_a_small_margin():
    """찍는 점은 모두 발자국 안이거나 둘레에서 여유(2 mm) 안이다: 원(0.075 m)처럼 옆으로 더 막지 않는다."""
    points = footprint_points(0.0, 0.0, 0.0, step_m=0.02)
    assert MARGIN_M == 0.002
    assert all(_inside(q, FOOTPRINT) or _edge_gap(q, FOOTPRINT) <= MARGIN_M + 1e-9 for q in points)
    assert max(abs(y) for _, y in points) == pytest.approx(0.056 + MARGIN_M, abs=1e-3)
    assert max(x for x, _ in points) == pytest.approx(0.040 + MARGIN_M, abs=1e-3)
    assert min(x for x, _ in points) == pytest.approx(-0.076 - MARGIN_M, abs=1e-3)


def test_the_footprint_points_leave_no_hole_on_the_body():
    """발자국 안의 어느 자리든 점에서 한 칸(대각선 반) 안이고, 둘레의 어느 자리든 점에서 반 칸 안이다."""
    step = 0.02
    points = footprint_points(0.0, 0.0, 0.0, step_m=step)
    for i in range(-40, 21):
        for j in range(-30, 31):
            q = (i * 0.002, j * 0.002)
            if _inside(q, FOOTPRINT):
                assert min(math.dist(q, p) for p in points) <= step * math.sqrt(0.5) + 1e-9, q
    for (x0, y0), (x1, y1) in zip(FOOTPRINT, [*FOOTPRINT[1:], FOOTPRINT[0]], strict=True):
        for k in range(11):
            q = (x0 + (x1 - x0) * k / 10, y0 + (y1 - y0) * k / 10)
            assert min(math.dist(q, p) for p in points) <= step / 2 + MARGIN_M + 1e-9, q


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi, -2.2])
def test_the_footprint_points_turn_and_move_with_the_pose(yaw):
    here = footprint_points(1.0, 0.5, yaw, step_m=0.02)
    base = footprint_points(0.0, 0.0, 0.0, step_m=0.02)
    c, s = math.cos(yaw), math.sin(yaw)
    moved = [(1.0 + c * x - s * y, 0.5 + s * x + c * y) for x, y in base]
    assert len(here) == len(moved)
    assert all(math.dist(a, b) < 1e-9 for a, b in zip(here, moved, strict=True))


def test_a_body_standing_along_a_shelf_is_marked_no_wider_than_it_is():
    """진열대에서 3 cm 띄워 나란히 선 몸(중심 y 0.2615)은 y 0.2035 아래로는 찍히지 않는다(원은 0.1865까지)."""
    points = footprint_points(1.40, 0.2615, 0.0, step_m=0.02)
    assert min(y for _, y in points) >= 0.2615 - 0.056 - MARGIN_M - 1e-9
    disc = body_points(1.40, 0.2615, 0.0, RADIUS_M, 0.02, CENTRE_M)
    assert min(y for _, y in disc) == pytest.approx(0.1865, abs=1e-3)


# ---- 선 몸에서 보는 로봇 쪽으로 늘인 띠 ----
POSE = (1.40, 0.2615, 0.0)


def test_the_lane_runs_from_the_rear_toward_a_viewer_behind():
    assert lane_span(POSE, (0.64, 0.24), 0.30) == pytest.approx((-0.376, -0.076))


def test_the_lane_runs_from_the_front_toward_a_viewer_ahead():
    assert lane_span(POSE, (1.86, 0.15), 0.30) == pytest.approx((0.040, 0.26))  # 0.46 - 0.20에서 끝난다


def test_the_lane_ends_before_a_near_viewer_and_is_gone_beside_the_body():
    assert lane_span(POSE, (1.05, 0.12), 0.30) == pytest.approx((-0.35 + LANE_KEEP_M, -0.076))
    assert lane_span(POSE, (1.15, 0.12), 0.30) is None  # 띠가 들어갈 자리가 없다
    assert lane_span(POSE, (1.40, 0.11), 0.30) is None  # 몸 옆


def test_no_lane_for_a_viewer_far_to_the_side_or_with_no_length():
    assert lane_span(POSE, (0.64, 0.60), 0.30) is None
    assert lane_span(POSE, (0.64, 0.24), 0.0) is None


def test_the_lane_follows_the_heading_of_the_standing_body():
    """서쪽을 보고 선 몸: 서쪽에서 오는 로봇은 몸 앞쪽에 있다."""
    assert lane_span((1.40, 0.2615, math.pi), (0.64, 0.24), 0.30) == pytest.approx((0.040, 0.34))


def test_the_lane_points_fill_a_strip_as_wide_as_the_body():
    points = lane_points(POSE, (0.64, 0.24), 0.30, half_m=0.058, step_m=0.02)
    assert min(x for x, _ in points) == pytest.approx(1.40 - 0.376)
    assert max(x for x, _ in points) == pytest.approx(1.40 - 0.076)
    assert min(y for _, y in points) == pytest.approx(0.2615 - 0.058)
    assert max(y for _, y in points) == pytest.approx(0.2615 + 0.058)
    # 띠 안의 어느 자리든 점에서 반 칸(대각선) 안이다
    for i in range(31):
        for j in range(12):
            q = (1.024 + 0.01 * i, 0.2035 + 0.01 * j)
            assert min(math.dist(q, p) for p in points) <= 0.01 * math.sqrt(0.5) + 1e-9
    assert lane_points(POSE, (1.40, 0.11), 0.30, half_m=0.058, step_m=0.02) == []
