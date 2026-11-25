"""좁은 통로에서 한쪽으로 붙어 설 자리 계산(통로 교차)."""
import math

import pytest
from fleet_core import tuck
from fleet_core.fleet_map import load_fleet_map

SITE = load_fleet_map("v7")
BOX = SITE.bounds()
SOLIDS = tuple(dict.fromkeys(tuple(r) for r in (*SITE.shelves, *SITE.sim_shelves)))
SOUTH = (0.0, 0.3475)  # 남쪽 통로: 벽과 진열대 남쪽 면
MIDDLE = (0.3875, 0.735)  # 가운데 통로: 남쪽 진열대 북쪽 면과 진열대 A 남쪽 면


def _flat(points) -> list:
    return [v for q in points for v in q]


def _plan(here, yaw, axis=0, **over):
    kw = {"clear": 0.03, "radius": 0.083, "rows": 3, **over}
    return tuck.plan(SOLIDS, BOX, here, yaw, axis, **kw)


# ---- 지나갈 줄 수 ----
@pytest.mark.parametrize("centre, rows", [
    (0.2615, 3),   # 진열대 쪽에 붙음: Gazebo에서 지나갔다
    (0.086, 2),    # 벽 쪽에 붙음: Gazebo에서 충돌 판정으로 섰다
])
def test_the_rows_left_beside_a_body_in_the_south_aisle(centre, rows):
    half = tuck.MARK_HALF_M
    assert tuck.pass_rows(*SOUTH, centre - half, centre + half) == rows


def test_the_upper_side_of_the_middle_aisle_leaves_two_rows():
    half = tuck.MARK_HALF_M
    assert tuck.pass_rows(*MIDDLE, 0.649 - half, 0.649 + half) == 2


def test_a_body_in_the_middle_of_the_aisle_leaves_no_row():
    assert tuck.pass_rows(*SOUTH, 0.174 - 0.075, 0.174 + 0.075) == 0


def test_an_empty_aisle_has_rows_from_three_cells_off_each_face():
    """벽(칸 경계 위)에서는 셋째 칸부터, 진열대 면(칸 안)에서는 그 칸에서 셋째 칸부터다: 0.0625~0.2625."""
    assert tuck.pass_rows(*SOUTH, 1.0, 1.0) == 9


# ---- 통로의 양쪽 면 ----
@pytest.mark.parametrize("p, faces", [
    ((1.40, 0.20), SOUTH), ((0.85, 0.10), SOUTH), ((0.85, 0.55), MIDDLE),
    ((1.12, 0.20), (0.0, 0.735)),  # 진열대 틈 아래: 위쪽 면은 진열대 A
    ((0.30, 0.50), (0.0, 0.98)),
])
def test_the_faces_across_an_aisle_along_x(p, faces):
    assert tuck.faces(SOLIDS, BOX, p, 0) == pytest.approx(faces)


def test_the_faces_across_a_way_along_y():
    """남쪽 진열대 사이 틈(x 0.99~1.25)을 y 방향으로 지나는 길."""
    assert tuck.faces(SOLIDS, BOX, (1.12, 0.3675), 1) == pytest.approx((0.99, 1.25))


# ---- 붙어 설 자리와 가는 길 ----
def test_the_south_aisle_tucks_to_the_shelf_side():
    """좌표가 큰 쪽 면: 운반 로봇의 길은 반 칸 원점 쪽에 그려져 청소 로봇에서 멀어진다."""
    t = _plan((1.30, 0.2125), 0.0)
    assert t.rows == 3 and t.yaw == pytest.approx(0.0)
    assert _flat(t.path) == pytest.approx(_flat([(1.30, 0.2125), (1.38, 0.2615), (1.43, 0.2615)]))
    assert t.spot == pytest.approx((1.43, 0.2615))
    assert t.back == pytest.approx((1.51, 0.2125))


def test_the_far_row_still_goes_to_the_shelf_side_with_a_longer_lead():
    t = _plan((1.30, 0.1375), 0.0)
    lead = 1.5 * (0.2615 - 0.1375)
    assert t.path[1] == pytest.approx((1.30 + lead, 0.2615))
    assert t.spot == pytest.approx((1.30 + lead + 0.05, 0.2615))


def test_a_cleaner_heading_west_tucks_westward():
    t = _plan((1.50, 0.2125), math.pi)
    assert t.yaw == pytest.approx(math.pi)
    assert _flat(t.path) == pytest.approx(_flat([(1.50, 0.2125), (1.42, 0.2615), (1.37, 0.2615)]))


def test_the_middle_aisle_has_no_tuck_unless_two_rows_are_enough():
    assert _plan((1.27, 0.5125), 0.0) is None
    assert _plan((1.27, 0.5125), 0.0, rows=2).spot[1] == pytest.approx(0.649)
    assert _plan((1.27, 0.6125), 0.0, rows=2).spot[1] == pytest.approx(0.649)


def test_the_body_stays_clear_of_both_faces_all_the_way():
    t = _plan((1.30, 0.2125), 0.0)
    for (ax, ay), (bx, by) in zip(t.path, t.path[1:], strict=False):
        for k in range(11):
            q = (ax + (bx - ax) * k / 10, ay + (by - ay) * k / 10)
            assert 0.083 - 1e-9 <= q[1] <= 0.3475 - 0.083 + 1e-9


def test_no_tuck_in_a_wide_place():
    assert _plan((0.30, 0.50), 0.0) is None


@pytest.mark.parametrize("x", [0.93, 1.12, 1.20, 1.60])
def test_the_gap_between_the_shelves_is_the_same_aisle(x):
    """진열대 틈 아래와 진열대 끝 가까이도 진열대 줄에 맞춰 붙는다."""
    t = _plan((x, 0.2125), 0.0)
    assert t is not None and t.spot[1] == pytest.approx(0.2615) and t.rows == 3


def test_no_tuck_when_something_stands_on_the_way_to_the_spot():
    box = (1.36, 0.27, 1.42, 0.3475)
    assert tuck.plan((*SOLIDS, box), BOX, (1.30, 0.2125), 0.0, 0, clear=0.03, radius=0.083,
                     rows=3) is None


def test_no_tuck_when_too_few_rows_are_left():
    assert _plan((1.30, 0.2125), 0.0, rows=4) is None


def test_a_spot_already_on_the_tuck_line_only_goes_straight_on():
    t = _plan((1.30, 0.2615), 0.0)
    assert _flat(t.path) == pytest.approx(_flat([(1.30, 0.2615), (1.35, 0.2615)]))
    assert t.back == pytest.approx((1.35, 0.2615))


def test_the_grid_values_match_the_nav2_maps_and_parameters():
    """Nav2 지도 갈래 a(0.025 m, 원점 -0.15), global footprint 0.06 + padding 0.01, 찍는 몸 여유."""
    from pathlib import Path

    import yaml

    root = Path(__file__).resolve().parents[3]
    arena = yaml.safe_load((root / "sim/fleet_gz_sim/maps/arena_v7_a.yaml").read_text())
    assert arena["resolution"] == tuck.NAV_GRID_M and arena["origin"][:2] == [tuck.NAV_ORIGIN_M] * 2
    nav = (root / "pode/pode_nav_config/params/nav2_params.yaml").read_text(encoding="utf-8")
    assert 'footprint: "[[0.06, 0.06], [0.06, -0.06], [-0.06, -0.06], [-0.06, 0.06]]"' in nav
    assert "footprint_padding: 0.01" in nav
    assert tuck.NAV_INSCRIBED_M == pytest.approx(0.06 + 0.01)
    body = (root / "pode/pode_peer_obstacles/pode_peer_obstacles/body.py").read_text()
    assert f"RADIUS_M = {tuck.MARK_HALF_M}" in body
    assert "[0.029, 0.056]" in nav and tuck.BODY_HALF_M == 0.056


# ---- 선 몸에서 운반 로봇 쪽으로 찍히는 띠 ----
SPOT = (1.40, 0.2615)


def test_the_lane_runs_from_the_body_toward_the_viewer_and_stops_short_of_it():
    def ends(yaw, viewer):
        (ax, ay), (bx, by) = tuck.lane(SPOT, yaw, viewer, 0.30)
        return [ax, ay, bx, by]

    assert ends(0.0, (0.64, 0.24)) == pytest.approx([1.024, 0.2615, 1.324, 0.2615])
    assert ends(0.0, (1.86, 0.15)) == pytest.approx([1.44, 0.2615, 1.66, 0.2615])
    assert ends(0.0, (1.05, 0.12)) == pytest.approx([1.25, 0.2615, 1.324, 0.2615])
    assert ends(math.pi, (0.64, 0.24)) == pytest.approx([1.36, 0.2615, 1.06, 0.2615])


@pytest.mark.parametrize("viewer, length", [((1.40, 0.11), 0.30), ((1.15, 0.12), 0.30),
                                            ((0.64, 0.60), 0.30), ((0.64, 0.24), 0.0)])
def test_no_lane_beside_the_body_near_it_far_to_the_side_or_with_no_length(viewer, length):
    assert tuck.lane(SPOT, 0.0, viewer, length) is None


def test_the_lane_shuts_the_node_under_the_shelf_gap_but_not_the_far_end():
    """남쪽 통로 진열대 쪽(x 1.40)에 서면 서쪽에서 오는 로봇에게 틈 아래 지점 GAP_S가 띠에 덮인다."""
    viewer = (0.64, 0.24)
    assert tuck.lane_shuts(SPOT, 0.0, viewer, 0.30, (1.12, 0.22))
    assert not tuck.lane_shuts(SPOT, 0.0, viewer, 0.30, (1.66, 0.15))  # Q2는 몸 건너편
    assert not tuck.lane_shuts(SPOT, 0.0, viewer, 0.30, (1.12, 0.05))  # 옆으로 멀다
    assert not tuck.lane_shuts(SPOT, 0.0, viewer, 0.0, (1.12, 0.22))


def test_the_lane_values_match_the_node_that_marks_it():
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    body = (root / "pode/pode_peer_obstacles/pode_peer_obstacles/body.py").read_text()
    assert f"LANE_KEEP_M = {tuck.LANE_KEEP_M:.2f}" in body
    assert f"LANE_SIDE_M = {tuck.LANE_SIDE_M:.2f}" in body
    assert "FOOTPRINT = ((0.040, 0.021)" in body and "(-0.076, 0.037)" in body
    assert (tuck.BODY_FRONT_M, tuck.BODY_REAR_M) == (0.040, -0.076)
