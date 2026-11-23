"""배치 v7_bidir(2026-11-23): v7에 WORK 양방향 진입을 더한 시험판. v7과 같은 경기장이고 다른 것만 본다."""
import math
from itertools import pairwise

import pytest
from fleet_core.fleet_map import (
    FOLLOW_PATH,
    NAV_GOAL,
    TEST_LAYOUTS,
    FleetMapError,
    fleet_map_from_text,
    load_fleet_map,
    shortest_path,
)
from fleet_core.fleet_map.geometry import point_segment_distance, segment_rect_distance, wall_gap
from fleet_core.traffic.zone.routes import heading, move_command
from fleet_core.traffic.zone.turns import Turns

RELEASE_GAP_M = 0.135  # footprint_radius_m 0.085 + release_margin_m 0.05
NODE_CLEAR_M = 0.20
BODY_CIRCLE_M = 0.083
HALF_WIDTH_M = 0.056
WALL_NAV_M = 0.13
STANDING = ["W", "O", "Q1", "Q2", "O_IN", "O_OUT", "M_E", "O_X"]


@pytest.fixture(scope="module")
def v7():
    return load_fleet_map("v7")


@pytest.fixture(scope="module")
def bidir():
    return load_fleet_map("v7_bidir")


def test_bidir_is_a_test_layout_on_the_same_arena(v7, bidir):
    assert "v7_bidir" in TEST_LAYOUTS
    assert bidir.name == v7.name == "arena_v7"  # 같은 월드, 지도, keepout 마스크
    assert bidir.bounds() == v7.bounds()
    assert bidir.shelves == v7.shelves and bidir.sim_shelves == v7.sim_shelves
    assert bidir.keepout_extra == v7.keepout_extra
    assert bidir.zones == v7.zones
    assert bidir.parking == v7.parking
    assert {n: bidir.node_pose(n) for n in bidir.nodes} == {n: v7.node_pose(n) for n in v7.nodes}
    assert bidir.warnings == ()
    # v7은 그대로다
    assert not v7.exit_turn_cost and v7.stations["WORK"].lines == (("Q1", "Q2"),)
    assert math.degrees(v7.nodes["W"].yaw) == pytest.approx(90.0)


def test_bidir_changes_only_w_m_e_and_the_work_station(v7, bidir):
    assert bidir.exit_turn_cost
    assert bidir.nodes["W"].yaw is None
    assert bidir.nodes["M_E"].kind == "wait" and v7.nodes["M_E"].kind == "fork"
    for name in v7.nodes:
        if name not in ("W", "M_E"):
            assert bidir.nodes[name] == v7.nodes[name], name
    work = bidir.stations["WORK"]
    assert work.queue == ("Q1", "Q2", "M_E") and work.lines == (("Q1", "Q2"), ("M_E",))
    assert bidir.stations["OUTBOUND"] == v7.stations["OUTBOUND"]
    old = {(e.a, e.b): e for e in v7.edges()}
    new = {(e.a, e.b): e for e in bidir.edges()}
    assert set(new) - set(old) == {("M_E", "W")} and set(old) <= set(new)
    for key, edge in old.items():
        assert new[key].points == edge.points and new[key].mode == edge.mode, key
        # W -> M_E의 고정 0.3은 도크에서 도는 시간(exit_turn_cost)으로 바꿨다
        assert new[key].extra_m == (0.0 if key == ("W", "M_E") else edge.extra_m), key


def test_no_waiting_spot_fits_in_the_strip_north_of_w(v7):
    """W 북쪽 띠(WORK_DOCK 위 y 0.63 ~ J_E 아래 0.77)는 0.14 m다. 선 로봇이 두 구역을 쥐지 않으려면
    두 구역에서 0.135씩, 0.27이 필요하다."""
    found = []
    for i in range(0, 21):
        for j in range(0, 15):
            x, y = 1.78 + i * 0.01, 0.63 + j * 0.01
            if min(v7.distance_to_zone(x, y, "WORK_DOCK"),
                   v7.distance_to_zone(x, y, "J_E")) >= RELEASE_GAP_M:
                found.append((x, y))
    assert found == []


def test_north_entry_runs_from_m_e_and_stands_facing_south(bidir):
    edge = bidir.edge("M_E", "W")
    assert edge.mode == FOLLOW_PATH and edge.zones == ("HALL", "WORK_DOCK")
    assert edge.extra_m == pytest.approx(0.39)
    # 마지막은 x 1.86에서 곧게 남쪽, 꼭대기는 J_E 아래(놓기 거리와 상관없이 J_E를 쥐지 않는다)
    assert edge.points[-2] == pytest.approx((1.86, 0.61))
    assert max(y for _, y in edge.points) < 0.77 - 0.05
    assert math.degrees(heading(*edge.points[-2:])) == pytest.approx(-90.0)
    # 들어온 방향 그대로 선다: 남쪽에서 90도, 북쪽에서 270도
    south = move_command(bidir, "pode1", 1, [bidir.edge("Q1", "W")], *bidir.node_pose("Q1"))
    north = move_command(bidir, "pode1", 2, [edge], *bidir.node_pose("M_E"))
    assert math.degrees(south.yaw) == pytest.approx(90.0)
    assert math.degrees(north.yaw) == pytest.approx(-90.0)
    # 원호는 14도마다(제자리에서 꺾지 않는다)
    heads = [math.degrees(heading(a, b)) for a, b in pairwise(edge.points)]
    turns = [abs(math.remainder(math.radians(b - a), math.tau)) for a, b in pairwise(heads)]
    assert max(math.degrees(t) for t in turns) < 15.0


def test_north_entry_keeps_the_body_clear_and_every_standing_spot_open(bidir):
    edge = bidir.edge("M_E", "W")
    for x, y in edge.points[1:-1]:
        assert wall_gap(bidir.bounds(), x, y) > BODY_CIRCLE_M, (x, y)
    for a, b in pairwise(edge.points):
        for shelf in bidir.shelves:
            assert segment_rect_distance(a, b, shelf) > BODY_CIRCLE_M + 0.06, (a, b, shelf)
    # 고정 경로는 비켜 가지 못한다: 그 길에 없는 오래 서는 자리에서 몸 둘 + 0.01
    for name in STANDING:
        if name in ("M_E", "W"):
            continue
        here = bidir.node_pose(name)
        gap = min(point_segment_distance(here, a, b) for a, b in pairwise(edge.points))
        assert gap >= 2 * BODY_CIRCLE_M + 0.01, (name, round(gap, 3))


def test_the_north_waiting_spot_is_off_the_express_exit(bidir):
    """M_E는 v7의 오래 서는 자리 규칙을 이미 지킨다. 급행으로 나가는 띠(work_out, express)에서도
    몸 둘 + 0.01 떨어져 있다. M_E로 나가는 이음(W -> M_E)은 M_E에 로봇이 서 있으면 닫힌다."""
    me = bidir.node_pose("M_E")
    assert bidir.distance_to_zone(*me, "WORK_DOCK") >= RELEASE_GAP_M - 1e-9
    for zone in ("J_E", "EXPRESS", "OUT_DOCK"):
        assert bidir.distance_to_zone(*me, zone) >= RELEASE_GAP_M, zone
    for name in bidir.nodes:
        if name != "M_E":
            assert math.dist(me, bidir.node_pose(name)) >= NODE_CLEAR_M - 1e-9, name
    for a, b in (("W", "W_X"), ("W_X", "O_X")):
        edge = bidir.edge(a, b)
        gap = min(point_segment_distance(me, p, q) for p, q in pairwise(edge.points))
        assert gap >= 2 * BODY_CIRCLE_M + 0.01, (a, b, round(gap, 3))
    # 다른 길(고정 경로, Nav2 goal)도 M_E에 선 로봇을 지나갈 수 있다(v7 규칙 그대로)
    for edge in bidir.edges():
        if "M_E" in (edge.a, edge.b):
            continue
        gap = min(point_segment_distance(me, p, q) for p, q in pairwise(edge.points))
        assert gap >= BODY_CIRCLE_M + HALF_WIDTH_M, (edge.a, edge.b, round(gap, 3))
        if edge.mode != NAV_GOAL:
            assert gap >= 2 * BODY_CIRCLE_M + 0.01, (edge.a, edge.b, round(gap, 3))


def test_with_both_lines_free_the_yard_goes_round_by_the_south(bidir):
    """북쪽은 M_E에서 82도 돌고 W에서 남쪽을 본 채 나갈 때 104도 돈다(extra_m 0.39). 빈 경기장에서는
    마당 어디서든 남쪽(Q2, Q1)이 싸다. 북쪽은 남쪽 줄이 찼을 때 쓴다(관제 시험)."""
    for start in ("P-1", "P-2", "P-3", "E-1", "E-2", "E-3", "O_OUT"):
        way = shortest_path(bidir, start, "W")
        assert [edge.b for edge in way][-3:] == ["Q2", "Q1", "W"], start
    # 북쪽 줄 M_E에서는 자기 줄로만 들어간다(관제 길 값이 남쪽 줄 자리로 건너가는 이음을 막는다)
    station = bidir.stations["WORK"]
    way = shortest_path(bidir, "M_E", "W",
                        lambda edge: edge.price_m + Turns._cross_line(station, edge))
    assert [edge.b for edge in way] == ["W"]


@pytest.mark.parametrize(("old", "new", "word"), [
    ("lines: [[Q1, Q2], [M_E]]", "lines: [[Q1, Q2]]", "every queue spot"),
    ("lines: [[Q1, Q2], [M_E]]", "lines: [[Q1, Q2], [M_E, Q1]]", "every queue spot"),
    ("lines: [[Q1, Q2], [M_E]]", "lines: [Q1, Q2, M_E]", "list of queue spot lists"),
    ("line: true, lines:", "line: false, lines:", "needs line: true"),
    ("exit_turn_cost: true", "exit_turn_cost: 1", "exit_turn_cost"),
    ("[1.86, 0.61], W], extra_m: 0.39}", "[1.86, 0.61], W], extra_m: -1}", "extra_m"),
])
def test_bad_bidir_keys_are_refused(bidir, old, new, word):
    assert old in bidir.source_text
    with pytest.raises(FleetMapError, match=word):
        fleet_map_from_text(bidir.source_text.replace(old, new))
