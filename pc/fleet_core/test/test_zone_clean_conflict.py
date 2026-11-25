"""청소 로봇 겹침 판정의 파라미터: 다른 로봇 여유, 남이 쥔 지점(R2)과 남의 base(R4) 대상.

v7 교통 층에 운반 pode1(Q2)과 청소 pode3(P-1)을 놓고, 다리와 쥔 것과 경로를 손으로 정해
_clean_conflict, _clean_admit, _clean_take를 바로 부른다.
"""
from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import pytest
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map
from fleet_core.traffic.zone.cleaning import BACK, LINE, _ahead, _path_gap
from fleet_core.traffic.zone.controller import ZoneTraffic
from fleet_core.traffic.zone.routes import Route

# E-2(0.38, 0.24) 바로 위를 지나는 줄: 지도 지점은 E-2 하나만 든다
LINE_PTS = [(0.30, 0.30), (0.46, 0.30)]


def _traffic(**changes):
    site = load_fleet_map("v7")
    params = replace(load_params(), **changes)
    starts = {"pode1": "Q2", "pode3": "P-1"}
    t = ZoneTraffic(site, params, list(starts), lambda: 0.0, cleaners=("pode3",))
    for name, node in starts.items():
        x, y = site.node_pose(node)
        t.on_robot_state(name, x, y, 0.0, "IDLE")
    t.tick()
    leg = t._leg(LINE, "clean_line_0", LINE_PTS, None, 0)
    assert leg.nodes == ("E-2",)
    return t, t._robots["pode3"], t._robots["pode1"], leg


def _route(other, *polylines):
    edges = [SimpleNamespace(points=tuple(p)) for p in polylines]
    other.route = Route("X", edges, done=0, granted=len(edges))


# ---- 기본값은 지금 판정과 같다 ----
def test_defaults_keep_the_current_rule():
    p = load_params()
    assert (p.clean_conflict_reach_m, p.clean_hold_check, p.clean_base_check) == (0.18, "all",
                                                                                   "all")
    assert p.clean_conflict_ahead_m == 1.0


@pytest.mark.parametrize("name", ["clean_hold_check", "clean_base_check"])
def test_unknown_check_mode_is_refused(name):
    with pytest.raises(ValueError):
        replace(load_params(), **{name: "near"})


# ---- R2: 남이 쥔 지점 ----
def test_r2_all_defers_on_a_far_held_node():
    t, me, other, leg = _traffic()
    assert t.grants.take("pode1", [], ["E-2"])
    assert t._clean_conflict(me, leg) == "E-2 is held by pode1"
    _, why = t._clean_admit(me, leg)
    assert why == "E-2 is held by pode1"


def test_take_still_refuses_a_node_held_by_another():
    t, me, _, leg = _traffic(clean_hold_check="off")
    assert t.grants.take("pode1", [], ["E-2"])
    # 판정이 지점을 안 봐도 그 지점을 쥐려 하면 take가 거절하고 미룸 이유가 된다
    assert t._clean_take(me, leg).startswith("cannot take")
    assert t.grants.node_holder("E-2") == "pode1"
    assert "E-2" not in t.grants.nodes_of("pode3")


@pytest.mark.parametrize("mode", ["ahead", "off"])
def test_r2_ahead_or_off_skips_a_far_held_node_and_does_not_take_it(mode):
    t, me, other, leg = _traffic(clean_hold_check=mode, clean_conflict_ahead_m=0.5)
    assert t.grants.take("pode1", [], ["E-2"])
    assert t._clean_conflict(me, leg) == ""
    sent, why = t._clean_admit(me, leg)
    assert why == "" and sent.nodes == ()
    assert t.grants.node_holder("E-2") == "pode1"
    assert "E-2" not in t.grants.nodes_of("pode3")


def test_r2_ahead_counts_a_held_node_near_its_holder():
    t, me, other, leg = _traffic(clean_hold_check="ahead", clean_conflict_ahead_m=0.5)
    assert t.grants.take("pode1", [], ["E-2"])
    other.x, other.y = 0.38, 0.62  # E-2에서 0.38 m, 다리에서 0.32 m
    assert t._clean_conflict(me, leg) == "E-2 is held by pode1"


def test_r2_ahead_keeps_free_nodes_in_the_leg():
    t, me, _, leg = _traffic(clean_hold_check="ahead")
    sent, why = t._clean_admit(me, leg)
    assert why == "" and sent.nodes == ("E-2",)
    assert t.grants.node_holder("E-2") == "pode3"


# ---- R3: 다른 로봇 자리와 여유 ----
@pytest.mark.parametrize(("reach", "near"), [(0.18, True), (0.15, False)])
def test_r3_uses_the_conflict_reach(reach, near):
    t, me, other, leg = _traffic(clean_conflict_reach_m=reach)
    other.x, other.y = 0.38, 0.47  # 다리에서 0.17 m
    assert (t._clean_conflict(me, leg) == "pode1 stands near") is near


def test_conflict_reach_does_not_change_node_picking():
    t, me, _, _ = _traffic(clean_conflict_reach_m=0.10)
    leg = t._leg(LINE, "clean_line_0", [(0.30, 0.38), (0.46, 0.38)], None, 0)
    assert "E-2" in leg.nodes  # E-2에서 0.14 m: clean_reach_m 0.18로 고른다


# ---- R4: 남의 쥔 base ----
FAR_BASE = [(1.66, 0.15), (1.0, 0.15)], [(1.0, 0.15), (0.2, 0.15)]  # 다리에서 0.15 m, 1.2 m 앞


@pytest.mark.parametrize(("mode", "ahead", "near"), [
    ("all", 1.0, True), ("ahead", 0.5, False), ("ahead", 1.0, False), ("off", 1.0, False)])
def test_r4_modes_on_a_far_base(mode, ahead, near):
    t, me, other, leg = _traffic(clean_base_check=mode, clean_conflict_ahead_m=ahead)
    _route(other, *FAR_BASE)
    assert (t._clean_conflict(me, leg) == "pode1's base passes near") is near


def test_r4_ahead_sees_a_base_close_ahead():
    t, me, other, leg = _traffic(clean_base_check="ahead", clean_conflict_ahead_m=0.5)
    _route(other, *FAR_BASE)
    other.x, other.y = 0.9, 0.15  # 지금 위치에서 base를 따라 0.44 m에 다리가 있다
    assert t._clean_conflict(me, leg) == "pode1's base passes near"


def test_r4_reach_also_uses_the_conflict_reach():
    t, me, other, leg = _traffic(clean_conflict_reach_m=0.14)
    _route(other, *FAR_BASE)
    assert t._clean_conflict(me, leg) == ""


# ---- 앞쪽 자르기 ----
def test_ahead_starts_at_the_foot_and_stops_at_the_limit():
    edges = [SimpleNamespace(points=((0.0, 0.0), (1.0, 0.0))),
             SimpleNamespace(points=((1.0, 0.0), (1.0, 1.0)))]
    assert _ahead((0.2, 0.1), edges, 0.5) == [(0.2, 0.0), (0.7, 0.0)]
    out = _ahead((0.8, -0.1), edges, 0.5)
    assert out[:2] == [(0.8, 0.0), (1.0, 0.0)]
    assert out[2] == pytest.approx((1.0, 0.3))
    assert _ahead((0.0, 0.0), [], 1.0) == []


# ---- 집으로 피하는 다리: 남이 쥔 지점과 그 로봇의 base에서 멀어지기만 하면 보낸다 ----
HOME_PTS = [(0.232, 0.482), (0.1375, 0.2875), (0.12, 0.24)]  # O_OUT 옆에서 E-1로


def _on_the_way_from_o_out():
    """pode1이 O에서 O_OUT, E-3으로 가는 길을 받았고 청소 로봇은 O_OUT 옆 그 길 위에 서 있다."""
    site = load_fleet_map("v7")
    t = ZoneTraffic(site, load_params(), ["pode1", "pode3"], lambda: 0.0, cleaners=("pode3",))
    t.on_robot_state("pode1", *site.node_pose("O"), 0.0, "IDLE")
    t.on_robot_state("pode3", *HOME_PTS[0], 0.0, "IDLE")
    other = t._robots["pode1"]
    other.route = Route("E-3", [site.edge("O", "O_OUT"), site.edge("O_OUT", "E-3")], granted=2)
    # 청소 로봇은 줄을 거두고 서 있어 쥔 지점이 없다
    for node in t.grants.nodes_of("pode3"):
        t.grants.release_node("pode3", node)
    assert t.grants.force("pode1", [], ["O_OUT", "E-3"])
    return t, t._robots["pode3"], other


def test_an_escape_leg_may_leave_a_node_another_robot_holds():
    t, me, _ = _on_the_way_from_o_out()
    leg = t._leg(BACK, "E-1", HOME_PTS, None)
    assert "O_OUT" in leg.nodes
    assert t._clean_conflict(me, leg, "pode1") == ""
    sent, why = t._clean_admit(me, leg, "pode1")
    assert why == "" and "O_OUT" not in sent.nodes
    assert t.grants.node_holder("O_OUT") == "pode1"
    assert t.grants.node_holder("E-1") == "pode3"


def test_an_ordinary_leg_still_stops_at_a_held_node_it_starts_beside():
    t, me, _ = _on_the_way_from_o_out()
    leg = t._leg(BACK, "E-1", HOME_PTS, None)
    assert t._clean_conflict(me, leg) == "O_OUT is held by pode1"


def test_an_escape_leg_that_goes_nearer_the_held_node_is_refused():
    t, me, _ = _on_the_way_from_o_out()
    leg = t._leg(BACK, "E-1", [HOME_PTS[0], (0.21, 0.56)], None)
    assert t._clean_conflict(me, leg, "pode1") == "O_OUT is held by pode1"


def test_an_escape_leg_along_the_base_is_refused():
    t, me, _ = _on_the_way_from_o_out()
    # O_OUT에서 E-3으로 가는 base를 따라 나란히 간다
    leg = t._leg(BACK, "E-1", [HOME_PTS[0], (0.40, 0.343), (0.50, 0.26)], None)
    assert t._clean_conflict(me, leg, "pode1")



# ---- 집으로 가는 길: 남이 쥔 지점 옆을 지나는 곧은 길은 그 지점을 돌아간다 ----
def _parked_beside_home(here):
    """d2: 운반 로봇이 청소 로봇 집(P-1) 옆 P-2에 주차했고 청소 로봇은 줄을 다 돌고 here에 서 있다."""
    site = load_fleet_map("d2")
    t = ZoneTraffic(site, load_params(), ["pode1", "pode3"], lambda: 0.0, cleaners=("pode3",))
    t.on_robot_state("pode1", *site.node_pose("P-2"), 0.0, "IDLE")
    t.on_robot_state("pode3", *here, 0.0, "IDLE")
    for node in t.grants.nodes_of("pode3"):
        t.grants.release_node("pode3", node)
    assert t.grants.node_holder("P-2") == "pode1"
    return t, t._robots["pode3"], t._cleaners["pode3"], site


def test_the_way_home_goes_round_a_node_another_robot_holds():
    here = (1.31, 0.64)
    t, me, job, site = _parked_beside_home(here)
    p2, home = site.node_pose("P-2"), site.node_pose("P-1")
    # 곧은 길은 P-2 옆을 지난다
    assert _path_gap(p2, [here, home]) < t.params.clean_reach_m
    leg = t._back_leg(me, job)
    assert leg.points[0] == pytest.approx(here) and leg.points[-1] == pytest.approx(home)
    assert "P-2" not in leg.nodes
    assert _path_gap(p2, t._clean_plan.cov.rounded(leg.points, t._clean_plan.radius)) > \
        t.params.clean_reach_m
    assert t._clean_plan.clear(leg.points)
    assert t._clean_conflict(me, leg) == ""


def test_the_way_home_stays_straight_when_no_held_node_is_near():
    here = (0.14, 0.64)
    t, me, job, site = _parked_beside_home(here)
    leg = t._back_leg(me, job)
    assert [tuple(round(v, 3) for v in p) for p in leg.points] == [(0.14, 0.64), (0.13, 0.15)]
    assert t._clean_conflict(me, leg) == ""


def test_the_way_home_is_the_straight_one_when_the_held_node_cannot_be_passed():
    here = (1.31, 0.64)
    t, me, job, site = _parked_beside_home(here)
    # 집 바로 위 줄 전체를 남이 쥔 지점이 막으면 돌아갈 길이 없다: 곧은 길로 두고 판정이 미룬다
    t.params = replace(t.params, clean_reach_m=0.24)
    leg = t._back_leg(me, job)
    assert "P-2" in leg.nodes
    assert t._clean_conflict(me, leg) == "P-2 is held by pode1"


# ---- 막다른 자리: 그 지점 둘레를 지나야만 나갈 수 있으면 지점을 놓지 않는다 ----
def _standing(here, held):
    """운반 pode1은 P-2에 서 있고, 청소 pode3이 here에서 goal을 끝냈다."""
    site = load_fleet_map("v7")
    starts = {"pode1": "P-2", "pode3": "P-1"}
    t = ZoneTraffic(site, load_params(), list(starts), lambda: 0.0, cleaners=("pode3",))
    for name, node in starts.items():
        x, y = site.node_pose(node)
        t.on_robot_state(name, x, y, 0.0, "IDLE")
    t.tick()
    me = t._robots["pode3"]
    for node in t.grants.nodes_of("pode3"):
        t.grants.release_node("pode3", node)
    me.x, me.y = here
    assert t.grants.take("pode3", [], list(held))
    t._clean_release(me, t._cleaners["pode3"], ended=True)
    return t


# 둘째 자리는 Q2 둘레(0.18 m)를 5 mm 벗어났지만 그 칸의 중심은 둘레 안이다
@pytest.mark.parametrize("here", [(1.8625, 0.2125), (1.845, 0.150)])
def test_the_way_out_node_is_kept_at_the_dead_end_of_the_south_aisle(here):
    """Q2 동쪽 구석은 Q2 둘레를 지나야만 나간다. 놓으면 운반 로봇이 Q2에 서서 입구를 막는다."""
    t = _standing(here, ["Q1", "Q2"])
    assert _path_gap(t.site.node_pose("Q2"), [here]) > t.params.clean_reach_m
    assert t.grants.nodes_of("pode3") == ("Q1", "Q2")
    assert not t.grants.take("pode1", [], ["Q2"])


@pytest.mark.parametrize("here", [(1.4125, 0.2125), (1.475, 0.150)])
def test_a_node_off_the_way_out_is_released_as_before(here):
    """틈과 Q2 사이는 Q2를 지나지 않고 틈으로 나가므로 Q2를 놓는다."""
    t = _standing(here, ["Q2"])
    assert _path_gap(t.site.node_pose("Q2"), [here]) > t.params.clean_reach_m
    assert t.grants.nodes_of("pode3") == ()
    assert t.grants.take("pode1", [], ["Q2"])


def test_the_kept_node_is_let_go_once_the_cleaner_is_out_of_the_dead_end():
    """구석에서 goal이 끝나 쥐고 있던 Q2는, 둘레에 든 보고가 없었어도 구석을 벗어난 첫 보고에서 놓는다."""
    t = _standing((1.8625, 0.2125), ["Q1", "Q2"])
    me, job = t._robots["pode3"], t._cleaners["pode3"]
    assert "Q2" in t.grants.nodes_of("pode3")
    me.x, me.y = 1.4125, 0.2125
    t._clean_release(me, job, ended=False)
    assert t.grants.nodes_of("pode3") == ()
