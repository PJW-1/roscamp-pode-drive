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
from fleet_core.traffic.zone.cleaning import LINE, _ahead
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
