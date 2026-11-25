"""운반 로봇 둘과 청소 로봇 하나(cleaners: pode3)를 v7 가짜 세계에서 돌린다.

청소 로봇은 주문과 호출을 받지 않고, dock·lane 구역에 들어가지 않고, 줄을 차례로 보내 한 바퀴를
돌고 P-1로 돌아온다. 운반 주문은 그대로 끝난다. 바닥의 물체 앞에서는 기다렸다가 우회하고,
못 쓴 칸은 다음 바퀴의 짧은 줄이 된다.
"""
from __future__ import annotations

import pytest
from fake_robots.world import Obstacle
from fake_robots.world.world import ZoneWorld
from fleet_core.common.commands import DispatchResult, MoveCmd, MoveOrderCmd

CLEANER = "pode3"
STARTS = {"pode1": "P-2", "pode2": "P-3", CLEANER: "P-1"}


def _laps(w, n):
    return sum(e.code == "CLEAN_LAP_DONE" for _, e in w.events) >= n


@pytest.fixture(scope="module")
def fleet():
    w = ZoneWorld(layout="v7", seed=1, robots=dict(STARTS), cleaners=(CLEANER,))
    zones = set()
    while w.t < 900 and not (w.orders_done() and _laps(w, 1)):
        w.step()
        r = w.robots[CLEANER]
        zones.add(w.site.zone_at(r.x, r.y))
    return w, zones


def test_transport_orders_still_finish(fleet):
    w, _ = fleet
    assert w.orders_done()


def test_the_cleaner_gets_no_orders_or_calls(fleet):
    w, _ = fleet
    assert not w.commands_of(MoveOrderCmd, CLEANER)
    assert not [c for _, c in w.commands if isinstance(c, DispatchResult) and c.robot_id == CLEANER
                and c.success]
    assert all(c.mode in ("follow_path", "nav_goal") for c in w.commands_of(MoveCmd, CLEANER))


def test_the_cleaner_stays_out_of_dock_and_lane_zones(fleet):
    w, zones = fleet
    kinds = {w.site.zones[z].kind for z in zones if z is not None}
    assert kinds == {"shared"}


def test_a_lap_ends_at_home(fleet):
    w, _ = fleet
    done = [e for _, e in w.events if e.code == "CLEAN_LAP_DONE" and e.robot == CLEANER]
    assert done and "lap 1" in done[0].detail
    sent = [e for _, e in w.events if e.code == "CLEAN_SENT" and e.robot == CLEANER]
    lines = [e for e in sent if e.detail.startswith("line ")]
    assert lines and sent[0].detail.startswith("out ")
    finished = [e for _, e in w.events if e.code == "CLEAN_LINE_DONE" and e.robot == CLEANER]
    assert finished
    # 바퀴 끝 직전 다리는 P-1로 들어가는 후진이다
    end = next(i for i, (_, e) in enumerate(w.events) if e.code == "CLEAN_LAP_DONE")
    last = [e for _, e in w.events[:end] if e.code == "CLEAN_SENT" and e.robot == CLEANER]
    assert last[-1].detail.startswith("bay_in to P-1")


def test_status_and_events_show_clean(fleet):
    w, _ = fleet
    view = {v.robot_id: v for v in w.manager.robots_view()}
    assert view[CLEANER].task_type == "CLEAN"
    assert view[CLEANER].priority == 0
    assert all(view[n].priority > 0 for n in ("pode1", "pode2"))
    codes = {e.code for _, e in w.events if e.robot == CLEANER}
    assert {"CLEAN_LAP_START", "CLEAN_SENT", "CLEAN_LINE_DONE", "CLEAN_LAP_DONE"} <= codes


def test_no_invariant_breaks_by_the_cleaner(fleet):
    w, _ = fleet
    assert not [v for v in w.violations if v.robot == CLEANER or CLEANER in v.detail]


# ---- 바닥의 물체: 기다림 → 우회 → 빈틈, 우회 못 하면 미룸 ----
@pytest.fixture(scope="module")
def obstacle_run():
    w = ZoneWorld(layout="v7", seed=1, robots={CLEANER: "P-1"}, cleaners=(CLEANER,), orders=[],
                  injections=[Obstacle(0.35, 0.5875, 0.05)])  # 서쪽 넓은 곳 넷째 줄 위
    w.run_until(lambda: _laps(w, 1), 600)
    w.run_until(lambda: any(e.code == "CLEAN_LAP_START" and "lap 2" in e.detail
                            for _, e in w.events), 60)
    return w


def test_wait_then_detour_then_gap(obstacle_run):
    w = obstacle_run
    codes = [e.code for _, e in w.events if e.robot == CLEANER]
    assert "BLOCKED_WAIT" in codes
    assert "CLEAN_DETOUR" in codes and "CLEAN_GAP" in codes
    detour_goals = [c for c in w.commands_of(MoveCmd, CLEANER) if c.mode == "nav_goal"]
    assert detour_goals
    lap2 = next(e for _, e in w.events if e.code == "CLEAN_LAP_START" and "lap 2" in e.detail)
    assert "(0 gaps)" not in lap2.detail


def test_detour_that_cannot_run_defers_the_line(obstacle_run):
    w = obstacle_run
    failed = [e for _, e in w.events if e.code == "CLEAN_DETOUR_FAILED"]
    assert failed
    assert [e for _, e in w.events if e.code in ("CLEAN_LINE_DEFERRED", "CLEAN_LINE_SKIPPED")]
    assert not w.violations


# ---- 비키기(4-6): 청소 로봇이 Q1을 쥐고 운반 로봇이 Q2에서 기다리던 seed도 주문을 끝낸다 ----
@pytest.mark.parametrize("seed", [2, 5])
def test_seeds_that_used_to_stick_finish_their_orders(seed):
    w = ZoneWorld(layout="v7", seed=seed, robots=dict(STARTS), cleaners=(CLEANER,))
    w.run_until(w.orders_done, 600)
    assert w.orders_done()
    # 두 로봇 사이 위반(접촉, 근접)은 robot 칸이 비어 detail에 이름이 든다
    assert not [v for v in w.violations if v.robot == CLEANER or CLEANER in v.detail]


# ---- 청소 로봇이 없으면 비키기 파라미터는 아무것도 바꾸지 않는다 ----
def test_yield_parameters_do_nothing_without_a_cleaner():
    def run(overrides):
        w = ZoneWorld(layout="v7", seed=1, robots={"pode1": "P-2", "pode2": "P-3"},
                      overrides=overrides)
        w.run_until(w.orders_done, 400)
        return [(t, e.code, e.detail, e.robot) for t, e in w.events], w.commands

    odd = {"clean_yield_band_m": 2.0, "clean_yield_near_m": 2.0, "clean_yield_wait_s": 0.1,
           "clean_stuck_s": 0.1}
    assert run({}) == run(odd)
