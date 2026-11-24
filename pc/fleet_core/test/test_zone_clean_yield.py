"""청소 로봇 비키고 이어 하기(4-6)와 STOP 뒤 이어 하기(4-9).

교통 층(ZoneTraffic)만 v7 지도로 세우고 보고와 결과를 손으로 넣는다(test_zone_cleaning의 World).
"""
from __future__ import annotations

import math
from dataclasses import replace

from fleet_core.common.commands import CancelCmd, RetreatCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.mission import RUN, STOP
from fleet_core.traffic.zone.cleaning import BACK, BAY_IN, HOME, LINE, YIELD
from fleet_core.traffic.zone.robots import Dispatch, Goal
from fleet_core.traffic.zone.routes import Route
from test_zone_cleaning import CLEANER_NAME, World, _blocked

# ---- 막힘 꼴: 청소 로봇이 Q1 옆 막다른 곳에서 Q1을 쥐고, 운반 로봇은 Q2 -> Q1 -> W로 온다 ----
AT_Q1 = (1.854, 0.288)


def _at_q1(pode1_at: str):
    """pode1_at이 Q2면 이미 Q2에서 Q1을 기다리고, E-3면 Q2를 쥐고 오는 중이다."""
    w = World({"pode1": pode1_at, CLEANER_NAME: "P-1"})
    t, job, site = w.t, w.job, w.site
    w.pos[CLEANER_NAME] = AT_Q1
    w.report("pode1")
    w.report(CLEANER_NAME)
    assert t.grants.node_holder("Q1") == CLEANER_NAME
    job.lap, job.stage, job.queue, job.waiting = 1, BACK, [], []
    job.segs = list(t._clean_plan.lines)
    job.entered.add("Q1")
    t._robots[CLEANER_NAME].phase = "CLEANING"
    m = t._robots["pode1"]
    edges = [site.edge("Q2", "Q1"), site.edge("Q1", "W")]
    if pode1_at == "Q2":
        m.route = Route("W", edges)
        m.goal = Goal("move", 900, 0, "Q2", "nav_goal")
        t.grants.note_waiting("pode1", ("Q1",))
    else:
        m.route = Route("W", [site.edge(pode1_at, "Q2"), *edges], granted=1, sent=1)
        m.goal = Goal("move", 900, 1, "Q2", "nav_goal")
        assert t.grants.force("pode1", [], ["Q2"])
    m.phase, m.dest = "TO_WORK", t._work
    m.dispatch = Dispatch(77, t._work)
    return w


def _keep_waiting(w):
    if w.t.grants.node_holder("Q1") == CLEANER_NAME:
        w.t.grants.note_waiting("pode1", ("Q1",))


def _cancels(w, goal_id):
    return [o for o in w.out if isinstance(o, CancelCmd) and o.goal_id == goal_id]


def _arrive(w, move):
    w.pos[CLEANER_NAME] = (move.x, move.y)
    w.step()
    w.finish(move.goal_id)
    w.step()


def test_the_stuck_form_is_undone_by_yielding_before_the_transport_comes():
    w = _at_q1("E-3")
    w.step()
    yields = w.events("CLEAN_YIELD")
    assert yields and "pode1" in yields[0].detail
    assert w.job.leg.kind == YIELD
    move = w.moves("follow_path")[-1]
    end, p = (move.x, move.y), w.params
    band = w.t._clean_band(w.t._robots["pode1"], full=True)
    assert [w.site.node_pose(n) for n in ("Q2", "Q1", "W")] == band[1:]
    from fleet_core.traffic.zone.cleaning import _path_gap
    assert _path_gap(end, band) > p.clean_yield_band_m / 2 + p.footprint_radius_m
    # Q2는 pode1이 멀리 있어 쥐지 않고 지난다
    assert "Q2" not in w.job.leg.nodes and w.t.grants.node_holder("Q2") == "pode1"
    # 비킨 자리에 닿으면 Q1을 놓아 운반 로봇이 잡을 수 있다
    _arrive(w, move)
    assert w.t.grants.node_holder("Q1") == ""
    assert w.t.grants.take("pode1", [], ["Q1"])


def test_a_transport_waiting_for_a_node_the_cleaner_holds_is_a_meeting():
    w = _at_q1("Q2")
    other, why = w.t._clean_meeting(w.t._robots[CLEANER_NAME], w.job)
    assert other is not None and other.name == "pode1" and "Q1" in why


def test_too_late_to_yield_goes_home_and_a_blocked_home_way_is_clean_stuck():
    # pode1이 이미 Q2에 서 있으면 막다른 곳에서 나갈 길이 없다(Q2 옆 0.165 m)
    w = _at_q1("Q2")
    sent = len(w.moves())
    for _ in range(round((w.params.clean_stuck_s + 0.5) / 0.1)):
        _keep_waiting(w)
        w.step()
    assert w.events("CLEAN_HOME") and w.job.homing
    assert len(w.moves()) == sent
    stuck = w.events("CLEAN_STUCK")
    assert len(stuck) == 1 and stuck[0].level == "WARN"
    assert w.job.stuck_count == 1
    assert w.t.clean_view(CLEANER_NAME)["stuck"] == 1


def test_yield_saves_progress_and_resumes_from_it():
    w = World({"pode1": "P-2", CLEANER_NAME: "P-1"})
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.3)
    w.pos["pode1"] = w.at(a, b, 0.5)
    w.step()
    t, job = w.t, w.job
    assert t._clean_make_way(t._robots[CLEANER_NAME], t._robots["pode1"], "test")
    w.out += t.take_output()
    assert _cancels(w, line.goal_id)
    assert math.dist(job.segs[k][0], w.at(a, b, 0.3)) < 1e-6 and job.segs[k][1] == b
    assert job.queue[0] == k
    assert job.leg.kind == YIELD
    move = w.moves("follow_path")[-1]
    p = w.params
    assert math.dist((move.x, move.y), w.pos["pode1"]) > p.clean_yield_band_m / 2 + p.footprint_radius_m
    # 운반 로봇이 떠나면 진행점부터 그 줄을 잇는다
    w.pos["pode1"] = w.site.node_pose("P-2")
    w.state[CLEANER_NAME] = "IDLE"
    _arrive(w, move)
    for _ in range(3):
        w.step()
    again = w.moves("follow_path")[-1]
    assert job.leg is not None and job.leg.kind == LINE and job.leg.line == k
    pts = [pt[:2] for pt in again.path]
    assert any(math.dist(pt, w.at(a, b, 0.3)) < 1e-6 for pt in pts)
    assert math.dist(pts[-1], b) < 1e-6


def test_already_clear_of_the_way_just_stops_and_keeps_progress():
    w = World({"pode1": "P-2", CLEANER_NAME: "P-1"})
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.3)
    w.step()
    t = w.t
    sent = len(w.moves())
    assert t._clean_make_way(t._robots[CLEANER_NAME], t._robots["pode1"], "test")
    w.out += t.take_output()
    assert _cancels(w, line.goal_id)
    assert len(w.moves()) == sent
    assert w.job.goal_id is None and w.job.queue[0] == k
    assert math.dist(w.job.segs[k][0], w.at(a, b, 0.3)) < 1e-6
    assert len(w.events("CLEAN_YIELD")) == 1


def test_no_yield_point_sends_the_cleaner_home_and_the_lap_goes_on(monkeypatch):
    w = World({"pode1": "P-3", CLEANER_NAME: "P-1"})
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.3)
    w.step()
    t, job = w.t, w.job
    monkeypatch.setattr(t, "_clean_yield_leg", lambda *args: None)
    monkeypatch.setattr(t, "_clear_of", lambda *args: False)
    lap = job.lap
    assert t._clean_make_way(t._robots[CLEANER_NAME], t._robots["pode1"], "test")
    w.out += t.take_output()
    assert w.events("CLEAN_HOME") and job.homing and job.queue[0] == k
    back = w.moves()[-1]
    assert back.target_id == "E-1"
    _arrive(w, back)
    assert job.stage == BAY_IN
    bay = [o for o in w.out if isinstance(o, RetreatCmd) and o.robot == CLEANER_NAME][-1]
    w.pos[CLEANER_NAME] = w.site.node_pose("P-1")
    w.step()
    t.on_retreat_result(CLEANER_NAME, bay.goal_id, True)
    w.out += t.take_output()
    assert job.stage == HOME and not job.homing and job.refuge
    assert w.events("CLEAN_AT_HOME") and not w.events("CLEAN_LAP_DONE")
    w.step()
    assert len(w.moves()) and w.moves()[-1].goal_id == back.goal_id
    w.now += w.params.clean_yield_wait_s
    w.step()
    # 바퀴는 새로 시작하지 않고 남은 줄을 이어 간다
    assert job.lap == lap and job.queue[0] == k
    assert w.moves()[-1].target_id == "clean_start"
    assert len(w.events("CLEAN_LAP_START")) == 1


def test_still_meeting_after_yielding_goes_home(monkeypatch):
    w = World({"pode1": "P-2", CLEANER_NAME: "P-1"})
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.3)
    w.step()
    pode1 = w.t._robots["pode1"]
    monkeypatch.setattr(w.t, "_clean_meeting", lambda m, job: (pode1, "test"))
    monkeypatch.setattr(w.t, "_clear_of", lambda *args: False)
    w.step()
    assert w.events("CLEAN_YIELD") and not w.events("CLEAN_HOME")
    for _ in range(round((w.params.clean_yield_wait_s + 0.5) / 0.1)):
        w.step()
        if w.job.goal_id is not None and w.job.leg.kind == YIELD:
            w.pos[CLEANER_NAME] = w.job.leg.points[-1]
            w.finish(w.job.goal_id)
    assert w.events("CLEAN_HOME") and w.job.homing


def test_a_transport_blocked_by_the_cleaner_asks_it_to_yield():
    w = World({"pode1": "E-3", CLEANER_NAME: "P-1"})
    k, a, b, line = w.to_line()
    t = w.t
    m, c = t._robots["pode1"], t._robots[CLEANER_NAME]
    w.pos["pode1"] = w.at(a, b, 0.0)
    w.pos[CLEANER_NAME] = w.at(a, b, 0.2)
    w.step()
    m.route = Route("Q2", [w.site.edge("E-3", "Q2")])
    assert t._yield_to_robot(m, c, w.now)
    w.out += t.take_output()
    assert w.events("CLEAN_YIELD") and _cancels(w, line.goal_id)
    assert [e for e in w.events("HALL_YIELD") if e.robot == "pode1" and CLEANER_NAME in e.detail]
    assert not [o for o in w.out if isinstance(o, CancelCmd) and o.robot == "pode1"]


def test_a_moving_transport_closing_in_is_a_meeting():
    w = World({"pode1": "P-2", CLEANER_NAME: "P-1"})
    k, a, b, line = w.to_line()
    t, job = w.t, w.job
    c, m = t._robots[CLEANER_NAME], t._robots["pode1"]
    here = w.at(a, b, 0.1)
    w.pos[CLEANER_NAME] = here
    m.goal = Goal("move", 999, 0, "", "nav_goal")
    m.phase, m.dest, m.dispatch = "TO_WORK", t._work, Dispatch(77, t._work)
    # 옆에서 다가온다(띠는 위치 한 점뿐이고 띠 반폭 + 몸 반지름 밖)
    sign = 1.0 if here[1] < 0.4 else -1.0
    w.pos["pode1"] = (here[0], here[1] + sign * 0.40)
    w.step()
    assert m.goal is not None and job.goal_id is not None
    assert t._clean_meeting(c, job)[0] is None
    w.pos["pode1"] = (here[0], here[1] + sign * 0.28)
    w.report("pode1")
    other, why = t._clean_meeting(c, job)
    assert other is m and "closes in" in why


# ---- 4-9: STOP은 줄만 거두고, RUN이면 진행점부터 잇는다 ----
def test_stop_cancels_the_line_and_run_resumes_from_progress():
    w = World()
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.3)
    w.step()
    w.t.on_run_mode(STOP)
    w.out += w.t.take_output()
    assert _cancels(w, line.goal_id)
    assert w.job.goal_id is None
    sent = len(w.moves())
    w.state[CLEANER_NAME] = "IDLE"
    for _ in range(20):
        w.step()
    assert len(w.moves()) == sent
    w.t.on_run_mode(RUN)
    w.step()
    again = w.moves()[-1]
    assert w.job.leg.kind == LINE and w.job.leg.line == k
    pts = [pt[:2] for pt in again.path]
    assert any(math.dist(pt, w.at(a, b, 0.3)) < 1e-6 for pt in pts)
    assert math.dist(pts[-1], b) < 1e-6


def test_stop_during_the_arrival_wait_does_not_end_the_withdrawn_leg():
    w = World()
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.3)
    w.step()
    w.finish(line.goal_id)
    assert w.job.arrival_by is not None
    w.t.on_run_mode(STOP)
    w.out += w.t.take_output()
    assert w.job.arrival_by is None and w.job.goal_id is None
    # 거둔 뒤 끝에 선 보고가 와도 줄을 끝낸 것으로 치지 않는다
    w.pos[CLEANER_NAME] = b
    w.state[CLEANER_NAME] = "IDLE"
    for _ in range(5):
        w.step()
    assert not w.events("CLEAN_LINE_DONE") and k not in w.job.done
    assert not w.events("GOAL_RETRY")


def test_a_goal_withdrawn_while_waiting_for_arrival_is_not_judged():
    w = World()
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.3)
    w.step()
    w.finish(line.goal_id)
    # 바깥(멈춤 감시 등)이 goal만 거둔 경우
    w.t._cancel_goal(w.t._robots[CLEANER_NAME])
    w.pos[CLEANER_NAME] = b
    w.report()
    w.out += w.t.take_output()
    assert not w.events("CLEAN_LINE_DONE") and k not in w.job.done
    assert w.job.arrival_by is None and w.job.goal_id is None


def test_blocked_reports_during_stop_do_not_detour():
    w = World()
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.1)
    w.step()
    w.t.on_run_mode(STOP)
    w.out += w.t.take_output()
    _blocked(w, 3.0, obstacle=(*w.at(a, b, 0.2), "unknown"))
    assert not w.moves("nav_goal") and not w.events("CLEAN_DETOUR")


def test_the_stall_watch_skips_the_arrival_wait():
    params = replace(load_params(), arrival_settle_s=20.0)
    w = World(params=params)
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.3)
    w.step()
    w.finish(line.goal_id)
    assert w.job.arrival_by is not None
    w.state[CLEANER_NAME] = "IDLE"
    for _ in range(round((params.stall_s + 2.0) / 0.1)):
        w.step()
    assert not w.events("GOAL_STALLED")


def test_yield_parameters_are_loaded():
    p = load_params()
    assert p.clean_yield_band_m > 0 and p.clean_yield_near_m > 0
    assert p.clean_yield_wait_s > 0 and p.clean_stuck_s > 0
    assert BACK == "back"
