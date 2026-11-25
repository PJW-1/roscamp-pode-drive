"""청소 로봇 실행부(cleaning.py): 역할, 줄 보내기, BLOCKED 기다림, 우회, 빈틈, 미루기.

교통 층(ZoneTraffic)만 v7 지도로 세우고 보고와 결과를 손으로 넣는다. 로봇은 움직이지 않으니
보고 자리를 시험이 정한다.
"""
from __future__ import annotations

import json
import math
from types import SimpleNamespace

import pytest
from fleet_core.common.commands import CancelCmd, DispatchResult, EventOut, MoveCmd, MoveOrderCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map
from fleet_core.manager import FleetManager
from fleet_core.traffic.zone.cleaning import CLEANER, LINES, TRANSPORT
from fleet_core.traffic.zone.controller import ZoneTraffic

CLEANER_NAME = "pode3"


class World:
    def __init__(self, starts=None, params=None):
        self.now = 0.0
        self.site = load_fleet_map("v7")
        self.params = params or load_params()
        self.starts = starts or {CLEANER_NAME: "P-1"}
        self.t = ZoneTraffic(self.site, self.params, list(self.starts), lambda: self.now,
                             cleaners=(CLEANER_NAME,))
        self.pos = {name: self.site.node_pose(node) for name, node in self.starts.items()}
        self.state = dict.fromkeys(self.starts, "IDLE")
        self.out: list = []

    @property
    def job(self):
        return self.t._cleaners[CLEANER_NAME]

    def report(self, name=CLEANER_NAME, obstacle=None):
        x, y = self.pos[name]
        self.t.on_robot_state(name, x, y, 0.0, self.state[name], obstacle=obstacle)

    def step(self, dt=0.1, obstacle=None):
        self.now += dt
        for name in self.starts:
            self.report(name, obstacle if name == CLEANER_NAME else None)
        self.out += self.t.tick()

    def events(self, code):
        return [o for o in self.out if isinstance(o, EventOut) and o.code == code]

    def moves(self, mode=None):
        return [o for o in self.out if isinstance(o, MoveCmd) and o.robot == CLEANER_NAME
                and (mode is None or o.mode == mode)]

    def finish(self, goal_id, success=True, reason=""):
        self.t.on_move_result(CLEANER_NAME, goal_id, success, reason)
        self.out += self.t.take_output()

    def to_line(self, k=None):
        """나가기를 끝내고 줄 k(기본: 앞 0.8 m 띠가 금지 구역에 안 닿는 가장 긴 줄)를 첫 줄로 받아 그 시작점에 선다."""
        self.step()
        out = self.moves()[-1]
        self.pos[CLEANER_NAME] = (out.x, out.y)
        self.finish(out.goal_id)
        job = self.job
        if k is None:
            half = self.params.footprint_radius_m + self.params.clean_detour_margin_m / 2
            open_lines = [i for i, (a, b) in enumerate(job.segs) if not self.t._strip_refusal(
                [self.at(a, b, 0.0), self.at(a, b, min(math.dist(a, b), 0.8))], half)]
            k = max(open_lines, key=lambda i: math.dist(*job.segs[i]))
        job.queue = [k] + [i for i in job.queue if i != k]
        self.step()
        line = self.moves()[-1]
        assert job.stage == LINES and job.leg.line == k
        a, b = job.segs[k]
        self.pos[CLEANER_NAME] = a
        self.state[CLEANER_NAME] = "MOVING"
        self.step()
        return k, a, b, line

    def east_line(self):
        """위 통로에서 동쪽으로 가는 가장 긴 줄(충전 칸과 남쪽 통로의 운반 로봇에서 멀다)."""
        lines = self.t._clean_plan.lines
        return max((k for k, (a, b) in enumerate(lines) if a[0] < b[0] and a[1] > 0.5),
                   key=lambda k: math.dist(*lines[k]))

    def at(self, a, b, s):
        span = math.dist(a, b)
        return (a[0] + (b[0] - a[0]) * s / span, a[1] + (b[1] - a[1]) * s / span)


def _blocked(w, seconds, obstacle=None):
    w.state[CLEANER_NAME] = "BLOCKED"
    for _ in range(round(seconds / 0.1)):
        w.step(obstacle=obstacle)


# ---- 역할 ----
def test_roles_and_cleaner_refuses_calls():
    w = World({"pode1": "P-2", CLEANER_NAME: "P-1"})
    assert w.t.role(CLEANER_NAME) == CLEANER and w.t.role("pode1") == TRANSPORT
    w.step()
    w.t.on_dispatch(77, "WORK", CLEANER_NAME)
    w.out += w.t.take_output()
    answers = [o for o in w.out if isinstance(o, DispatchResult) and o.goal_id == 77]
    assert answers and not answers[0].success
    assert not [o for o in w.out if isinstance(o, MoveOrderCmd) and o.robot == CLEANER_NAME]


def test_cleaner_priority_is_lowest():
    w = World({"pode1": "P-2", CLEANER_NAME: "P-1"})
    robots = w.t._robots
    assert robots[CLEANER_NAME].sent_priority == 0
    assert robots["pode1"].sent_priority > 0


def test_one_cleaner_only_and_known_names():
    site, params = load_fleet_map("v7"), load_params()
    with pytest.raises(ValueError):
        ZoneTraffic(site, params, ["pode1", "pode2"], lambda: 0.0, cleaners=("pode1", "pode2"))
    with pytest.raises(ValueError):
        ZoneTraffic(site, params, ["pode1"], lambda: 0.0, cleaners=("pode9",))


def test_home_is_closed_to_transports():
    w = World({"pode1": "P-2", CLEANER_NAME: "P-1"})
    m = w.t._robots["pode1"]
    assert "P-1" in w.t._reserved(m)
    assert not w.t._reserved(w.t._robots[CLEANER_NAME])


def test_cleaner_cannot_take_dock_or_lane_zones():
    w = World()
    barred = [z.id for z in w.site.zones.values() if z.kind != "shared"]
    assert barred and w.t._barred(barred[:1], [])


# ---- 줄 보내기 ----
def test_lines_are_follow_path_in_the_floor_zone():
    w = World()
    k, a, b, line = w.to_line()
    assert line.mode == "follow_path" and line.path
    assert math.dist((line.x, line.y), b) < 1e-6
    for node in w.job.leg.nodes:
        assert w.site.zones[w.site.nodes[node].zone].kind == "shared"


def test_a_line_goal_not_sent_is_sent_again():
    w = World()
    k, a, b, line = w.to_line()
    w.t.on_goal_not_sent(CLEANER_NAME, line.goal_id)
    for _ in range(round(w.params.move_retry_wait_s / 0.1) + 2):
        w.step()
    again = w.moves("follow_path")[-1]
    assert again.goal_id != line.goal_id and again.target_id == f"clean_line_{k}"


def test_a_rejected_detour_puts_its_line_back():
    w = World()
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.1)
    _blocked(w, 2.3, obstacle=(*w.at(a, b, 0.2), "unknown"))
    nav = w.moves("nav_goal")[-1]
    w.t.on_goal_rejected(CLEANER_NAME, nav.goal_id)
    w.state[CLEANER_NAME] = "IDLE"
    for _ in range(round(w.params.move_retry_wait_s / 0.1) + 2):
        w.step()
    assert w.job.detour is None
    assert w.moves("follow_path")[-1].target_id == f"clean_line_{k}"


def test_a_late_answer_of_a_withdrawn_goal_is_dropped():
    w = World()
    k, a, b, line = w.to_line()
    m = w.t._robots[CLEANER_NAME]
    w.t._cancel_goal(m)
    w.t.on_goal_rejected(CLEANER_NAME, line.goal_id)
    w.out += w.t.take_output()
    assert m.attempts == 0 and not w.events("GOAL_RETRY")


def test_a_goal_not_sent_lets_go_of_its_nodes():
    w = World()
    k, a, b, line = w.to_line()
    held = set(w.t.grants.nodes_of(CLEANER_NAME))
    w.t.on_goal_not_sent(CLEANER_NAME, line.goal_id)
    m = w.t._robots[CLEANER_NAME]
    left = set(w.t.grants.nodes_of(CLEANER_NAME))
    assert left <= held
    assert all(w.t._gap(m, node) <= w.params.clean_reach_m for node in left)


def test_the_cleaner_is_not_placed_in_a_dock_or_lane_zone():
    site = load_fleet_map("v7")
    dock = next(n for n, node in site.nodes.items()
                if node.zone and site.zones[node.zone].kind != "shared")
    w = World({CLEANER_NAME: dock})
    w.step()
    m = w.t._robots[CLEANER_NAME]
    assert not m.placed and not w.t.grants.nodes_of(CLEANER_NAME)


def test_the_cleaner_is_not_placed_when_it_stands_in_a_dock_zone_near_a_shared_node():
    w = World()
    w.pos[CLEANER_NAME] = (0.49, 0.77)
    zone = w.site.zone_at(0.49, 0.77)
    assert zone is not None and w.site.zones[zone].kind != "shared"
    w.step()
    assert not w.t._robots[CLEANER_NAME].placed


def test_a_transport_does_not_step_up_onto_the_cleaner_home():
    w = World({"pode1": "P-2", CLEANER_NAME: "P-1"})
    m = w.t._robots["pode1"]
    m.node = "P-2"
    station = SimpleNamespace(line=True, queue=("P-1", "P-2"))
    assert w.t._line_step(m, station) is None


# ---- BLOCKED 기다림 ----
def test_wait_short_block_does_nothing():
    w = World()
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.1)
    _blocked(w, 1.5)
    assert w.events("BLOCKED_WAIT")
    assert not [o for o in w.out if isinstance(o, CancelCmd)]
    assert not w.events("CLEAN_DETOUR")


def test_wait_ends_when_progress_moves_on():
    """보고가 BLOCKED로 남아도(hold) 진행점이 나아가면 우회하지 않는다."""
    w = World()
    k, a, b, line = w.to_line()
    w.state[CLEANER_NAME] = "BLOCKED"
    for i in range(30):
        w.pos[CLEANER_NAME] = w.at(a, b, 0.1 + 0.01 * i)
        w.step()
    assert not [o for o in w.out if isinstance(o, CancelCmd)]
    assert not w.events("CLEAN_DETOUR")


def test_local_yield_is_not_counted():
    w = World()
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.1)
    # 영역 줄은 짧아서 0.5 m 앞을 장애물로 보면 복귀점이 줄 밖이다: 장애물 자리를 준다
    seen = (*w.at(a, b, 0.2), "unknown")
    _blocked(w, 1.5, obstacle=seen)
    w.state[CLEANER_NAME] = "RECOVERING"
    for _ in range(10):
        w.step()
    _blocked(w, 1.5, obstacle=seen)
    assert not w.events("CLEAN_DETOUR")
    _blocked(w, 0.8, obstacle=seen)
    assert w.events("CLEAN_DETOUR")


# ---- 우회 ----
def test_detour_goes_to_a_return_point_past_the_obstacle_and_leaves_a_gap():
    w = World()
    k, a, b, line = w.to_line()
    p = w.params
    w.pos[CLEANER_NAME] = w.at(a, b, 0.1)
    seen = w.at(a, b, 0.25)
    _blocked(w, 2.3, obstacle=(*seen, "unknown"))
    cancels = [o for o in w.out if isinstance(o, CancelCmd)]
    assert [c.goal_id for c in cancels] == [line.goal_id]
    nav = w.moves("nav_goal")
    assert len(nav) == 1 and w.events("CLEAN_DETOUR")
    back = (nav[0].x, nav[0].y)
    span = math.dist(a, b)
    s_back = math.dist(a, back)
    assert s_back >= 0.25 + p.footprint_radius_m + p.clean_detour_margin_m - 1e-6
    assert s_back <= span + 1e-6
    # 우회 goal도 같은 goal 수명이다
    m = w.t._robots[CLEANER_NAME]
    assert m.goal is not None and m.goal.id == nav[0].goal_id
    d = w.job.detour
    assert math.dist(d.start, w.at(a, b, 0.1)) < 1e-6 and d.back == back
    # 옆으로 비켜 돌아간다: 0.1~0.15는 줄 위, 그 뒤는 줄에서 0.2 m 옆
    w.state[CLEANER_NAME] = "MOVING"
    nx, ny = -(b[1] - a[1]) / span, (b[0] - a[0]) / span
    for i in range(6):
        w.pos[CLEANER_NAME] = w.at(a, b, 0.1 + 0.01 * i)
        w.step()
    for s in (0.2, 0.3, 0.4, s_back - 0.05):
        x, y = w.at(a, b, s)
        w.pos[CLEANER_NAME] = (x + 0.2 * nx, y + 0.2 * ny)
        w.step()
    w.pos[CLEANER_NAME] = back
    w.step()
    # 취소한 줄 goal의 늦은 결과는 버린다
    w.finish(line.goal_id, False, "CANCELED")
    assert w.job.detour is not None
    w.finish(nav[0].goal_id)
    gaps = w.events("CLEAN_GAP")
    assert len(gaps) == 1 and len(w.job.gaps) == 1
    g0, g1 = w.job.gaps[0]
    # 받은 위치로 덮인 진행점 근처는 빈틈이 아니다
    assert math.dist(a, g0) > 0.1 + 0.02
    assert math.dist(a, g1) <= s_back + 1e-6
    # 복귀점부터 남은 줄을 FOLLOW_PATH로 잇는다
    w.step()
    rest = w.moves("follow_path")[-1]
    assert math.dist((rest.x, rest.y), b) < 1e-6
    assert math.dist(rest.path[0][:2], back) < 1e-6


def test_gap_becomes_a_short_line_next_lap():
    w = World()
    k, a, b, line = w.to_line()
    w.job.gaps.append((w.at(a, b, 0.2), w.at(a, b, 0.4)))
    count = len(w.job.segs)
    w.job.stage, w.job.queue, w.job.waiting, w.job.leg = "home", [], [], None
    w.t._cancel_goal(w.t._robots[CLEANER_NAME])
    w.job.goal_id = None
    w.pos[CLEANER_NAME] = w.site.node_pose("P-1")
    w.state[CLEANER_NAME] = "IDLE"
    w.step()
    w.step()
    starts = w.events("CLEAN_LAP_START")
    assert "(1 gaps)" in starts[-1].detail
    assert len(w.job.segs) == count + 1 and w.job.gaps == []


def test_without_an_obstacle_point_looks_half_a_metre_ahead():
    w = World()
    k, a, b, line = w.to_line()
    p = w.params
    w.pos[CLEANER_NAME] = w.at(a, b, 0.05)
    _blocked(w, 2.3)
    nav = w.moves("nav_goal")
    if math.dist(a, b) < 0.05 + p.clean_block_look_m + p.footprint_radius_m + p.clean_detour_margin_m:
        assert w.events("CLEAN_DETOUR_FAILED")
        return
    assert nav
    assert math.dist(a, (nav[0].x, nav[0].y)) >= (0.05 + p.clean_block_look_m
                                                  + p.footprint_radius_m
                                                  + p.clean_detour_margin_m - 1e-6)


def test_no_return_point_defers_the_line():
    w = World()
    k, a, b, line = w.to_line()
    span = math.dist(a, b)
    w.pos[CLEANER_NAME] = w.at(a, b, span - 0.1)
    _blocked(w, 2.3, obstacle=(*w.at(a, b, span - 0.02), "unknown"))
    failed = w.events("CLEAN_DETOUR_FAILED")
    assert failed and "no free return point" in failed[0].detail
    assert w.events("CLEAN_LINE_DEFERRED") and not w.moves("nav_goal")
    assert k in w.job.waiting


def test_failed_detour_goal_defers_the_line():
    w = World()
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.1)
    _blocked(w, 2.3, obstacle=(*w.at(a, b, 0.2), "unknown"))
    nav = w.moves("nav_goal")[0]
    w.finish(nav.goal_id, False, "NAV_ABORTED")
    assert w.events("CLEAN_DETOUR_FAILED") and w.events("CLEAN_LINE_DEFERRED")
    assert w.job.detour is None and k in w.job.waiting and not w.job.gaps


def test_blocked_again_moves_the_return_point_once_then_defers():
    w = World()
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.1)
    _blocked(w, 2.3, obstacle=(*w.at(a, b, 0.2), "unknown"))
    first = w.moves("nav_goal")[-1]
    w.state[CLEANER_NAME] = "MOVING"
    w.step()
    _blocked(w, 2.3)
    second = w.moves("nav_goal")
    if len(second) == 1:  # 줄이 짧아 더 옮길 곳이 없다
        assert w.events("CLEAN_DETOUR_FAILED")
        return
    assert math.dist(a, (second[-1].x, second[-1].y)) > math.dist(a, (first.x, first.y))
    assert first.goal_id in [c.goal_id for c in w.out if isinstance(c, CancelCmd)]
    w.state[CLEANER_NAME] = "MOVING"
    w.step()
    _blocked(w, 2.3)
    assert len(w.moves("nav_goal")) == 2
    assert "again" in w.events("CLEAN_DETOUR_FAILED")[-1].detail
    assert k in w.job.waiting and w.job.detour is None


def test_a_transport_in_the_way_is_not_detoured():
    w = World({"pode1": "P-2", CLEANER_NAME: "P-1"})
    k, a, b, line = w.to_line()
    w.pos[CLEANER_NAME] = w.at(a, b, 0.1)
    w.pos["pode1"] = w.at(a, b, 0.3)
    w.state["pode1"] = "MOVING"
    _blocked(w, 2.3, obstacle=(*w.at(a, b, 0.2), "unknown"))
    assert not w.moves("nav_goal") and not w.events("CLEAN_DETOUR")
    assert any(f"line {k} (1): blocked for 2 s by pode1" in e.detail
               for e in w.events("CLEAN_LINE_DEFERRED"))


def test_detour_strip_must_stay_off_keepouts():
    w = World()
    rect = next(r for z in w.site.zones.values() if z.kind != "shared" for r in z.rects)
    inside = ((rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2)
    assert w.t._strip_refusal([inside, (inside[0] + 0.01, inside[1])], 0.1)


# ---- 기록 머리 ----
def test_the_record_head_names_the_cleaner():
    site = load_fleet_map("v7")
    core = FleetManager(site, load_params(), ["pode1", "pode2", CLEANER_NAME], lambda: 0.0,
                        (CLEANER_NAME,))
    written = []
    core.record_to(written.append)
    assert json.loads(written[0])["cleaners"] == [CLEANER_NAME]
    plain = FleetManager(site, load_params(), ["pode1", "pode2"], lambda: 0.0)
    written = []
    plain.record_to(written.append)
    assert "cleaners" not in json.loads(written[0])


# ---- 성공 결과의 도착 확인: 운반과 같은 arrival_tolerance_m, arrival_settle_s ----
KINDS = ("out", "line", "detour", "back_nav", "back", "bay_in")


def _leg_of(w, kind):
    """kind 다리를 보낸 상태로 만들고 (goal 번호, 끝 점, 결과 넣는 함수)를 돌려준다."""
    if kind == "out":
        w.step()
        cmd = w.moves()[-1]
        return cmd.goal_id, (cmd.x, cmd.y), w.finish
    k, a, b, line = w.to_line()
    if kind == "line":
        return line.goal_id, b, w.finish
    if kind == "detour":
        w.pos[CLEANER_NAME] = w.at(a, b, 0.1)
        _blocked(w, 2.3, obstacle=(*w.at(a, b, 0.2), "unknown"))
        nav = w.moves("nav_goal")[-1]
        w.state[CLEANER_NAME] = "MOVING"
        return nav.goal_id, (nav.x, nav.y), w.finish
    w.job.queue, w.job.waiting = [], []
    w.job.back_nav = kind == "back_nav"
    w.pos[CLEANER_NAME] = b
    w.finish(line.goal_id)
    w.step()
    assert w.job.leg.kind == ("back_nav" if kind == "back_nav" else "back")
    back = w.moves()[-1]
    if kind != "bay_in":
        return back.goal_id, (back.x, back.y), w.finish
    w.pos[CLEANER_NAME] = (back.x, back.y)
    w.finish(back.goal_id)
    w.step()
    assert w.job.leg.kind == "bay_in"

    def retreat(goal_id, success=True, reason=""):
        w.t.on_retreat_result(CLEANER_NAME, goal_id, success, reason)
        w.out += w.t.take_output()
    return w.job.goal_id, w.job.leg.points[-1], retreat


def _short_of(end):
    return (end[0] - 0.12, end[1] - 0.12)


@pytest.mark.parametrize("kind", KINDS)
def test_an_early_success_waits_for_the_arrival_report(kind):
    w = World()
    goal_id, end, result = _leg_of(w, kind)
    w.state[CLEANER_NAME] = "IDLE"
    w.pos[CLEANER_NAME] = _short_of(end)
    w.step()
    done = len(w.events("CLEAN_LINE_DONE")) + len(w.events("CLEAN_LAP_DONE"))
    result(goal_id)
    for _ in range(3):
        w.step()
    assert w.job.goal_id == goal_id
    assert len(w.events("CLEAN_LINE_DONE")) + len(w.events("CLEAN_LAP_DONE")) == done
    w.pos[CLEANER_NAME] = end
    w.step()
    assert w.job.goal_id != goal_id
    assert not w.events("GOAL_RETRY") and not [
        e for e in w.events("CLEAN_LINE_DEFERRED") if "NOT_AT_TARGET" in e.detail]


@pytest.mark.parametrize("kind", KINDS)
def test_an_early_success_without_arrival_fails_as_not_at_target(kind):
    w = World()
    goal_id, end, result = _leg_of(w, kind)
    w.state[CLEANER_NAME] = "IDLE"
    w.pos[CLEANER_NAME] = _short_of(end)
    w.step()
    result(goal_id)
    for _ in range(round(w.params.arrival_settle_s / 0.1) + 2):
        w.step()
    assert w.job.goal_id != goal_id
    if kind in ("line", "detour"):
        assert any("NOT_AT_TARGET" in e.detail for e in w.events("CLEAN_LINE_DEFERRED"))
    else:
        assert any("NOT_AT_TARGET" in e.detail for e in w.events("GOAL_RETRY"))
        assert not w.events("CLEAN_LAP_DONE")


# ---- 연결 길: 멈춤 오차로 reach 안에 선 자리 ----
def test_a_stop_just_inside_reach_still_joins_the_next_lines():
    """칸은 빈칸이어도 지금 자리가 reach 안이면 가장 가까운 빈칸 중심부터 검사해 다음 줄로 간다."""
    w = World()
    plan, job = w.t._clean_plan, w.job
    here = (0.598, 0.322)  # 남쪽 진열대 서쪽 모서리에서 0.105 m(진열대에서 떨어질 0.113 m 안)
    cell = plan.grid.cell(here)
    assert plan.free[cell[1], cell[0]] and not plan.clear([here])
    w.pos[CLEANER_NAME] = here
    w.report()
    m = w.t._robots[CLEANER_NAME]
    job.segs = list(plan.lines)
    for k in range(len(plan.lines)):
        leg = w.t._line_leg(m, job, k)
        assert leg is not None, k
        assert math.dist(leg.points[0], here) < 1e-9
