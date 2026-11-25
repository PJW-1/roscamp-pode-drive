# ---- 청소 로봇 실행부: 줄을 하나씩 보내고 점유와 바퀴를 맡는 믹스인 ----
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from itertools import pairwise

from fleet_core import tuck
from fleet_core.common.commands import MoveCmd
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, SHARED, Edge
from fleet_core.fleet_map.geometry import (
    point_segment_distance,
    segment_rect_distance,
    segments_cross,
)
from fleet_core.fleet_map.keepout import keepout_rects
from fleet_core.traffic.mission import (
    BLOCKED,
    CLEAN,
    CLEANING,
    HEARTBEAT_LOST,
    NOT_AT_TARGET,
    OUT_OF_SERVICE,
    PARKED,
)

from .robots import Goal
from .routes import heading, retreat_command

# ---- 역할과 바퀴 단계 ----
CLEANER, TRANSPORT = "cleaner", "transport"
HOME, OUT, LINES, BACK, BAY_IN = "home", "out", "lines", "back", "bay_in"
LINE, DETOUR, BACK_NAV, YIELD = "line", "detour", "back_nav", "yield"
BACKUP = "clean_backup"  # 비키기 다리 가운데 후진으로 보내는 것의 목표 이름
TUCK = "clean_tuck"  # 비키기 다리 가운데 통로 한쪽 면으로 붙어 서는 것의 목표 이름
PROGRESS_M = 0.02  # BLOCKED 기다림 중 진행점이 이만큼 앞으로 가면 기다림을 끝낸다
YIELD_TRIES = 40  # 비킬 점 후보 가운데 A* 경로를 찾아 볼 개수(가까운 순)
# ---- 겹침 검사 대상(clean_hold_check, clean_base_check): 전부, 가까운 앞쪽만, 끔 ----
ALL, AHEAD, OFF = "all", "ahead", "off"


# ---- 보낼 goal 하나: 나가기, 줄, 돌아오기, 집 자리 후진 ----
@dataclass(frozen=True)
class Leg:
    kind: str
    target: str
    points: tuple
    nodes: tuple
    end_yaw: float | None = None
    line: int = -1


# ---- 우회 하나: 진행점, 복귀점, 받은 위치 ----
@dataclass
class Detour:
    line: int
    start: tuple
    back: tuple
    s0: float
    s_back: float
    moves: int = 0
    trail: list = field(default_factory=list)


# ---- 청소 로봇 한 대의 바퀴 기록 ----
@dataclass
class CleanJob:
    home: str
    stage: str = HOME
    lap: int = 0
    lap_since: float = 0.0
    queue: list = field(default_factory=list)
    waiting: list = field(default_factory=list)
    deferrals: dict = field(default_factory=dict)
    done: list = field(default_factory=list)
    skipped: list = field(default_factory=list)
    retry_at: float = 0.0
    # 기다리는 다음 줄, 기다리기 시작한 시각, 다시 볼 시각
    wait_line: int | None = None
    wait_since: float | None = None
    wait_check_at: float = 0.0
    segs: list = field(default_factory=list)
    gaps: list = field(default_factory=list)
    leg: Leg | None = None
    goal_id: int | None = None
    entered: set = field(default_factory=set)
    blocked_since: float | None = None
    blocked_at: float | None = None
    arrival_by: float | None = None
    detour: Detour | None = None
    back_nav: bool = False
    not_home_warned: bool = False
    # 비키기(4-6): 비켜 준 운반 로봇, 처음 만난 시각, 집 자리 피난, 막힘 경고
    yield_for: str = ""
    escape_from: str = ""  # 비킬 곳이 없어 집으로 갈 때 피하는 운반 로봇
    yield_since: float | None = None
    homing: bool = False
    refuge: bool = False
    home_until: float = 0.0
    yield_try_at: float = 0.0
    stuck_since: float | None = None
    stuck_noted: bool = False
    stuck_count: int = 0
    yields: int = 0
    gaps_seen: dict = field(default_factory=dict)
    # 붙어 서기: 설 자리(tuck.Tuck), 붙기 전에 가던 줄, 붙어 준 운반 로봇
    tuck: tuck.Tuck | None = None
    tuck_line: int = -1
    tuck_for: str = ""


# ---- 청소 경로 자료: 단계 1의 커버리지 계획과 집 자리 고정 경로 ----
class CleanPlan:
    def __init__(self, site):
        from fleet_core import coverage
        self.cov = coverage
        plan = coverage.make_plan(site)
        self.params = plan.params
        self.grid = plan.grid
        self.free = plan.grid.free
        self.solids = tuple(dict.fromkeys(tuple(r) for r in (*site.shelves, *site.sim_shelves)))
        self.lines = [(a, b) for a, b, _ in plan.lines]
        self.start = plan.start
        self.home = coverage.HOME
        self.floor = coverage.FLOOR_ZONE
        self.radius = plan.params.turn_radius_m
        edges = list(site.edges())
        self.out_edge = next((e for e in edges if e.a == self.home and e.mode == FOLLOW_PATH),
                             None)
        self.in_edge = next((e for e in edges if e.b == self.home and e.mode == RETREAT), None)
        self._pockets = {}

    # ---- 지금 자리에서 점까지 청소 격자 위 꺾은선: (보낼 점들, 검사할 점들) ----
    # away: (점, 거리) 목록. 그 점들에서 그 거리 넘게 떨어진 길만 찾고, 없으면 ValueError
    def way(self, here, to, away=()) -> tuple[list, list]:
        # 빈칸 안이어도 멈춤 오차로 reach 안에 서면 가장 가까운 빈칸 중심부터 검사한다
        cell = self.grid.cell(here)
        inside = (cell is not None and self.free[cell[1], cell[0]]
                  and self.cov.path_clear(self.grid, [here]))
        first = here if inside else self.grid.centre(self.grid.nearest_free(here))
        pts = self.cov.transit(self.grid, self.free, first, to, self.radius, away)
        return ([] if inside else [here]) + pts, pts

    def clear(self, pts) -> bool:
        return self.cov.path_clear(self.grid, self.cov.rounded(pts, self.radius))

    # ---- 지금 자리에서 빈칸마다 청소 격자 위로 가는 거리(칸 단위) ----
    def steps(self, here):
        cell = self.grid.cell(here)
        if cell is None or not self.free[cell[1], cell[0]]:
            cell = self.grid.nearest_free(here)
        return self.cov.distances(self.free, cell)

    # ---- 막다른 자리: 그 점에서 reach 안을 지나야만 출발점으로 나갈 수 있는 자리인가 ----
    def shut_in(self, here, pose, reach: float) -> bool:
        key = (pose, reach)
        if key not in self._pockets:
            self._pockets[key] = self.cov.pocket(self.grid, self.free, self.start, pose, reach)
        return self.cov.in_pocket(self.grid, self.free, self._pockets[key], pose, reach, here)

    def covered(self, segs) -> float:
        hit = self.cov.covered_by(self.grid, self.params, [list(seg) for seg in segs])
        return float((hit & self.free).sum() / self.free.sum())


def clean_jobs(site, robots, cleaners) -> tuple[dict, CleanPlan | None]:
    """역할 설정을 확인하고 청소 로봇마다 바퀴 기록을 만든다."""
    cleaners = list(dict.fromkeys(cleaners))
    unknown = [name for name in cleaners if name not in robots]
    if unknown:
        raise ValueError(f"cleaners {unknown} are not in robots {list(robots)}")
    if not cleaners:
        return {}, None
    if len(cleaners) > 1:
        raise ValueError(f"one cleaner only, got {cleaners}")
    plan = CleanPlan(site)
    if plan.home not in site.parking:
        raise ValueError(f"the cleaner home {plan.home} is not a parking spot")
    return {name: CleanJob(plan.home) for name in cleaners}, plan


# ---- 선분과 선분 사이 거리 ----
def _segments_gap(a, b, c, d) -> float:
    if segments_cross(a, b, c, d):
        return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def _path_gap(point, pts) -> float:
    if len(pts) == 1:
        return math.dist(point, pts[0])
    return min(point_segment_distance(point, a, b) for a, b in pairwise(pts))


# ---- 점을 선분에 내린 자리 ----
def _foot(q, a, b) -> tuple:
    dx, dy = b[0] - a[0], b[1] - a[1]
    span2 = dx * dx + dy * dy
    t = 0.0 if span2 <= 1e-12 else ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / span2
    t = min(max(t, 0.0), 1.0)
    return (a[0] + t * dx, a[1] + t * dy)


# ---- 쥔 base에서 지금 위치에 가장 가까운 첫 구간 위 점부터 limit만큼만 자른 꺾은선 ----
def _ahead(here, edges, limit: float) -> list:
    if not edges:
        return []
    pts = []
    for edge in edges:
        for p in edge.points:
            if not pts or math.dist(pts[-1], p) > 1e-9:
                pts.append(tuple(p))
    if len(pts) < 2:
        return []
    # 가까운 점은 첫 구간 안에서만 찾는다(되돌아오는 길로 건너뛰지 않게)
    first = max(1, len(edges[0].points) - 1)
    i = min(range(min(first, len(pts) - 1)),
            key=lambda j: point_segment_distance(here, pts[j], pts[j + 1]))
    out, left = [_foot(here, pts[i], pts[i + 1])], limit
    for p in pts[i + 1:]:
        d = math.dist(out[-1], p)
        if d >= left:
            if d > 1e-9:
                t = left / d
                out.append((out[-1][0] + t * (p[0] - out[-1][0]),
                            out[-1][1] + t * (p[1] - out[-1][1])))
            break
        out.append(p)
        left -= d
    return out


class Cleaning:
    # ---- 역할 조회와 운반 로봇에 닫힌 집 자리 ----
    def role(self, robot) -> str:
        return CLEANER if robot in self._cleaners else TRANSPORT

    def _reserved(self, m) -> frozenset:
        return frozenset() if m.name in self._cleaners else self._homes

    def _join_role(self, m) -> None:
        if not self._cleaners:
            return
        m.sent_priority = self._priority(m)
        if m.name in self._cleaners:
            self._cleaners[m.name] = CleanJob(self._clean_plan.home)

    # ---- 한 걸음: 이어지는 goal을 보거나 다음 goal을 보낸다 ----
    def _clean_step(self, m) -> None:
        job, now = self._cleaners[m.name], self._clock()
        self._clean_reap(m, job)
        if job.goal_id is not None:
            if job.arrival_by is not None:
                self._clean_arrival(m, job, now)
            else:
                self._clean_watch_meeting(m, job, now)
            return
        if m.goal is not None or now < m.retry_at:
            return
        if self._clean_watch_meeting(m, job, now):
            return
        if job.stage == HOME:
            if job.refuge:
                # 피난 뒤: 잠시 쉬고 바퀴를 새로 시작하지 않고 남은 줄로 나간다
                if now < job.home_until:
                    return
                job.refuge = False
                job.stage = OUT if self._clean_plan.out_edge is not None else LINES
                m.phase = CLEANING
            elif not self._clean_start_lap(m, job, now):
                return
        leg = self._clean_next(m, job, now)
        if leg is not None:
            job.stuck_since, job.stuck_noted = None, False
            self._clean_send(m, job, leg)
        else:
            if job.homing:
                self._clean_note_stuck(m, job, now)
            self._clean_idle_meeting(m, job)

    # ---- 결과 없이 거둔 goal(멈춤, STOP, 통신 복구) 정리: 도착 대기도 함께 끝낸다 ----
    def _clean_reap(self, m, job) -> None:
        if job.goal_id is None or (m.goal is not None and m.goal.id == job.goal_id):
            return
        job.goal_id, job.arrival_by = None, None
        job.blocked_since = job.blocked_at = None
        self._clean_release(m, job, ended=True)

    def _clean_on_stop(self) -> None:
        for name, job in self._cleaners.items():
            self._clean_reap(self._robots[name], job)

    def _clean_waits_arrival(self, m) -> bool:
        job = self._cleaners.get(m.name)
        return job is not None and job.arrival_by is not None

    # ---- 바퀴 시작: 집 자리에서 나가거나 격자 위에서 바로 줄로 ----
    def _clean_start_lap(self, m, job, now: float) -> bool:
        plan = self._clean_plan
        at_home = self._gap(m, job.home) <= self.params.arrival_tolerance_m
        cell = plan.grid.cell((m.x, m.y))
        on_grid = cell is not None and plan.free[cell[1], cell[0]]
        if not (at_home or on_grid):
            if not job.not_home_warned:
                job.not_home_warned = True
                self._event("WARN", "CLEAN_NOT_HOME", f"at ({m.x:.2f}, {m.y:.2f}), neither at "
                            f"{job.home} nor on the clean grid, waits", m.name)
            return False
        job.lap += 1
        job.lap_since = now
        # 지난 바퀴에 우회로 못 쓴 구간은 짧은 줄로 뒤에 붙인다
        job.segs, job.gaps = [*plan.lines, *job.gaps], []
        job.queue = list(range(len(job.segs)))
        job.waiting, job.deferrals, job.done, job.skipped = [], {}, [], []
        job.detour = None
        job.retry_at = 0.0
        job.wait_line = job.wait_since = None
        job.stage = OUT if at_home and plan.out_edge is not None else LINES
        m.phase = CLEANING
        self._event("INFO", "CLEAN_LAP_START", f"lap {job.lap}, {len(job.segs)} lines "
                    f"({len(job.segs) - len(plan.lines)} gaps)", m.name)
        return True

    # ---- 단계별 다음 goal ----
    def _clean_next(self, m, job, now: float):
        if job.stage == OUT:
            return self._clean_ready(m, self._out_leg(m))
        if job.stage == LINES:
            # 결과 없이 끝난 줄·우회(유실, 못 보냄, 거절, heartbeat 끊김, STOP)는 진행점부터 맨 앞에 다시 둔다
            self._clean_requeue(m, job)
            leg = self._clean_line(m, job, now)
            if leg is not None or job.stage != BACK:
                return leg
        if job.stage == BACK:
            leaving = job.escape_from if job.homing else ""
            return self._clean_ready(m, self._back_leg(m, job, leaving), leaving)
        if job.stage == BAY_IN:
            return self._clean_ready(m, self._bay_leg(m))
        return None

    def _clean_line(self, m, job, now: float):
        while True:
            if not job.queue:
                if not job.waiting:
                    # 못 한 줄이나 빈틈이 남았으면 집에 가지 않고 그것부터 다음 바퀴를 돈다.
                    # 한 tick에 한 번만 넘긴다(모든 줄이 바로 포기되면 집으로)
                    if (job.skipped or job.gaps) and not job.homing and job.lap_since < now:
                        self._clean_carry_lap(m, job, now)
                        continue
                    job.stage = BACK
                    return None
                if now < job.retry_at:
                    return None
                job.queue, job.waiting = sorted(job.waiting), []
            k = job.queue[0]
            if job.wait_line == k and now < job.wait_check_at:
                return None
            leg = self._line_leg(m, job, k)
            if leg is None:
                why = "the joined path is not clear after rounding"
            elif self._clean_off_floor(leg):
                why = self._clean_conflict(m, leg)
            else:
                leg, why = self._clean_admit(m, leg)
                # 운반 로봇과 겹치면 다른 줄로 건너뛰지 않고 이 줄 앞에서 기다린다
                if why and self._clean_wait(m, job, k, why, now):
                    return None
            job.queue.pop(0)
            job.wait_line = job.wait_since = None
            if not why:
                return leg
            self._clean_defer(m, job, k, why)

    def _clean_off_floor(self, leg) -> bool:
        """다리가 공유 구역이 아닌 지점을 지난다(R1): 기다려도 풀리지 않는다."""
        site = self.site
        return any(site.nodes[n].zone is None or site.zones[site.nodes[n].zone].kind != SHARED
                   for n in leg.nodes)

    def _clean_wait(self, m, job, k: int, why: str, now: float) -> bool:
        """다음 줄 k를 clean_wait_max_s까지 기다린다. 기다리면 True, 다 기다렸으면 False."""
        if job.wait_line != k or job.wait_since is None:
            job.wait_line, job.wait_since = k, now
            self._event("INFO", "CLEAN_LINE_WAIT", f"line {k}: {why}", m.name)
        if now - job.wait_since >= self.params.clean_wait_max_s:
            return False
        job.wait_check_at = now + self.params.clean_wait_check_s
        return True

    def _clean_carry_lap(self, m, job, now: float) -> None:
        """포기한 줄이 남은 바퀴를 끝내고, 집에 가지 않고 그 줄을 맨 앞에 둔 다음 바퀴를 시작한다."""
        plan = self._clean_plan
        self._clean_lap_report(m, job, "continues from the skipped lines without going home")
        count = len(plan.lines)
        left = sorted(k for k in job.skipped if k < count)
        # 포기한 빈틈 줄은 다음 바퀴의 빈틈으로 넘긴다
        gaps = [job.segs[k] for k in sorted(job.skipped) if k >= count] + job.gaps
        job.lap += 1
        job.lap_since = now
        job.segs, job.gaps = [*plan.lines, *gaps], []
        first = left + list(range(count, len(job.segs)))
        job.queue = first + [k for k in range(len(job.segs)) if k not in first]
        job.waiting, job.deferrals, job.done, job.skipped = [], {}, [], []
        job.detour = None
        job.retry_at = 0.0
        job.wait_line = job.wait_since = None
        self._event("INFO", "CLEAN_LAP_START", f"lap {job.lap}, {len(job.segs)} lines "
                    f"({len(job.segs) - count} gaps), first {first}", m.name)

    def _clean_ready(self, m, leg, leaving: str = ""):
        leg, why = self._clean_admit(m, leg, leaving)
        return None if why else leg

    # ---- 다리 만들기 ----
    def _line_leg(self, m, job, k: int):
        plan = self._clean_plan
        a, b = job.segs[k]
        if job.tuck_line == k and self._tucked(m, job):
            # 붙어 선 자리에서는 줄 앞쪽 복귀점으로 비스듬히 돌아간다
            here = (m.x, m.y)
            if (tuck.clear(plan.solids, plan.grid.box, [here, a], plan.params.robot_radius_m)
                    and (math.dist(a, b) <= 1e-6 or plan.clear([a, b]))):
                return self._leg(LINE, f"clean_line_{k}", [here, a, b], None, k)
        sent, checked = plan.way((m.x, m.y), a)
        if not plan.clear([*checked, b]):
            return None
        return self._leg(LINE, f"clean_line_{k}", [*sent, b], None, k)

    def _out_leg(self, m):
        plan = self._clean_plan
        pts = [(m.x, m.y), *plan.out_edge.points[1:], plan.start]
        return self._leg(OUT, "clean_start", pts, None)

    def _back_leg(self, m, job, leaving: str = ""):
        plan, site = self._clean_plan, self.site
        # 돌아오는 길이 막혔으면 출발점까지 Nav2 goal로 돌아간다
        if job.back_nav and math.dist((m.x, m.y), plan.start) > self.params.arrival_tolerance_m:
            end = site.node_pose(plan.in_edge.a) if plan.in_edge else site.node_pose(plan.home)
            return self._leg(BACK_NAV, "clean_start", [(m.x, m.y), plan.start],
                             heading(plan.start, end) if plan.start != end else 0.0)
        if plan.in_edge is None:
            return self._home_leg(m, site.node_pose(plan.home), [], plan.home, leaving)
        entry = plan.in_edge.a
        return self._home_leg(m, plan.start, [site.node_pose(entry)], entry, leaving)

    # ---- 집으로 가는 다리: 남이 쥔 지점이 길을 막으면 그런 지점들을 돌아가는 길로 바꾼다 ----
    def _home_leg(self, m, to, tail, target, leaving: str):
        plan, here, yaw = self._clean_plan, (m.x, m.y), self.site.nodes[target].yaw
        leg = self._leg(BACK, target, [*plan.way(here, to)[0], *tail], yaw)
        if not any(self._held_against(m, node, leg.points, leaving) for node in leg.nodes):
            return leg
        gap = self.params.clean_reach_m + self.params.clean_yield_margin_m
        away = []
        for node in self.site.nodes:
            pose = self.site.node_pose(node)
            # 서 있는 자리나 끝 구간 옆 지점은 돌아갈 수 없다
            if (self._held_against(m, node, leg.points, "") and math.dist(pose, here) > gap
                    and _path_gap(pose, [to, *tail]) > gap):
                away.append((pose, gap))
        try:
            sent, _ = plan.way(here, to, away)
        except ValueError:
            return leg
        return self._leg(BACK, target, [*sent, *tail], yaw)

    def _bay_leg(self, m):
        edge = self._clean_plan.in_edge
        return self._leg(BAY_IN, edge.b, [(m.x, m.y), *edge.points[1:]], None)

    def _leg(self, kind, target, pts, end_yaw, line=-1) -> Leg:
        out = [tuple(pts[0])]
        for p in pts[1:]:
            if math.dist(out[-1], p) > 1e-6:
                out.append(tuple(p))
        reach = self.params.clean_reach_m
        nodes = tuple(name for name in self.site.nodes
                      if _path_gap(self.site.node_pose(name), out) <= reach)
        return Leg(kind, target, tuple(out), nodes, end_yaw, line)

    # ---- 겹침 검사와 쥐기: 볼 대상이 아닌 남의 지점은 다리에서 빼고 쥐지 않는다 ----
    def _clean_admit(self, m, leg, leaving: str = "") -> tuple:
        why = self._clean_conflict(m, leg, leaving)
        if why:
            return leg, why
        if self.params.clean_hold_check != ALL or leaving:
            held = self.grants.node_holder
            leg = replace(leg, nodes=tuple(n for n in leg.nodes if held(n) in ("", m.name)))
        return leg, self._clean_take(m, leg)

    # ---- 겹침 검사: 막힌 구역, 남이 쥔 지점, 남의 자리와 base ----
    def _clean_conflict(self, m, leg, leaving: str = "") -> str:
        """leaving: 집으로 피하는 다리가 비켜 주는 운반 로봇. 그 로봇이 청소 로봇이 쥔 지점을
        기다리는 것은 막지 않는다(떠나면 놓는다). 띠와 그 로봇의 base는 다리가 가로지르거나 곧
        벗어나지 않거나 벗어난 뒤 다시 닿으면 막는다. 남이 쥔 지점은 다리가 다가갈 때만 막는다."""
        site, grants, reach = self.site, self.grants, self.params.clean_conflict_reach_m
        for node in leg.nodes:
            zone = site.nodes[node].zone
            if zone is None or site.zones[zone].kind != SHARED:
                return f"{node} is in {zone}, not a shared zone"
            if self._held_against(m, node, leg.points, leaving):
                return f"{node} is held by {grants.node_holder(node)}"
        for other in self._robots.values():
            if other is m or not other.placed or other.health.fault == OUT_OF_SERVICE:
                continue
            if _path_gap((other.x, other.y), leg.points) <= reach:
                return f"{other.name} stands near"
            escape = other.name == leaving
            if escape:
                base = self._base_line(other)
                if len(base) >= 2 and not self._leaves_band(leg.points, base):
                    return f"does not leave {other.name}'s base"
            elif self._base_near(other, leg.points, reach):
                return f"{other.name}'s base passes near"
            # 운반 로봇의 만남 띠(4-6)에 닿거나 그 로봇이 기다리는 지점을 지나는 다리는 보내지 않는다
            wanted = [n for n in self._wanted(other) if n in leg.nodes
                      and not (escape and grants.node_holder(n) == m.name)]
            if wanted:
                return f"{other.name} waits for {wanted}"
            if escape:
                band = self._clean_band(other)
                if band is not None and not self._leaves_band(leg.points, band):
                    return f"does not leave {other.name}'s way"
                continue
            band = self._clean_band(other) if self._band_on_legs() else None
            if band is not None and self._ways_meet(leg.points, band):
                return f"meets {other.name}'s way"
        return ""

    def _band_ahead(self, band):
        """운반 로봇이 곧 지날 띠: 첫 구간 전체와, 그 뒤는 로봇에서 clean_conflict_ahead_m 안."""
        if len(band) < 2:
            return list(band)
        out = [band[0], band[1]]
        left = self.params.clean_conflict_ahead_m - math.dist(band[0], band[1])
        for a, b in pairwise(band[1:]):
            if left <= 1e-9:
                break
            span = math.dist(a, b)
            if span > left:
                out.append((a[0] + (b[0] - a[0]) * left / span, a[1] + (b[1] - a[1]) * left / span))
                break
            out.append(b)
            left -= span
        return out

    def _leaves_band(self, pts, band) -> bool:
        """띠 안에서 떠나는 다리: 띠를 가로지르지 않고, clean_yield_band_m 안에 띠에서
        clean_conflict_reach_m 밖으로 나가며, 나간 뒤로는 다시 닿지 않는다."""
        reach, step = self.params.clean_conflict_reach_m, self._clean_plan.grid.res
        if len(band) >= 2 and any(segments_cross(a, b, c, d) for a, b in pairwise(pts)
                                  for c, d in pairwise(band)):
            return False
        if len(pts) < 2:
            return _path_gap(pts[0], band) > reach
        walked = 0.0
        for i, (a, b) in enumerate(pairwise(pts)):
            span = math.dist(a, b)
            for n in range(math.ceil(span / step) + 1):
                s = min(n * step, span)
                q = (a[0] + (b[0] - a[0]) * s / span, a[1] + (b[1] - a[1]) * s / span) if span else a
                if walked + s > self.params.clean_yield_band_m + 1e-9:
                    return False
                if _path_gap(q, band) > reach:
                    return not self._ways_meet([q, *pts[i + 1:]], band)
            walked += span
        return False

    def _band_on_legs(self) -> bool:
        """다리와 띠의 만남은 base를 모두 볼 때(clean_base_check all)만 본다. 보내기와 비키기가 같은 기준이다."""
        return self.params.clean_base_check == ALL

    # ---- 만남 판단(4-6): 운반 로봇의 지금 위치 → 목표 노드 → base 노드 → 기다리는 노드를 이은 띠 ----
    def _wanted(self, other) -> list:
        return [n for n in self.grants.waiting().get(other.name, ()) if n in self.site.nodes]

    def _clean_band(self, other, full: bool = False):
        """띠의 중심선 점들. full이면 base 대신 남은 route 전체. 가는 곳이 없는 로봇은 None."""
        site, route, goal = self.site, other.route, other.goal
        wanted = self._wanted(other)
        ends, seen = [], set()
        if route is not None:
            stop = len(route.edges) if full else route.granted
            edges = list(route.edges[route.done:stop])
            for edge in route.edges[stop:]:
                if getattr(edge, "b", None) not in wanted:
                    break
                edges.append(edge)
            for edge in edges:
                ends.append(tuple(edge.points[-1]))
                seen.add(getattr(edge, "b", None))
        if not ends and goal is not None and goal.end in site.nodes:
            ends.append(site.node_pose(goal.end))
            seen.add(goal.end)
        ends += [site.node_pose(n) for n in wanted if n not in seen]
        if not ends and goal is None:
            return None
        pts = [(other.x, other.y)]
        for p in ends:
            if math.dist(pts[-1], p) > 1e-9:
                pts.append(p)
        return pts

    def _band_reach(self) -> float:
        return self.params.clean_yield_band_m / 2 + self.params.footprint_radius_m

    def _ways_meet(self, pts, band) -> bool:
        """다리와 띠의 만남. 보내기 검사와 같은 거리(clean_conflict_reach_m)로 봐서 보낸 다리가 곧바로 만나지 않는다."""
        reach = self.params.clean_conflict_reach_m
        if len(pts) < 2 or len(band) < 2:
            one, other = (pts, band) if len(pts) == 1 else (band, pts)
            return _path_gap(one[0], other) <= reach
        return any(_segments_gap(a, b, c, d) <= reach
                   for a, b in pairwise(pts) for c, d in pairwise(band))

    def _clean_meeting(self, m, job):
        """만난 운반 로봇과 까닭. 기다리는 지점을 쥠, 서 있으면 띠 안의 자리, 달리면 남은 다리, 다가오는 운반 로봇."""
        here, p = (m.x, m.y), self.params
        held = set(self.grants.nodes_of(m.name))
        moving = job.goal_id is not None
        rest = self._clean_rest(m, job) if moving and self._band_on_legs() else []
        met, why = None, ""
        for other in self._robots.values():
            if (other is m or other.name in self._cleaners or not other.placed
                    or other.health.fault == OUT_OF_SERVICE):
                continue
            # 다가옴은 운반 로봇이 움직인 것만 본다: 지난 자리와 지금 자리를 청소 로봇의 지금 자리에서 잰다
            spot, now = (other.x, other.y), self._clock()
            gap = math.dist(here, spot)
            seen_at, before = job.gaps_seen.get(other.name, (None, None))
            job.gaps_seen[other.name] = (now, spot)
            last = None
            if seen_at is not None and now - seen_at <= 2 * p.move_retry_wait_s:
                last = math.dist(here, before)  # 오래된 자리는 쓰지 않는다
            if met is not None:
                continue
            wanted = [n for n in self._wanted(other) if n in held]
            band = self._clean_band(other)
            if wanted:
                met, why = other, f"{other.name} waits for {wanted} it holds"
            elif not moving and band is not None and _path_gap(here, band) <= self._band_reach():
                met, why = other, f"stands in {other.name}'s way"
            elif band is not None and len(rest) >= 2 and self._ways_meet(rest, band):
                met, why = other, f"its way meets {other.name}'s way"
            elif (other.goal is not None and last is not None
                  and gap <= p.clean_yield_near_m and gap < last - 1e-3):
                # 서 있을 때도 본다: 다가오는 동안 새 다리를 보내지 않고 기다린다
                met, why = other, f"{other.name} closes in ({gap:.2f} m)"
        return met, why

    def _clean_rest(self, m, job) -> list:
        """지금 다리에서 아직 가지 않은 꺾은선(지금 위치부터)."""
        here = (m.x, m.y)
        pts = job.leg.points if job.leg is not None else ()
        if len(pts) < 2:
            return [here]
        i = min(range(len(pts) - 1), key=lambda j: point_segment_distance(here, pts[j], pts[j + 1]))
        return [here, *pts[i + 1:]]

    # ---- 비키기(4-6): 만나면 줄을 거두고 진행점을 남긴 뒤 띠 밖으로 비킨다 ----
    def _clean_watch_meeting(self, m, job, now: float) -> bool:
        """만나서 비키기를 했거나 비키는 중이면 True."""
        if job.stage not in (LINES, BACK) or job.homing:
            return False
        if job.leg is not None and job.leg.kind == YIELD and job.goal_id is not None:
            return True
        other, why = self._clean_meeting(m, job)
        if other is None:
            # 비켜 준 로봇의 남은 길에서 벗어났을 때만 만남 시계를 지운다(선 채 만남과 같은 기준)
            last = self._robots.get(job.yield_for)
            if last is None or self._clear_of(m, last):
                job.yield_for, job.yield_since, job.tuck_for = "", None, ""
            return False
        self._clean_make_way(m, other, why)
        return True

    def _clean_make_way(self, m, other, why: str) -> bool:
        """청소 로봇 m을 운반 로봇 other의 길에서 비키게 한다. 비켰거나 비키러 가면 True."""
        job, now, p = self._cleaners[m.name], self._clock(), self.params
        if not self.running or not m.available or self._localizing(m):
            return False
        if job.stage not in (LINES, BACK):
            return False
        if job.leg is not None and job.leg.kind == YIELD and job.goal_id is not None:
            return True
        if job.homing:
            return job.goal_id is not None or self._clean_yield_on_the_way_home(m, job, other, why)
        tucked = self._tucked(m, job)
        if job.yield_for != other.name or job.yield_since is None:
            job.yield_for, job.yield_since = other.name, now
        elif now - job.yield_since >= (p.clean_tuck_wait_s if tucked else p.clean_yield_wait_s):
            return self._clean_go_home(m, job, f"{other.name} still meets it "
                                       f"{now - job.yield_since:.0f} s after yielding ({why})",
                                       other.name)
        if tucked:
            # 붙어 서서 운반 로봇이 옆으로 지나가기를 기다린다
            return True
        clear = self._clear_of(m, other)
        if job.goal_id is None and clear:
            return True
        line = job.leg.line if job.leg is not None and job.leg.kind == LINE else -1
        if job.goal_id is not None:
            self._clean_cancel(m, job)
        self._clean_requeue(m, job)
        job.yields += 1
        if clear:
            self._event("INFO", "CLEAN_YIELD", f"stops for {other.name}: {why}", m.name)
            return True
        # 좁은 통로에서는 한쪽 면으로 붙어 서서 옆으로 지나가게 한다
        if self._clean_tuck(m, job, other, why, line):
            return True
        # 먼저 몸 방향 그대로 후진해 빠지고, 안 되면 비킬 점으로 간다
        leg = self._clean_backup_leg(m, other)
        if leg is not None and self._clean_take(m, leg):
            leg = None
        how = "backs up" if leg is not None else "goes"
        if leg is None:
            leg = self._clean_yield_leg(m, other)
        refusal = "no place out of its way" if leg is None else self._clean_take(m, leg)
        if refusal:
            return self._clean_go_home(m, job, f"cannot yield to {other.name}: {refusal}",
                                       other.name)
        end = leg.points[-1]
        self._event("INFO", "CLEAN_YIELD", f"{how} for {other.name} to ({end[0]:.3f}, "
                    f"{end[1]:.3f}): {why}", m.name)
        self._clean_send(m, job, leg)
        return True

    # ---- 붙어 서기: 좁은 통로에서 한쪽 면으로 붙어 운반 로봇의 Nav2가 옆으로 지나가게 한다 ----
    def _tucked(self, m, job) -> bool:
        """붙어 설 자리 둘레에 서 있는가(다리가 끝났다)."""
        return (job.tuck is not None and job.goal_id is None
                and math.dist((m.x, m.y), job.tuck.spot) <= self.params.arrival_tolerance_m)

    def _clean_tuck(self, m, job, other, why: str, line: int) -> bool:
        """붙어 서러 가면 True. 같은 운반 로봇에게는 한 번만 붙는다(못 가면 원래 비키기로)."""
        found = None if job.tuck_for == other.name else self._clean_tuck_leg(m, other)
        if found is None or self._clean_take(m, found[0]):
            return False
        leg, spot = found
        line = self._tuck_rejoin(m, job, spot, line)
        self._clean_send(m, job, leg)
        job.tuck, job.tuck_line, job.tuck_for = spot, line, other.name
        self._event("INFO", "CLEAN_YIELD", f"tucks in for {other.name} at ({spot.spot[0]:.3f}, "
                    f"{spot.spot[1]:.3f}), {spot.rows} rows left beside: {why}", m.name)
        return True

    def _clean_tuck_leg(self, m, other):
        """통로의 좌표가 큰 쪽 면에 붙어 통로와 나란히 서는 다리와 설 자리. 없으면 None."""
        plan, p, site = self._clean_plan, self.params, self.site
        if p.clean_tuck_rows <= 0:
            return None
        here, pos = (m.x, m.y), (other.x, other.y)
        rest = [o for o in self._robots.values() if o is not m and o is not other and o.placed]
        # 운반 로봇이 기다리거나 앞으로 지날 지점
        route = other.route
        coming = set(self._wanted(other)) | {
            getattr(edge, "b", None) for edge in (route.edges[route.done:] if route else ())}
        along_x = abs(math.cos(m.yaw)) >= abs(math.sin(m.yaw))
        for axis in ((0, 1) if along_x else (1, 0)):
            spot = tuck.plan(plan.solids, plan.grid.box, here, m.yaw, axis,
                             clear=p.clean_tuck_clear_m, radius=plan.params.robot_radius_m,
                             rows=p.clean_tuck_rows)
            if spot is None:
                continue
            pts = list(spot.path)
            # 가는 길이 운반 로봇이나 다른 로봇에 가까우면 쓰지 않는다
            if (_path_gap(pos, pts) <= p.clean_yield_near_m
                    or any(_path_gap((o.x, o.y), pts) <= p.clean_conflict_reach_m for o in rest)):
                continue
            leg = self._leg(YIELD, TUCK, pts, spot.yaw)
            # 운반 로봇이 지날 지점 둘레에 서면 그 지점을 쥔 채 길을 막는다
            if self._clean_off_floor(leg) or coming & set(leg.nodes):
                continue
            # 선 몸에서 운반 로봇 쪽으로 찍히는 띠가 그 로봇이 갈 지점을 덮으면 계획이 닿지 못한다
            if any(tuck.lane_shuts(spot.spot, spot.yaw, pos, p.clean_tuck_lane_m, site.node_pose(n))
                   for n in coming if n):
                continue
            held = [(n, self.grants.node_holder(n)) for n in leg.nodes]
            if any(h not in ("", m.name)
                   and _path_gap(site.node_pose(n), pts) < math.dist(site.node_pose(n), here) - 1e-6
                   for n, h in held):
                continue
            return replace(leg, nodes=tuple(n for n, h in held if h in ("", m.name))), spot
        return None

    def _tuck_rejoin(self, m, job, spot, k: int) -> int:
        """붙기 전에 가던 줄 k를 복귀점부터로 줄이고, 건너뛰는 구간에서 덜 닦인 곳을 빈틈으로 남긴다.
        줄 위에서 줄 방향으로 붙는 것이 아니면 -1(줄은 진행점부터 그대로 다시 간다)."""
        if k < 0 or not job.queue or job.queue[0] != k:
            return -1
        a, u, span = self._axis(job.segs[k])
        along = u[0] * math.cos(spot.yaw) + u[1] * math.sin(spot.yaw)
        if math.dist((m.x, m.y), a) > self.params.arrival_tolerance_m / 2 or along < 0.99:
            return -1
        s_back = min(max(self._along(a, u, spot.back), 0.0), span)
        back = (a[0] + u[0] * s_back, a[1] + u[1] * s_back)
        self._clean_note_gaps(m, job, k, Detour(k, a, back, 0.0, s_back, trail=[*spot.path, back]))
        job.segs[k] = (back, job.segs[k][1])
        return k

    def _clean_yield_on_the_way_home(self, m, job, other, why: str) -> bool:
        """집으로 가는 길이 막혀 선 동안에도 그 운반 로봇의 길 위면 비킬 점을 clean_retry_s마다 다시 찾는다."""
        now = self._clock()
        if self._clear_of(m, other) or now < job.yield_try_at:
            return False
        job.yield_try_at = now + self.params.clean_retry_s
        leg = self._clean_yield_leg(m, other)
        if leg is None or self._clean_take(m, leg):
            return False
        job.yields += 1
        end = leg.points[-1]
        self._event("INFO", "CLEAN_YIELD", f"for {other.name} to ({end[0]:.3f}, {end[1]:.3f}) "
                    f"on the way home: {why}", m.name)
        self._clean_send(m, job, leg)
        return True

    def _clear_of(self, m, other) -> bool:
        """지금 자리가 그 로봇의 띠 밖이고 그 로봇이 기다리는 지점을 쥐지 않았는가."""
        here = (m.x, m.y)
        band = self._clean_band(other, full=True) or [(other.x, other.y)]
        if _path_gap(here, band) <= self._band_reach():
            return False
        return not set(self._wanted(other)) & set(self.grants.nodes_of(m.name))

    def _clean_idle_meeting(self, m, job) -> None:
        """보낼 다리가 없어 선 청소 로봇이 운반 로봇의 남은 길 위에 있으면 미리 비킨다."""
        if job.stage not in (LINES, BACK):
            return
        here, reach = (m.x, m.y), self._band_reach()
        for other in self._robots.values():
            if (other is m or other.name in self._cleaners or not other.placed
                    or other.route is None or other.health.fault == OUT_OF_SERVICE):
                continue
            band = self._clean_band(other, full=True)
            if band is not None and _path_gap(here, band) <= reach:
                self._clean_make_way(m, other, f"waits on {other.name}'s way to "
                                     f"{other.route.goal}")
                return

    def _clean_yield_leg(self, m, other):
        """청소 격자에서 띠와 떨어진, 가는 길이 가장 짧은 점까지 A* 다리. 운반 로봇 앞을 가로지르면 쓰지 않는다."""
        plan, p, site = self._clean_plan, self.params, self.site
        here, pos = (m.x, m.y), (other.x, other.y)
        band = self._clean_band(other, full=True) or [pos]
        reach, ahead = self._band_reach(), self._band_ahead(band)
        # 가까운 순서는 곧은 거리가 아니라 청소 격자 위로 가는 거리다(진열대 건너편은 멀다)
        steps = plan.steps(here)
        js, iz = plan.free.nonzero()
        ranked = sorted((float(steps[j, i]), math.dist(here, q), q) for j, i in zip(js, iz)
                        if math.isfinite(steps[j, i])
                        for q in [(float(plan.grid.xs[i]), float(plan.grid.ys[j]))])
        spots = [q for _, _, q in ranked]
        rest = [o for o in self._robots.values() if o is not m and o is not other and o.placed]
        body, start_gap = 2 * p.footprint_radius_m, math.dist(here, pos)
        tries = 0
        for spot in spots:
            if _path_gap(spot, band) <= reach + p.clean_yield_margin_m or any(
                    math.dist(spot, (o.x, o.y)) <= p.clean_conflict_reach_m for o in rest):
                continue
            tries += 1
            if tries > YIELD_TRIES:
                return None
            try:
                sent, checked = plan.way(here, spot)
            except ValueError:
                continue
            if len(sent) < 2 or not plan.clear(checked):
                continue
            # 꺾인 운반 로봇 길은 첫 구간 뒤도 곧 지난다
            if len(ahead) >= 2 and any(segments_cross(a, b, c, d) for a, b in pairwise(sent)
                                       for c, d in pairwise(ahead)):
                continue
            if min(point_segment_distance(pos, a, b)
                   for a, b in pairwise(sent)) < min(body, start_gap) - 1e-6:
                continue
            if any(_path_gap((o.x, o.y), sent) <= p.clean_conflict_reach_m for o in rest):
                continue
            leg = self._leg(YIELD, "clean_yield", sent, None)
            if any(site.nodes[n].zone is None or site.zones[site.nodes[n].zone].kind != SHARED
                   for n in leg.nodes):
                continue
            # 남이 쥔 지점은 경로가 지금 자리보다 그 지점에 다가가면 막는다. 다만 비켜 주는
            # 그 로봇이 쥔 지점은 그 로봇이 지점에서 clean_conflict_ahead_m 밖이면 쥐지 않고 지난다
            held = [(n, self.grants.node_holder(n)) for n in leg.nodes]
            if any(h not in ("", m.name)
                   and (h != other.name or self._near_its_node(h, n))
                   and _path_gap(site.node_pose(n), sent) < math.dist(site.node_pose(n), here) - 1e-6
                   for n, h in held):
                continue
            return replace(leg, nodes=tuple(n for n, h in held if h in ("", m.name)))
        return None

    def _clean_backup_leg(self, m, other):
        """몸 방향 그대로 clean_backup_max_m 안에서 띠 밖(여유 포함)까지 후진하는 다리. 못 하면 None."""
        plan, p, site = self._clean_plan, self.params, self.site
        here, pos = (m.x, m.y), (other.x, other.y)
        band = self._clean_band(other, full=True) or [pos]
        need = self._band_reach() + p.clean_yield_margin_m
        back = (-math.cos(m.yaw), -math.sin(m.yaw))
        rest = [o for o in self._robots.values() if o is not m and o is not other and o.placed]
        step = plan.grid.res
        for n in range(1, math.floor(p.clean_backup_max_m / step + 1e-9) + 1):
            end = (here[0] + back[0] * n * step, here[1] + back[1] * n * step)
            if _path_gap(end, band) <= need:
                continue
            # 뒤가 막혔거나, 운반 로봇에 지금보다 다가가거나, 띠를 가로지르면 더 가도 같다
            if (not plan.clear([here, end])
                    or point_segment_distance(pos, here, end) < math.dist(here, pos) - 1e-6
                    or any(segments_cross(here, end, a, b) for a, b in pairwise(band))
                    or any(_path_gap((o.x, o.y), [here, end]) <= p.clean_conflict_reach_m
                           for o in rest)):
                return None
            leg = self._leg(YIELD, BACKUP, [here, end], None)
            if self._clean_off_floor(leg):
                return None
            held = [(n, self.grants.node_holder(n)) for n in leg.nodes]
            if any(h not in ("", m.name)
                   and _path_gap(site.node_pose(n), [here, end])
                   < math.dist(site.node_pose(n), here) - 1e-6 for n, h in held):
                return None
            return replace(leg, nodes=tuple(n for n, h in held if h in ("", m.name)))
        return None

    def _near_its_node(self, holder: str, node: str) -> bool:
        other = self._robots.get(holder)
        return other is None or (math.dist((other.x, other.y), self.site.node_pose(node))
                                 <= self.params.clean_conflict_ahead_m)

    # ---- 집 자리 피난(3-6): 돌아오기 다리로 입구까지 간 뒤 후진. 그 길도 막히면 세우고 경고 ----
    def _clean_go_home(self, m, job, why: str, leaving: str = "") -> bool:
        if job.goal_id is not None:
            self._clean_cancel(m, job)
        self._clean_requeue(m, job)
        job.homing, job.stage, job.back_nav = True, BACK, False
        job.yield_for, job.yield_since, job.tuck_for = "", None, ""
        job.escape_from = leaving
        self._event("WARN", "CLEAN_HOME", f"goes home to {job.home}: {why}", m.name)
        leg = self._clean_ready(m, self._back_leg(m, job, leaving), leaving)
        if leg is None:
            self._clean_note_stuck(m, job, self._clock())
            return False
        job.stuck_since, job.stuck_noted = None, False
        self._clean_send(m, job, leg)
        return True

    def _clean_note_stuck(self, m, job, now: float) -> None:
        if job.stuck_since is None:
            job.stuck_since = now
        if job.stuck_noted or now - job.stuck_since < self.params.clean_stuck_s:
            return
        job.stuck_noted = True
        job.stuck_count += 1
        self._event("WARN", "CLEAN_STUCK", f"cannot go home to {job.home} for "
                    f"{now - job.stuck_since:.0f} s, stands still ({job.stuck_count} times)",
                    m.name)

    # ---- 거둔 줄·우회를 진행점부터 바퀴 목록 맨 앞에 다시 둔다 ----
    def _clean_requeue(self, m, job) -> None:
        leg, d = job.leg, job.detour
        if leg is None or leg.kind not in (LINE, DETOUR):
            return
        job.leg, job.detour = None, None
        k, p = leg.line, self.params
        a, u, span = self._axis(job.segs[k])
        if leg.kind == DETOUR and d is not None:
            s = d.s0
        else:
            here = self._along(a, u, (m.x, m.y))
            off = abs((m.x - a[0]) * u[1] - (m.y - a[1]) * u[0])
            on_line = off <= p.arrival_tolerance_m / 2 and here >= -p.arrival_tolerance_m / 2
            s = min(max(here, 0.0), span) if on_line else 0.0
        if k in job.waiting:
            job.waiting.remove(k)
        if k in job.queue:
            job.queue.remove(k)
        if span - s < self._clean_plan.grid.res:
            job.done.append(k)
            self._event("INFO", "CLEAN_LINE_DONE", f"line {k} ({len(job.done)}/"
                        f"{len(job.segs)})", m.name)
            return
        if s > 1e-6:
            job.segs[k] = ((a[0] + u[0] * s, a[1] + u[1] * s), job.segs[k][1])
        job.queue.insert(0, k)

    def _held_against(self, m, node: str, pts, leaving: str) -> bool:
        """남이 쥔 지점이 다리를 막는가. 집으로 피하는 다리는 그 지점에 다가갈 때만 막힌다."""
        holder = self.grants.node_holder(node)
        if holder in ("", m.name) or not self._hold_counts(holder, node):
            return False
        return not leaving or self._nears(node, pts)

    def _hold_counts(self, holder: str, node: str) -> bool:
        """남이 쥔 지점을 볼 대상인가. ahead면 그 로봇 지금 위치에서 직선 clean_conflict_ahead_m 안만."""
        mode, other = self.params.clean_hold_check, self._robots.get(holder)
        if mode == OFF:
            return False
        if mode == ALL or other is None:
            return True
        gap = math.dist((other.x, other.y), self.site.node_pose(node))
        return gap <= self.params.clean_conflict_ahead_m

    def _base_near(self, other, pts, reach: float) -> bool:
        route, mode = other.route, self.params.clean_base_check
        if route is None or len(pts) < 2 or mode == OFF:
            return False
        edges = route.edges[route.done:route.granted]
        if mode == AHEAD:
            # 지금 위치에서 base를 따라 clean_conflict_ahead_m까지만 본다
            near = _ahead((other.x, other.y), edges, self.params.clean_conflict_ahead_m)
            return any(_segments_gap(a, b, c, d) <= reach
                       for c, d in pairwise(near) for a, b in pairwise(pts))
        for edge in edges:
            for c, d in pairwise(edge.points):
                if any(_segments_gap(a, b, c, d) <= reach for a, b in pairwise(pts)):
                    return True
        return False

    def _base_line(self, other) -> list:
        """받은 구간(base)을 이은 꺾은선. clean_base_check가 off거나 경로가 없으면 빈 목록."""
        route = other.route
        if route is None or self.params.clean_base_check == OFF:
            return []
        pts = []
        for edge in route.edges[route.done:route.granted]:
            for q in edge.points:
                if not pts or math.dist(pts[-1], q) > 1e-9:
                    pts.append(tuple(q))
        return pts

    def _nears(self, node: str, pts) -> bool:
        """다리가 출발 자리보다 그 지점에 다가가는가."""
        pose = self.site.node_pose(node)
        return _path_gap(pose, pts) < math.dist(pose, pts[0]) - 1e-6

    def _clean_take(self, m, leg) -> str:
        floor = self._clean_plan.floor
        if self.grants.take(m.name, [floor], list(leg.nodes)):
            return ""
        self.grants.stop_waiting(m.name)
        return f"cannot take {floor} or {list(leg.nodes)}"

    # ---- 미루기와 포기 ----
    def _clean_defer(self, m, job, k: int, why: str) -> None:
        count = job.deferrals.get(k, 0) + 1
        job.deferrals[k] = count
        job.retry_at = self._clock() + self.params.clean_retry_s
        if count > self.params.clean_defer_limit:
            job.skipped.append(k)
            self._event("WARN", "CLEAN_LINE_SKIPPED", f"line {k} gives up this lap after "
                        f"{count - 1} deferrals: {why}", m.name)
            return
        job.waiting.append(k)
        self._event("INFO", "CLEAN_LINE_DEFERRED", f"line {k} ({count}): {why}", m.name)

    # ---- 보내기 ----
    def _clean_send(self, m, job, leg) -> None:
        goal_id = self._next_goal_id()
        job.tuck, job.tuck_line = None, -1
        if leg.kind == BAY_IN or leg.target == BACKUP:
            if leg.kind == BAY_IN:
                edge = self._clean_plan.in_edge
            else:
                pts = tuple(leg.points)
                edge = Edge("", BACKUP, RETREAT, "", pts, (self._clean_plan.floor,),
                            math.dist(pts[0], pts[-1]))
            self._out.append(retreat_command(self.params, m.name, goal_id, [edge], m.x, m.y))
            mode = RETREAT
        elif leg.kind in (DETOUR, BACK_NAV):
            (x, y), yaw = leg.points[-1], leg.end_yaw
            # controller는 goal마다 고르지 않는다(청소 로봇 Nav2 설정이 정한다)
            self._out.append(MoveCmd(m.name, goal_id, leg.target, x, y, yaw, (), True, NAV_GOAL,
                                     self._goal_priority(m)))
            mode = NAV_GOAL
        else:
            pts = leg.points
            yaws = [heading(a, b) for a, b in pairwise(pts)]
            end_yaw = leg.end_yaw if leg.end_yaw is not None else (yaws[-1] if yaws else m.yaw)
            path = tuple((x, y, yaw) for (x, y), yaw in zip(pts, [*yaws, end_yaw], strict=True))
            self._out.append(MoveCmd(m.name, goal_id, leg.target, pts[-1][0], pts[-1][1],
                                     end_yaw, path, True, FOLLOW_PATH, self._goal_priority(m)))
            mode = FOLLOW_PATH
        self._set_goal(m, Goal(CLEAN, goal_id, 0, leg.target, mode))
        job.goal_id, job.leg, job.blocked_since, job.arrival_by = goal_id, leg, None, None
        m.phase = CLEANING
        what = f"{leg.kind} {leg.line}" if leg.kind in (LINE, DETOUR) else leg.kind
        self._event("INFO", "CLEAN_SENT", f"{what} to {leg.target}, holds {list(leg.nodes)}",
                    m.name)

    # ---- 결과 ----
    def _clean_result(self, m, goal_id, success: bool, reason: str) -> None:
        job = self._cleaners[m.name]
        if not self._live(m, job, goal_id) or job.arrival_by is not None:
            return
        far = math.dist((m.x, m.y), job.leg.points[-1]) > self.params.arrival_tolerance_m
        if success and far:
            # 운반 로봇과 같다: 끝에서 먼 성공은 도착 보고를 arrival_settle_s 기다린다
            job.arrival_by = self._clock() + self.params.arrival_settle_s
            m.heard_since_success = False
            return
        self._clean_end(m, job, success, reason)

    def _clean_arrival(self, m, job, now: float) -> None:
        """성공 뒤 도착 기다림: 끝 둘레에서 새 보고가 오면 끝내고, 시간이 지나면 실패로 본다."""
        if (m.heard_since_success
                and math.dist((m.x, m.y), job.leg.points[-1]) <= self.params.arrival_tolerance_m):
            self._clean_end(m, job, True, "")
        elif now >= job.arrival_by:
            self._clean_end(m, job, False, NOT_AT_TARGET)

    def _clean_end(self, m, job, success: bool, reason: str) -> None:
        leg = job.leg
        self._clear_goal(m)
        job.goal_id, job.arrival_by = None, None
        self._clean_release(m, job, ended=True)
        if success:
            job.leg = None
            m.attempts = 0
            m.not_sent_warned = False
            self._clean_leg_done(m, job, leg)
        elif reason == HEARTBEAT_LOST:
            m.retry_at = self._clock() + self.params.move_retry_wait_s
        elif leg.kind in (LINE, DETOUR):
            job.leg = None
            if leg.kind == DETOUR:
                job.detour = None
                self._event("WARN", "CLEAN_DETOUR_FAILED", f"line {leg.line}: {reason}", m.name)
            self._clean_defer(m, job, leg.line, reason or "failed")
        else:
            self._goal_failed(m, reason or "failed")

    def _clean_leg_done(self, m, job, leg) -> None:
        if leg.kind == OUT:
            job.stage = LINES
        elif leg.kind == LINE:
            job.done.append(leg.line)
            self._event("INFO", "CLEAN_LINE_DONE", f"line {leg.line} ({len(job.done)}/"
                        f"{len(job.segs)})", m.name)
        elif leg.kind == DETOUR:
            self._clean_rejoin(m, job, leg)
        elif leg.kind == BACK_NAV:
            job.back_nav = False
        elif leg.kind == YIELD:
            return
        elif leg.kind == BACK and self._clean_plan.in_edge is not None:
            job.stage = BAY_IN
        elif job.homing and (job.queue or job.waiting or job.skipped or job.gaps):
            self._clean_refuge(m, job)
        else:
            job.homing = False
            self._clean_lap_done(m, job)

    def _clean_refuge(self, m, job) -> None:
        """피난으로 집 자리에 닿음: 바퀴는 끝내지 않고 남은 줄을 쥔 채 잠시 쉰다."""
        job.homing, job.refuge = False, True
        job.stage = HOME
        job.home_until = self._clock() + self.params.clean_yield_wait_s
        m.node, m.off_node = job.home, None
        m.phase = PARKED
        self._event("INFO", "CLEAN_AT_HOME", f"waits at {job.home}, "
                    f"{len(job.queue) + len(job.waiting)} lines left in lap {job.lap}", m.name)

    def _clean_lap_done(self, m, job) -> None:
        job.stage = HOME
        m.node, m.off_node = job.home, None
        m.phase = PARKED
        self._clean_lap_report(m, job, f"at {job.home}")

    def _clean_lap_report(self, m, job, where: str) -> None:
        plan = self._clean_plan
        deferred = sum(job.deferrals.values())
        self._event("INFO", "CLEAN_LAP_DONE",
                    f"lap {job.lap} in {self._clock() - job.lap_since:.1f} s: "
                    f"{len(job.done)}/{len(job.segs)} lines, {deferred} deferrals, "
                    f"skipped {sorted(job.skipped)}, {len(job.gaps)} gaps, covered "
                    f"{plan.covered([job.segs[k] for k in job.done]):.3f} of the clean grid, "
                    f"{where}", m.name)

    def _live(self, m, job, goal_id) -> bool:
        """지금 달리는 그 goal의 소식인가. 거둔 goal의 늦은 소식은 버린다."""
        return job.goal_id == goal_id and m.goal is not None and m.goal.id == goal_id

    def _clean_rejected(self, m, goal_id, reason: str) -> None:
        job = self._cleaners[m.name]
        if not self._live(m, job, goal_id):
            return
        self._clear_goal(m)
        job.goal_id = None
        self._clean_release(m, job, ended=True)
        self._goal_failed(m, reason)

    def _clean_not_sent(self, m, goal_id) -> None:
        job = self._cleaners[m.name]
        if not self._live(m, job, goal_id):
            return
        self._clear_goal(m)
        job.goal_id = None
        self._clean_release(m, job, ended=True)
        m.retry_at = self._clock() + self.params.move_retry_wait_s
        if not m.not_sent_warned:
            m.not_sent_warned = True
            self._event("WARN", "GOAL_NOT_SENT", "the action server is not ready, sends it again",
                        m.name)

    def _clean_lost(self, m, goal_id) -> None:
        job = self._cleaners[m.name]
        if self._live(m, job, goal_id):
            self._clear_goal(m)
            job.goal_id = None
            self._clean_release(m, job, ended=True)

    # ---- 상태 보고: 지나간 지점 놓기, 우회 위치 기록, BLOCKED 기다림 ----
    def _clean_report(self, m, now: float) -> None:
        job = self._cleaners[m.name]
        # 바깥에서 거둔 goal(STOP, 멈춤 감시)은 도착 판정과 막힘 처리에 쓰지 않는다
        self._clean_reap(m, job)
        self._clean_release(m, job, ended=False)
        self._watch_outside(m)
        leg = job.leg
        if (job.detour is not None and leg is not None and leg.kind == DETOUR
                and job.detour.trail[-1] != (m.x, m.y)):
            job.detour.trail.append((m.x, m.y))
        if job.arrival_by is not None:
            self._clean_arrival(m, job, now)
            return
        # BLOCKED 보고 때부터 센다. 스스로 비키는 중(RECOVERING)이면 세지 않는다
        if m.report.robot_state != BLOCKED or job.goal_id is None or not self.running:
            job.blocked_since = job.blocked_at = None
            return
        s = self._clean_progress(m, job)
        if job.blocked_since is None:
            job.blocked_since, job.blocked_at = now, s
            self._event("INFO", "BLOCKED_WAIT", f"stands before something on the way to "
                        f"{leg.target}", m.name)
        elif s is not None and job.blocked_at is not None and s - job.blocked_at > PROGRESS_M:
            # 보고만 잠시 남은 것(hold): 진행점이 나아갔으니 기다림을 끝낸다
            job.blocked_since, job.blocked_at = now, s
            return
        if now - job.blocked_since < self.params.clean_blocked_s:
            return
        self._clean_cancel(m, job)
        why = f"blocked for {self.params.clean_blocked_s:.0f} s"
        if leg.kind == LINE:
            job.leg = None
            self._clean_detour(m, job, leg.line, why)
        elif leg.kind == DETOUR:
            job.leg = None
            self._clean_detour_again(m, job, why)
        else:
            job.back_nav = leg.kind in (BACK, BACK_NAV)
            m.retry_at = now + self.params.move_retry_wait_s

    def _clean_cancel(self, m, job) -> None:
        """지금 goal(줄이든 우회든)을 거둔다. 늦게 온 결과는 goal 번호로 버려진다."""
        self._cancel_goal(m)
        job.goal_id, job.arrival_by = None, None
        job.blocked_since, job.blocked_at = None, None
        self._clean_release(m, job, ended=True)

    # ---- 줄 위 진행점: 줄 축에 내린 거리 ----
    @staticmethod
    def _axis(seg):
        a, b = seg
        span = math.dist(a, b)
        u = ((b[0] - a[0]) / span, (b[1] - a[1]) / span) if span > 1e-6 else (1.0, 0.0)
        return a, u, span

    @staticmethod
    def _along(a, u, q) -> float:
        return (q[0] - a[0]) * u[0] + (q[1] - a[1]) * u[1]

    def _clean_progress(self, m, job):
        leg = job.leg
        if leg is None or leg.kind not in (LINE, DETOUR):
            return None
        a, u, span = self._axis(job.segs[leg.line])
        return min(max(self._along(a, u, (m.x, m.y)), 0.0), span)

    # ---- 우회: 사람·물건이면 줄 앞쪽 복귀점까지 Nav2 goal, 운반 로봇이면 미룬다 ----
    def _clean_detour(self, m, job, k: int, why: str) -> None:
        p = self.params
        a, u, span = self._axis(job.segs[k])
        if span <= 1e-6:
            self._clean_defer(m, job, k, why)
            return
        here = self._along(a, u, (m.x, m.y))
        off = abs((m.x - a[0]) * u[1] - (m.y - a[1]) * u[0])
        s = min(max(here, 0.0), span)
        seen = m.report.obstacle
        if seen:
            point = tuple(seen[:2])
            s_obs = max(self._along(a, u, point), s)
        else:
            s_obs = s + p.clean_block_look_m
            point = (a[0] + u[0] * s_obs, a[1] + u[1] * s_obs)
        for other in self._robots.values():
            if (other is not m and other.name not in self._cleaners and other.placed
                    and math.dist(point, (other.x, other.y)) <= p.near_m):
                self._clean_defer(m, job, k, f"{why} by {other.name}")
                return
        # 줄에 닿기 전에 막히면 줄 시작점까지 돌아간다(못 쓴 칸 없음)
        if off > p.arrival_tolerance_m / 2 or here < -p.arrival_tolerance_m / 2:
            self._clean_go_around(m, job, k, None, 0.0, point, why)
            return
        self._clean_go_around(m, job, k, s, s_obs + p.footprint_radius_m
                              + p.clean_detour_margin_m, point, why)

    def _clean_detour_again(self, m, job, why: str) -> None:
        """복귀점 전에 다시 막히면 복귀점을 한 번 더 앞으로 옮기고, 그래도 막히면 미룬다."""
        d, p = job.detour, self.params
        if d is None or d.moves >= 1 or d.s_back <= d.s0:
            k = d.line if d is not None else job.leg.line
            job.detour = None
            self._event("WARN", "CLEAN_DETOUR_FAILED", f"line {k}: {why} again", m.name)
            self._clean_defer(m, job, k, why)
            return
        self._clean_go_around(m, job, d.line, d.s0, d.s_back + p.footprint_radius_m
                              + p.clean_detour_margin_m, None, why, d)

    def _clean_go_around(self, m, job, k, s0, s_from, point, why, last=None) -> None:
        p = self.params
        a, u, span = self._axis(job.segs[k])
        s_back = self._return_s(a, u, span, s_from)
        if s_back is None:
            job.detour = None
            self._event("WARN", "CLEAN_DETOUR_FAILED", f"line {k}: no free return point", m.name)
            self._clean_defer(m, job, k, why)
            return
        back = (a[0] + u[0] * s_back, a[1] + u[1] * s_back)
        if s0 is None:
            s0 = s_back
        start = (a[0] + u[0] * s0, a[1] + u[1] * s0)
        leg = self._leg(DETOUR, f"clean_detour_{k}", [(m.x, m.y), back], math.atan2(u[1], u[0]),
                        k)
        half = p.footprint_radius_m + p.clean_detour_margin_m / 2
        strip = [(m.x, m.y), start, back]
        leg = replace(leg, nodes=tuple(dict.fromkeys(
            (*leg.nodes, *(name for name in self.site.nodes
                           if _path_gap(self.site.node_pose(name), strip) <= half)))))
        refusal = self._strip_refusal(strip, half)
        if not refusal:
            leg, refusal = self._clean_admit(m, leg)
        if refusal:
            job.detour = None
            self._event("WARN", "CLEAN_DETOUR_FAILED", f"line {k}: {refusal}", m.name)
            self._clean_defer(m, job, k, why)
            return
        trail = last.trail if last is not None else [(m.x, m.y)]
        job.detour = Detour(k, start, back, s0, s_back, last.moves + 1 if last else 0, trail)
        around = f"around ({point[0]:.2f}, {point[1]:.2f})" if point else "further"
        self._event("INFO", "CLEAN_DETOUR", f"line {k} {why}: {around} to ({back[0]:.3f}, "
                    f"{back[1]:.3f})", m.name)
        self._clean_send(m, job, leg)

    def _strip_refusal(self, strip, half: float) -> str:
        """우회 띠가 dock·lane 구역이나 금지 구역에 닿으면 그 까닭."""
        for rect in keepout_rects(self.site, 0.0):
            for c, d in pairwise(strip):
                if segment_rect_distance(c, d, rect) <= half:
                    return f"the detour strip touches the keepout {rect}"
        return ""

    def _return_s(self, a, u, span: float, s: float):
        """줄 위 s부터 끝까지 격자 칸 간격으로 걸으며 처음 만나는 청소 격자 빈칸까지 거리."""
        plan = self._clean_plan
        while s <= span + 1e-9:
            cell = plan.grid.cell((a[0] + u[0] * s, a[1] + u[1] * s))
            if cell is not None and plan.free[cell[1], cell[0]]:
                return s
            s += plan.grid.res
        return None

    # ---- 우회가 끝나면 덮이지 않은 줄 칸만 빈틈으로 적고 복귀점부터 남은 줄을 잇는다 ----
    def _clean_rejoin(self, m, job, leg) -> None:
        d, job.detour = job.detour, None
        k = leg.line
        if d is not None:
            self._clean_note_gaps(m, job, k, d)
        a, b = job.segs[k]
        here = d.back if d is not None else a
        if math.dist(here, b) <= 1e-3:
            job.done.append(k)
            self._event("INFO", "CLEAN_LINE_DONE", f"line {k} ({len(job.done)}/"
                        f"{len(job.segs)})", m.name)
            return
        job.segs[k] = (here, b)
        job.queue.insert(0, k)

    def _clean_note_gaps(self, m, job, k: int, d) -> None:
        """줄 k에서 건너뛴 구간(d.s0~d.s_back) 가운데 덜 닦인 곳을 다음 바퀴의 빈틈으로 남긴다."""
        if d.s_back <= d.s0:
            return
        for gap in self._uncovered(job.segs[k], d):
            job.gaps.append(gap)
            (x0, y0), (x1, y1) = gap
            self._event("INFO", "CLEAN_GAP", f"line {k} ({x0:.3f}, {y0:.3f})-({x1:.3f}, "
                        f"{y1:.3f}), {math.dist(*gap):.2f} m left for the next lap", m.name)

    def _uncovered(self, seg, d) -> list:
        """진행점~복귀점 줄 칸 가운데 받은 위치의 청소 폭에 안 든 칸을 이은 구간들."""
        plan = self._clean_plan
        a, u, _ = self._axis(seg)
        half, step = plan.params.clean_width_m / 2, plan.grid.res
        n = max(1, math.ceil((d.s_back - d.s0) / step - 1e-9))
        runs, lo = [], None
        for i in range(n + 1):
            s = min(d.s0 + i * step, d.s_back)
            q = (a[0] + u[0] * s, a[1] + u[1] * s)
            if not d.trail or _path_gap(q, d.trail) > half:
                lo = s if lo is None else lo
                hi = s
            elif lo is not None:
                runs.append((lo, hi))
                lo = None
        if lo is not None:
            runs.append((lo, hi))
        out = []
        for lo, hi in runs:
            lo, hi = max(lo - step / 2, d.s0), min(hi + step / 2, d.s_back)
            out.append(((a[0] + u[0] * lo, a[1] + u[1] * lo), (a[0] + u[0] * hi, a[1] + u[1] * hi)))
        return out

    def _clean_release(self, m, job, ended: bool) -> None:
        """몸이 둘레에 든 지점은 기록하고, 들었다 나온 지점은 놓는다. goal이 끝나면 둘레 밖은 모두 놓는다.
        그 지점 둘레를 지나야만 나갈 수 있는 막다른 자리에서는 놓지 않는다(놓으면 남이 입구에 선다)."""
        reach, here, plan = self.params.clean_reach_m, (m.x, m.y), self._clean_plan
        for node in self.grants.nodes_of(m.name):
            if self._gap(m, node) <= reach:
                job.entered.add(node)
            elif node in job.entered or ended:
                if plan is not None and plan.shut_in(here, self.site.node_pose(node), reach):
                    # 막다른 자리를 벗어난 첫 보고에서 놓는다
                    job.entered.add(node)
                    continue
                self.grants.release_node(m.name, node)
                job.entered.discard(node)

    # ---- 조회 ----
    def clean_view(self, robot) -> dict:
        job = self._cleaners.get(robot)
        if job is None:
            return {}
        return {"lap": job.lap, "stage": job.stage, "line": job.leg.line if job.leg else -1,
                "done": list(job.done), "deferred": sorted(job.waiting),
                "skipped": sorted(job.skipped), "lines": len(job.segs), "gaps": len(job.gaps),
                "yields": job.yields, "stuck": job.stuck_count}
