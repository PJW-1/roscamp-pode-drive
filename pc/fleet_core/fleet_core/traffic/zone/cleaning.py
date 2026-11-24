# ---- 청소 로봇 실행부: 줄을 하나씩 보내고 점유와 바퀴를 맡는 믹스인 ----
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from itertools import pairwise

from fleet_core.common.commands import MoveCmd
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, SHARED
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
LINE, DETOUR, BACK_NAV = "line", "detour", "back_nav"
PROGRESS_M = 0.02  # BLOCKED 기다림 중 진행점이 이만큼 앞으로 가면 기다림을 끝낸다


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


# ---- 청소 경로 자료: 단계 1의 커버리지 계획과 집 자리 고정 경로 ----
class CleanPlan:
    def __init__(self, site):
        from fleet_core import coverage
        self.cov = coverage
        plan = coverage.make_plan(site)
        self.params = plan.params
        self.grid = plan.grid
        self.free = plan.grid.free
        self.lines = [(a, b) for a, b, _ in plan.lines]
        self.start = plan.start
        self.home = coverage.HOME
        self.floor = coverage.FLOOR_ZONE
        self.radius = plan.params.turn_radius_m
        edges = list(site.edges())
        self.out_edge = next((e for e in edges if e.a == self.home and e.mode == FOLLOW_PATH),
                             None)
        self.in_edge = next((e for e in edges if e.b == self.home and e.mode == RETREAT), None)

    # ---- 지금 자리에서 점까지 청소 격자 위 꺾은선: (보낼 점들, 검사할 점들) ----
    def way(self, here, to) -> tuple[list, list]:
        cell = self.grid.cell(here)
        inside = cell is not None and self.free[cell[1], cell[0]]
        first = here if inside else self.grid.centre(self.grid.nearest_free(here))
        pts = self.cov.transit(self.grid, self.free, first, to, self.radius)
        return ([] if inside else [here]) + pts, pts

    def clear(self, pts) -> bool:
        return self.cov.path_clear(self.grid, self.cov.rounded(pts, self.radius))

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
        if job.goal_id is not None:
            if m.goal is not None and m.goal.id == job.goal_id:
                if job.arrival_by is not None:
                    self._clean_arrival(m, job, now)
                return
            # 결과 없이 거둔 goal(멈춤, STOP, 통신 복구): 같은 다리를 다시 보낸다
            job.goal_id, job.arrival_by = None, None
            self._clean_release(m, job, ended=True)
        if m.goal is not None or now < m.retry_at:
            return
        if job.stage == HOME and not self._clean_start_lap(m, job, now):
            return
        leg = self._clean_next(m, job, now)
        if leg is not None:
            self._clean_send(m, job, leg)

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
            # 결과 없이 끝난 줄·우회(유실, 못 보냄, 거절, heartbeat 끊김)는 그 줄을 맨 앞에 다시 둔다
            if job.leg is not None and job.leg.kind in (LINE, DETOUR):
                job.queue.insert(0, job.leg.line)
                job.leg, job.detour = None, None
            leg = self._clean_line(m, job, now)
            if leg is not None or job.stage != BACK:
                return leg
        if job.stage == BACK:
            return self._clean_ready(m, self._back_leg(m, job))
        if job.stage == BAY_IN:
            return self._clean_ready(m, self._bay_leg(m))
        return None

    def _clean_line(self, m, job, now: float):
        while True:
            if not job.queue:
                if not job.waiting:
                    job.stage = BACK
                    return None
                if now < job.retry_at:
                    return None
                job.queue, job.waiting = sorted(job.waiting), []
            k = job.queue.pop(0)
            leg = self._line_leg(m, job, k)
            why = "the joined path is not clear after rounding" if leg is None else (
                self._clean_conflict(m, leg) or self._clean_take(m, leg))
            if not why:
                return leg
            self._clean_defer(m, job, k, why)

    def _clean_ready(self, m, leg):
        return None if self._clean_conflict(m, leg) or self._clean_take(m, leg) else leg

    # ---- 다리 만들기 ----
    def _line_leg(self, m, job, k: int):
        plan = self._clean_plan
        a, b = job.segs[k]
        sent, checked = plan.way((m.x, m.y), a)
        if not plan.clear([*checked, b]):
            return None
        return self._leg(LINE, f"clean_line_{k}", [*sent, b], None, k)

    def _out_leg(self, m):
        plan = self._clean_plan
        pts = [(m.x, m.y), *plan.out_edge.points[1:], plan.start]
        return self._leg(OUT, "clean_start", pts, None)

    def _back_leg(self, m, job):
        plan, site = self._clean_plan, self.site
        # 돌아오는 길이 막혔으면 출발점까지 Nav2 goal로 돌아간다
        if job.back_nav and math.dist((m.x, m.y), plan.start) > self.params.arrival_tolerance_m:
            end = site.node_pose(plan.in_edge.a) if plan.in_edge else site.node_pose(plan.home)
            return self._leg(BACK_NAV, "clean_start", [(m.x, m.y), plan.start],
                             heading(plan.start, end) if plan.start != end else 0.0)
        if plan.in_edge is None:
            sent, _ = plan.way((m.x, m.y), site.node_pose(plan.home))
            return self._leg(BACK, plan.home, sent, site.nodes[plan.home].yaw)
        entry = plan.in_edge.a
        sent, _ = plan.way((m.x, m.y), plan.start)
        return self._leg(BACK, entry, [*sent, site.node_pose(entry)], site.nodes[entry].yaw)

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

    # ---- 겹침 검사: 막힌 구역, 남이 쥔 지점, 남의 자리와 base ----
    def _clean_conflict(self, m, leg) -> str:
        site, grants, reach = self.site, self.grants, self.params.clean_reach_m
        for node in leg.nodes:
            zone = site.nodes[node].zone
            if zone is None or site.zones[zone].kind != SHARED:
                return f"{node} is in {zone}, not a shared zone"
            holder = grants.node_holder(node)
            if holder not in ("", m.name):
                return f"{node} is held by {holder}"
        for other in self._robots.values():
            if other is m or not other.placed or other.health.fault == OUT_OF_SERVICE:
                continue
            if _path_gap((other.x, other.y), leg.points) <= reach:
                return f"{other.name} stands near"
            if self._base_near(other, leg.points, reach):
                return f"{other.name}'s base passes near"
        return ""

    @staticmethod
    def _base_near(other, pts, reach: float) -> bool:
        route = other.route
        if route is None or len(pts) < 2:
            return False
        for edge in route.edges[route.done:route.granted]:
            for c, d in pairwise(edge.points):
                if any(_segments_gap(a, b, c, d) <= reach for a, b in pairwise(pts)):
                    return True
        return False

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
        if leg.kind == BAY_IN:
            edge = self._clean_plan.in_edge
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
        elif leg.kind == BACK and self._clean_plan.in_edge is not None:
            job.stage = BAY_IN
        else:
            self._clean_lap_done(m, job)

    def _clean_lap_done(self, m, job) -> None:
        plan = self._clean_plan
        job.stage = HOME
        m.node, m.off_node = job.home, None
        m.phase = PARKED
        deferred = sum(job.deferrals.values())
        self._event("INFO", "CLEAN_LAP_DONE",
                    f"lap {job.lap} in {self._clock() - job.lap_since:.1f} s: "
                    f"{len(job.done)}/{len(job.segs)} lines, {deferred} deferrals, "
                    f"skipped {sorted(job.skipped)}, {len(job.gaps)} gaps, covered "
                    f"{plan.covered([job.segs[k] for k in job.done]):.3f} of the clean grid",
                    m.name)

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
        self._clean_release(m, job, ended=False)
        self._watch_outside(m)
        leg = job.leg
        if (job.detour is not None and leg is not None and leg.kind == DETOUR
                and job.detour.trail[-1] != (m.x, m.y)):
            job.detour.trail.append((m.x, m.y))
        if job.arrival_by is not None:
            if job.goal_id is not None:
                self._clean_arrival(m, job, now)
            return
        # BLOCKED 보고 때부터 센다. 스스로 비키는 중(RECOVERING)이면 세지 않는다
        if m.report.robot_state != BLOCKED or job.goal_id is None:
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
        job.goal_id, job.blocked_since, job.blocked_at = None, None, None
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
        refusal = (self._strip_refusal(strip, half) or self._clean_conflict(m, leg)
                   or self._clean_take(m, leg))
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
        if d is not None and d.s_back > d.s0:
            for gap in self._uncovered(job.segs[k], d):
                job.gaps.append(gap)
                (x0, y0), (x1, y1) = gap
                self._event("INFO", "CLEAN_GAP", f"line {k} ({x0:.3f}, {y0:.3f})-({x1:.3f}, "
                            f"{y1:.3f}), {math.dist(*gap):.2f} m left for the next lap", m.name)
        a, b = job.segs[k]
        here = d.back if d is not None else a
        if math.dist(here, b) <= 1e-3:
            job.done.append(k)
            self._event("INFO", "CLEAN_LINE_DONE", f"line {k} ({len(job.done)}/"
                        f"{len(job.segs)})", m.name)
            return
        job.segs[k] = (here, b)
        job.queue.insert(0, k)

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
        """몸이 둘레에 든 지점은 기록하고, 들었다 나온 지점은 놓는다. goal이 끝나면 둘레 밖은 모두 놓는다."""
        reach = self.params.clean_reach_m
        for node in self.grants.nodes_of(m.name):
            if self._gap(m, node) <= reach:
                job.entered.add(node)
            elif node in job.entered or ended:
                self.grants.release_node(m.name, node)
                job.entered.discard(node)

    # ---- 조회 ----
    def clean_view(self, robot) -> dict:
        job = self._cleaners.get(robot)
        if job is None:
            return {}
        return {"lap": job.lap, "stage": job.stage, "line": job.leg.line if job.leg else -1,
                "done": list(job.done), "deferred": sorted(job.waiting),
                "skipped": sorted(job.skipped), "lines": len(job.segs), "gaps": len(job.gaps)}
