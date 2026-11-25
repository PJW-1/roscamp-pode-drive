"""시뮬레이터 없이 쓰는 청소 로봇 칸: 실제 청소 로봇(fleet_core coverage, zone/cleaning)의 줄 바퀴.

줄과 이동 구간은 feat/cleaner-yield-2026-11-25 의 pc/fleet_core/fleet_core/coverage.py 계산을
그대로 옮겼다 (청소 격자, boustrophedon 셀, 셀마다 줄 고르기, A* 이동, 모서리 둥글게).
바퀴는 실제 순서대로 bay_out_1 (P-1 -> E-1) -> 출발점 -> 줄마다 (이동 + 줄) -> 출발점 -> E-1
-> bay_in_1 (P-1) 이다.
바퀴를 이어 붙여 robot_spec 경로 dict (꼭짓점 시간 폴리라인) 로 깔고,
꼭짓점 노드는 -1 (노드 예약 밖) 이다.
"""

import heapq
import math
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise

import numpy as np
import robot_spec as rbs  # arena_spec 가 fleet_core 경로를 넣음
import torch
from fleet_core.fleet_map.fleet_map import TURN_RADIUS_M  # noqa: E402

# ---- 기본 파라미터: coverage.py 와 같음 ----
GRID_M = 0.025
ROBOT_RADIUS_M = 0.083
MARGIN_M = 0.01
LINE_SPACING_M = 0.10
CLEAN_WIDTH_M = 0.112
SPEED_MPS = 0.12
ARC_STEP_M = 0.01
HOME = "P-1"
FLOOR_ZONE = "HALL"
_SAME_M = 1e-6
_EPS = 1e-9

Point = tuple[float, float]
Cell = tuple[int, int]  # (i, j) = (x 칸, y 칸)


@dataclass(frozen=True)
class Params:
    grid_m: float = GRID_M
    robot_radius_m: float = ROBOT_RADIUS_M
    margin_m: float = MARGIN_M
    line_spacing_m: float = LINE_SPACING_M
    clean_width_m: float = CLEAN_WIDTH_M
    turn_radius_m: float = TURN_RADIUS_M
    speed_mps: float = SPEED_MPS

    def __post_init__(self):
        if self.line_spacing_m < self.grid_m:
            raise ValueError(f"line spacing {self.line_spacing_m} m is below the grid {self.grid_m} m")


# ---- 모서리 둥글게 바꾸기: pode_fsm geometry.rounded와 같은 계산 ----
def _dedup(points: Sequence[Point]) -> list[Point]:
    out: list[Point] = []
    for p in points:
        p = (float(p[0]), float(p[1]))
        if not out or math.dist(out[-1], p) > _SAME_M:
            out.append(p)
    return out


def _corner_arc(a: Point, b: Point, c: Point, room_in: float, room_out: float,
                radius: float, step: float) -> list[Point]:
    h1 = math.atan2(b[1] - a[1], b[0] - a[0])
    h2 = math.atan2(c[1] - b[1], c[0] - b[0])
    turn = math.remainder(h2 - h1, math.tau)
    if abs(turn) < 1e-3 or abs(turn) > math.pi - 1e-3:
        return [b]
    half = math.tan(abs(turn) / 2.0)
    cut = min(radius * half, room_in, room_out)
    r = cut / half
    start = (b[0] - cut * math.cos(h1), b[1] - cut * math.sin(h1))
    side = 1.0 if turn > 0 else -1.0
    centre = (start[0] - side * r * math.sin(h1), start[1] + side * r * math.cos(h1))
    a0 = math.atan2(start[1] - centre[1], start[0] - centre[0])
    n = max(2, math.ceil(r * abs(turn) / step))
    return [(centre[0] + r * math.cos(a0 + turn * i / n),
             centre[1] + r * math.sin(a0 + turn * i / n)) for i in range(n + 1)]


def rounded(points: Sequence[Point], radius: float, step: float = ARC_STEP_M) -> list[Point]:
    pts = _dedup(points)
    if len(pts) < 3:
        return pts
    last = len(pts) - 2
    lengths = [math.dist(p, q) for p, q in pairwise(pts)]
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        room_in = lengths[i - 1] if i - 1 == 0 else lengths[i - 1] / 2.0
        room_out = lengths[i] if i == last else lengths[i] / 2.0
        arc = [pts[i]]
        if radius > 0.0:
            arc = _corner_arc(pts[i - 1], pts[i], pts[i + 1], room_in, room_out, radius, step)
        out.extend(arc)
    out.append(pts[-1])
    return _dedup(out)


def path_length(points: Sequence[Point]) -> float:
    return sum(math.dist(p, q) for p, q in pairwise(points))


# ---- 청소 격자: HALL만 바닥, 막힌 것은 (반지름 + 여유)만큼 키움 ----
def _rect_distance(rect, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    dx = np.maximum(np.maximum(rect[0] - xs, 0.0), xs - rect[2])
    dy = np.maximum(np.maximum(rect[1] - ys, 0.0), ys - rect[3])
    return np.hypot(dx, dy)


def _inside(rect, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    return ((rect[0] - _EPS <= xs) & (xs <= rect[2] + _EPS)
            & (rect[1] - _EPS <= ys) & (ys <= rect[3] + _EPS))


class CleanGrid:
    def __init__(self, site, params: Params):
        self.res = params.grid_m
        self.reach = params.robot_radius_m + params.margin_m
        x0, y0, x1, y1 = site.bounds()
        self.x0, self.y0 = x0, y0
        self.box = (x0, y0, x1, y1)
        self.w = math.ceil((x1 - x0) / self.res - _EPS)
        self.h = math.ceil((y1 - y0) / self.res - _EPS)
        self.xs = x0 + (np.arange(self.w) + 0.5) * self.res
        self.ys = y0 + (np.arange(self.h) + 0.5) * self.res
        xs, ys = np.meshgrid(self.xs, self.ys)
        # 바닥: HALL 사각형의 합
        hall = np.zeros_like(xs, dtype=bool)
        for rect in site.zones[FLOOR_ZONE].rects:
            hall |= _inside(rect, xs, ys)
        shelves = list(dict.fromkeys((*site.shelves, *site.sim_shelves)))
        blocked = [rect for zone in sorted(site.zones.values(), key=lambda z: z.id)
                   if zone.id != FLOOR_ZONE for rect in zone.rects]
        blocked += list(site.keepout_extra) + shelves
        self.blocked = tuple(blocked)
        self.hall_rects = tuple(site.zones[FLOOR_ZONE].rects)
        # 키운 격자: 벽, 다른 구역, keepout_extra, 진열대에서 reach 이상
        wall = np.minimum(np.minimum(xs - x0, x1 - xs), np.minimum(ys - y0, y1 - ys))
        free = hall & (wall >= self.reach - _EPS)
        for rect in blocked:
            free &= _rect_distance(rect, xs, ys) >= self.reach - _EPS
        self.free = free
        # 키우기 전 바닥: HALL 안이고 진열대 밖
        floor = hall & (wall > 0.0)
        for rect in shelves:
            floor &= ~_inside(rect, xs, ys)
        self.floor = floor

    # ---- 칸과 좌표 변환 ----
    def centre(self, cell: Cell) -> Point:
        return (float(self.xs[cell[0]]), float(self.ys[cell[1]]))

    def cell(self, p: Point) -> Cell | None:
        i = math.floor((p[0] - self.x0) / self.res)
        j = math.floor((p[1] - self.y0) / self.res)
        if 0 <= i < self.w and 0 <= j < self.h:
            return (i, j)
        return None

    def nearest_free(self, p: Point) -> Cell:
        js, iz = np.nonzero(self.free)
        gaps = np.hypot(self.xs[iz] - p[0], self.ys[js] - p[1])
        k = int(np.argmin(gaps))
        return (int(iz[k]), int(js[k]))


# ---- 정확한 거리 검사: 경로 선분과 벽·막힌 사각형 사이 최소 거리 ----
def _point_rect(p: Point, rect) -> float:
    return math.hypot(max(rect[0] - p[0], 0.0, p[0] - rect[2]),
                      max(rect[1] - p[1], 0.0, p[1] - rect[3]))


def _point_segment(p: Point, a: Point, b: Point) -> float:
    ux, uy = b[0] - a[0], b[1] - a[1]
    span = ux * ux + uy * uy
    t = 0.0 if span <= _EPS else min(1.0, max(0.0, ((p[0] - a[0]) * ux + (p[1] - a[1]) * uy) / span))
    return math.hypot(p[0] - a[0] - t * ux, p[1] - a[1] - t * uy)


def _clip(a: Point, b: Point, rect) -> tuple[float, float] | None:
    """선분 a→b가 사각형 안에 드는 t 구간. 안 들면 None."""
    t0, t1 = 0.0, 1.0
    dx, dy = b[0] - a[0], b[1] - a[1]
    for p, q in ((-dx, a[0] - rect[0]), (dx, rect[2] - a[0]),
                 (-dy, a[1] - rect[1]), (dy, rect[3] - a[1])):
        if abs(p) <= _EPS:
            if q < -_EPS:
                return None
            continue
        t = q / p
        if p < 0.0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return None
    return t0, t1


def _segment_hits_rect(a: Point, b: Point, rect) -> bool:
    return _clip(a, b, rect) is not None


def _segment_rect(a: Point, b: Point, rect) -> float:
    # 겹치지 않으면 최소 거리는 선분 끝점과 사각형, 또는 사각형 꼭짓점과 선분 사이에 있다
    if _segment_hits_rect(a, b, rect):
        return 0.0
    corners = ((rect[0], rect[1]), (rect[2], rect[1]), (rect[0], rect[3]), (rect[2], rect[3]))
    return min(_point_rect(a, rect), _point_rect(b, rect),
               *(_point_segment(c, a, b) for c in corners))


def _wall(box, p: Point) -> float:
    return min(p[0] - box[0], box[2] - p[0], p[1] - box[1], box[3] - p[1])


def segment_clearance(grid: CleanGrid, a: Point, b: Point) -> float:
    # 벽 거리는 선분 위에서 끝점이 가장 작다
    return min(_wall(grid.box, a), _wall(grid.box, b),
               *(_segment_rect(a, b, rect) for rect in grid.blocked))


def _on_hall(grid: CleanGrid, a: Point, b: Point) -> bool:
    # HALL 사각형마다 선분이 드는 구간을 모아 [0, 1]을 빈틈없이 덮는지 본다
    spans = sorted(span for rect in grid.hall_rects
                   if (span := _clip(a, b, (rect[0] - _EPS, rect[1] - _EPS,
                                            rect[2] + _EPS, rect[3] + _EPS))) is not None)
    reach = 0.0
    for t0, t1 in spans:
        if t0 > reach + _EPS:
            return False
        reach = max(reach, t1)
    return reach >= 1.0 - _EPS


def path_clear(grid: CleanGrid, points: Sequence[Point]) -> bool:
    """경로의 모든 점이 HALL 안이고 막힌 것과 벽에서 (반지름 + 여유) 이상."""
    pts = _dedup(points)
    if len(pts) == 1:
        pts = [pts[0], pts[0]]
    return all(segment_clearance(grid, a, b) >= grid.reach - _EPS and _on_hall(grid, a, b)
               for a, b in pairwise(pts))


# ---- 격자 탐색: 8방향, 모서리 자르기 없음 ----
_STEPS = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def _neighbours(free: np.ndarray, cell: Cell):
    h, w = free.shape
    i, j = cell
    for di, dj in _STEPS:
        ni, nj = i + di, j + dj
        if not (0 <= ni < w and 0 <= nj < h) or not free[nj, ni]:
            continue
        if di and dj and not (free[j, ni] and free[nj, i]):
            continue
        yield (ni, nj), math.sqrt(2.0) if di and dj else 1.0


def distances(free: np.ndarray, start: Cell) -> np.ndarray:
    """start에서 모든 칸까지 격자 거리(칸 단위). 닿지 않으면 inf."""
    dist = np.full(free.shape, math.inf)
    dist[start[1], start[0]] = 0.0
    queue = [(0.0, start)]
    while queue:
        d, cell = heapq.heappop(queue)
        if d > dist[cell[1], cell[0]]:
            continue
        for nxt, cost in _neighbours(free, cell):
            nd = d + cost
            if nd < dist[nxt[1], nxt[0]] - _EPS:
                dist[nxt[1], nxt[0]] = nd
                heapq.heappush(queue, (nd, nxt))
    return dist


def astar(free: np.ndarray, start: Cell, goal: Cell) -> list[Cell]:
    def guess(c: Cell) -> float:
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    best = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    queue = [(guess(start), 0.0, start)]
    while queue:
        _, g, cell = heapq.heappop(queue)
        if cell == goal:
            path = [cell]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return path[::-1]
        if g > best[cell]:
            continue
        for nxt, cost in _neighbours(free, cell):
            ng = g + cost
            if ng < best.get(nxt, math.inf) - _EPS:
                best[nxt] = ng
                parent[nxt] = cell
                heapq.heappush(queue, (ng + guess(nxt), ng, nxt))
    raise ValueError(f"no path on the clean grid from {start} to {goal}")


# ---- 이동 구간: A* 칸 경로를 보이는 점끼리 이은 꺾은선(둥글게 바꿔도 빈칸) ----
def _shorten(grid: CleanGrid, pts: list[Point], radius: float) -> list[Point]:
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = i + 1
        for k in range(len(pts) - 1, i + 1, -1):
            if path_clear(grid, [pts[i], pts[k]]) and path_clear(
                    grid, rounded([*out, pts[k]], radius)):
                j = k
                break
        out.append(pts[j])
        i = j
    if path_clear(grid, rounded(out, radius)):
        return out
    return pts


def transit(grid: CleanGrid, free: np.ndarray, a: Point, b: Point, radius: float) -> list[Point]:
    cells = astar(free, grid.cell(a), grid.cell(b))
    pts = _dedup([a, *(grid.centre(c) for c in cells[1:-1]), b])
    return _shorten(grid, pts, radius)


# ---- boustrophedon 셀 분해: 같은 y 칸 줄의 이어진 빈칸 구간을 위아래로 묶음 ----
def _runs(row: np.ndarray) -> list[tuple[int, int]]:
    runs, start = [], None
    for i, ok in enumerate(row):
        if ok and start is None:
            start = i
        elif not ok and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(row) - 1))
    return runs


def decompose(free: np.ndarray) -> list[list[tuple[int, int, int]]]:
    """셀 목록. 셀은 (j, i0, i1) 칸 구간을 j 순서로 담는다."""
    cells: list[list[tuple[int, int, int]]] = []
    prev: list[tuple[int, int]] = []
    owner: dict[tuple[int, int], int] = {}
    for j in range(free.shape[0]):
        runs = _runs(free[j])
        mine: dict[tuple[int, int], int] = {}

        def touching(r, others):
            return [o for o in others if r[0] <= o[1] and o[0] <= r[1]]

        for run in runs:
            below = touching(run, prev)
            if len(below) == 1 and len(touching(below[0], runs)) == 1:
                index = owner[below[0]]
            else:
                index = len(cells)
                cells.append([])
            cells[index].append((j, *run))
            mine[run] = index
        prev, owner = runs, mine
    return cells


# ---- 셀마다 줄 고르기: 줄 간격 이하로 고르게 ----
def cell_lines(cell: list[tuple[int, int, int]], params: Params) -> list[tuple[int, int, int]]:
    step = max(1, math.floor(params.line_spacing_m / params.grid_m + _EPS))
    j0, j1 = cell[0][0], cell[-1][0]
    span = j1 - j0
    count = math.ceil(span / step) if span else 0
    picks = sorted({j0 + math.floor(k * span / count + 0.5) for k in range(count + 1)}
                   if count else {j0})
    rows = {j: (i0, i1) for j, i0, i1 in cell}
    return [(j, *rows[j]) for j in picks if rows[j][1] > rows[j][0]]


# ---- 계획: 셀 순서, 줄 방향, 이동 구간, 집 자리 왕복 ----
@dataclass
class Plan:
    params: Params
    grid: CleanGrid
    home: Point
    start: Point
    home_moved: bool
    unreachable_cells: int
    cells: list[dict]
    lines: list[tuple[Point, Point, int]]
    transits: list[tuple[str, str, list[Point]]]

    def paths(self) -> list[list[Point]]:
        """로봇이 차례로 따라가는 경로(줄과 이동 구간)."""
        out = [self.transits[0][2]]
        for k, (a, b, _) in enumerate(self.lines):
            out.append([a, b])
            out.append(self.transits[k + 1][2])
        return out


def make_plan(site, params: Params = Params()) -> Plan:
    grid = CleanGrid(site, params)
    home = site.node_pose(HOME)
    start_cell = grid.cell(home)
    moved = start_cell is None or not grid.free[start_cell[1], start_cell[0]]
    if moved:
        start_cell = grid.nearest_free(home)
    start = grid.centre(start_cell) if moved else home
    # 집 자리에서 닿는 빈칸만 쓴다
    reach = distances(grid.free, start_cell)
    free = np.isfinite(reach)
    unreachable = int(grid.free.sum() - free.sum())
    todo = []
    for raw in decompose(free):
        lines = cell_lines(raw, params)
        if lines:
            todo.append((raw, lines))
    cells, lines, transits = [], [], []
    here, here_name = start, "home"
    radius = params.turn_radius_m
    while todo:
        dist = distances(free, grid.cell(here))
        best = None
        for index, (raw, picks) in enumerate(todo):
            for up in (True, False):
                sweep = picks if up else picks[::-1]
                for east in (True, False):
                    j, i0, i1 = sweep[0]
                    entry = (i0 if east else i1, j)
                    key = (dist[entry[1], entry[0]], index, not up, not east)
                    if best is None or key < best[0]:
                        best = (key, index, sweep, east)
        _, index, sweep, east = best
        raw, _ = todo.pop(index)
        number = len(cells)
        xs = [grid.xs[i] for _, i0, i1 in raw for i in (i0, i1)]
        cells.append({"id": number, "box": [min(xs), grid.ys[raw[0][0]],
                                            max(xs), grid.ys[raw[-1][0]]],
                      "lines": len(sweep)})
        for j, i0, i1 in sweep:
            a, b = grid.centre((i0, j)), grid.centre((i1, j))
            if not east:
                a, b = b, a
            name = f"line {len(lines)}"
            transits.append((here_name, name, transit(grid, free, here, a, radius)))
            lines.append((a, b, number))
            here, here_name = b, name
            east = not east
    transits.append((here_name, "home", transit(grid, free, here, start, radius)))
    return Plan(params, grid, home, start, moved, unreachable, cells, lines, transits)


# ---- 실제 순서의 한 바퀴: 나가기, 줄마다 이동 + 줄, 돌아오기, 후진 들어가기 ----
DENSE_M = 0.05  # 꼭짓점 간격 상한: 후퇴(robot_step retreat)가 5 cm 남짓만 물러나게


@dataclass(frozen=True)
class CleanLap:
    pts: np.ndarray  # (V, 2) 한 바퀴 꼭짓점, 처음과 끝은 집 자리
    arc: np.ndarray  # (V,) 누적 길이 m
    leg_end: np.ndarray  # (V,) 꼭짓점이 든 다리 끝의 호장 (intent 는 여기서 멈춤)
    reverse: np.ndarray  # (V - 1,) 후진 구간 (bay_in)
    rows: tuple  # 줄 (a, b), plan.lines 순서
    plan_lap_s: float  # coverage.py summary 의 lap_time_s (줄 + 둥근 이동, 나가기/들어가기 빼고)

    @property
    def length(self) -> float:
        return float(self.arc[-1])


def _densify(pts: list[Point], step: float) -> list[Point]:
    out = [pts[0]]
    for a, b in pairwise(pts):
        k = max(1, math.ceil(math.dist(a, b) / step - _EPS))
        out += [(a[0] + (b[0] - a[0]) * i / k, a[1] + (b[1] - a[1]) * i / k)
                for i in range(1, k + 1)]
    return out


def clean_lap(site, params: Params = Params(), dense_m: float = DENSE_M) -> CleanLap:
    """make_plan 의 줄과 이동 구간을 zone/cleaning 의 다리 순서로 이은 한 바퀴."""
    plan = make_plan(site, params)
    edges = list(site.edges())
    out_edge = next(e for e in edges if e.a == HOME and e.mode == "follow_path")
    in_edge = next(e for e in edges if e.b == HOME and e.mode == "retreat")
    r = params.turn_radius_m
    legs = [rounded([*out_edge.points, plan.start], r)]  # _out_leg
    for k, (_, b, _) in enumerate(plan.lines):  # _line_leg: 지금 자리 -> 줄 시작 이동 + 줄 끝
        legs.append(rounded([*plan.transits[k][2], b], r))
    legs.append(rounded([*plan.transits[-1][2], site.node_pose(in_edge.a)], r))  # _back_leg
    legs.append(list(in_edge.points))  # bay_in_1 후진
    # 다리 이음 꼭짓점은 두 번 둔다 (길이 0 구간): 새 다리 첫 꼭짓점부터 intent 가 새 다리 끝까지
    pts, ends, rev = [], [], []
    for li, leg in enumerate(legs):
        dense = _densify(_dedup(leg), dense_m)
        rev += ([False] if pts else []) + [li == len(legs) - 1] * (len(dense) - 1)
        pts += dense
        ends.append(len(pts) - 1)
    xy = np.array(pts)
    arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))))
    leg_end = np.zeros(len(xy))
    first = 0
    for e in ends:
        leg_end[first : e + 1] = arc[e]
        first = e + 1
    line_m = sum(math.dist(a, b) for a, b, _ in plan.lines)
    move_m = sum(path_length(rounded(p, r)) for _, _, p in plan.transits)
    return CleanLap(
        xy, arc, leg_end, np.array(rev),
        tuple((a, b) for a, b, _ in plan.lines), (line_m + move_m) / params.speed_mps,
    )


def laps_needed(lap: CleanLap, travel_m: float) -> int:
    """아무 출발점에서 travel_m 을 달려도 끝나지 않는 바퀴 수."""
    return 1 + math.ceil(travel_m / lap.length) + 1


def lap_table(lap: CleanLap, laps: int) -> dict:
    """바퀴를 laps 번 이은 (V,) 표: pts, arc, leg_end_arc, heading (후진 구간은 뒤를 봄)."""
    # 바퀴 이음도 집 자리 꼭짓점을 두 번 둔다
    pts = [lap.pts] * laps
    arc = [lap.arc + i * lap.length for i in range(laps)]
    end = [lap.leg_end + i * lap.length for i in range(laps)]
    p = torch.as_tensor(np.concatenate(pts), dtype=torch.float32)
    heading = rbs.route_heading(p[None])[0]
    rev = torch.as_tensor(np.concatenate([lap.reverse] + [[False], lap.reverse] * (laps - 1)))
    heading = torch.where(rev, heading + math.pi, heading)
    return {
        "pts": p,
        "arc": torch.as_tensor(np.concatenate(arc), dtype=torch.float32),
        "leg_end_arc": torch.as_tensor(np.concatenate(end), dtype=torch.float32),
        "heading": heading,
    }


def lap_route(table: dict, speed: torch.Tensor) -> dict:
    """env 별 청소 로봇 경로 dict (robot_spec 꼴): speed (n,) m/s, 꼭짓점 노드는 -1 (예약 밖)."""
    n, v = len(speed), table["pts"].shape[0]
    dev = speed.device
    arc = table["arc"].to(dev)[None].expand(n, v)
    nodes = torch.full((n, v), -1, dtype=torch.long, device=dev)
    return {
        "pts": table["pts"].to(dev)[None].expand(n, v, 2).clone(),
        "time": arc / speed[:, None].clamp(min=1e-6),
        "arc": arc.clone(),
        "leg_end_arc": table["leg_end_arc"].to(dev)[None].expand(n, v).clone(),
        "heading": table["heading"].to(dev)[None].expand(n, v - 1).clone(),
        "vnode": nodes,
        "is_goal": torch.zeros(n, v, dtype=torch.bool, device=dev),
        "next_node": nodes.clone(),
    }


# ---- env 연결: 길이 맞춤, 칸 섞기, 출발 시각, 로봇 수, 비키기 상대 ----
def pad_route(route: dict, v: int) -> dict:
    """경로 dict 를 꼭짓점 v 개로: 끝 꼭짓점을 되풀이 (시간이 같아 움직임은 그대로)."""
    out = {}
    for key, x in route.items():
        extra = v - route["pts"].shape[1]
        out[key] = torch.cat((x, x[:, -1:].expand(-1, extra, *x.shape[2:])), 1) if extra else x
    return out


def mix_route(graph_route: dict, clean_route: dict, on: torch.Tensor) -> dict:
    """on (n,) 인 행은 청소 경로, 나머지는 (길이를 맞춘) 그래프 경로."""
    v = max(graph_route["pts"].shape[1], clean_route["pts"].shape[1])
    g, c = pad_route(graph_route, v), pad_route(clean_route, v)
    out = {}
    for key in g:
        m = on.view(-1, *([1] * (g[key].dim() - 1)))
        out[key] = torch.where(m, c[key].to(g[key].dtype), g[key])
    return out


def start_clock(route: dict, lap_s: torch.Tensor, u: torch.Tensor, avoid: torch.Tensor,
                avoid_on: torch.Tensor, sep_m: float) -> torch.Tensor:
    """바퀴 위 무작위 출발 시각 (n,): 후보 u (n, m) * lap_s 중 avoid (n, A, 2) 의 켜진 점에서
    sep_m 이상 떨어진 첫 후보, 없으면 가장 먼 후보."""
    n, m = u.shape
    cand = u * lap_s[:, None]
    gaps = []
    for j in range(m):
        pos, _, _ = rbs.robot_pose(route, cand[:, j])
        d = (avoid - pos[:, None]).norm(dim=-1)
        gaps.append(torch.where(avoid_on, d, torch.full_like(d, float("inf"))).min(-1).values)
    gaps = torch.stack(gaps, 1)  # (n, m)
    ok = gaps >= sep_m
    first = torch.where(ok.any(-1), ok.float().argmax(-1), gaps.argmax(-1))
    return cand.gather(1, first[:, None])[:, 0]


def cleaner_active(count: torch.Tensor, on: torch.Tensor, nk: int) -> torch.Tensor:
    """(nk, n) 활성 칸: 청소 env 는 마지막 칸이 청소 로봇, 그래프 로봇은 앞 칸 count - 1 대."""
    k = torch.arange(nk, device=count.device)[:, None]
    graph = k < count[None]
    clean = (k < count[None] - 1) | (k == nk - 1)
    return torch.where(on[None], clean, graph)


def watch_xy(cleaner_xy: torch.Tensor, our_xy: torch.Tensor, others: torch.Tensor,
             others_on: torch.Tensor) -> torch.Tensor:
    """(n, 2) 청소 로봇이 비켜 줄 상대: 우리 로봇과 켜진 운반 로봇 (n, J, 2) 중 가장 가까운 것."""
    cand = torch.cat((our_xy[:, None], others), 1)
    ours_on = torch.ones(len(our_xy), 1, dtype=torch.bool, device=our_xy.device)
    on = torch.cat((ours_on, others_on), 1)
    d = (cand - cleaner_xy[:, None]).norm(dim=-1)
    d = torch.where(on, d, torch.full_like(d, float("inf")))
    return cand[torch.arange(len(d), device=d.device), d.argmin(-1)]
