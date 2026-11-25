# ---- 청소 로봇 커버리지 경로: 청소 격자, boustrophedon 줄, A* 이동, 둥근 경로 검사 ----
from __future__ import annotations

import argparse
import heapq
import math
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

import numpy as np
import yaml

from .fleet_map import load_fleet_map
from .fleet_map.fleet_map import DATA, TURN_RADIUS_M

# ---- 기본 파라미터 ----
GRID_M = 0.025
ROBOT_RADIUS_M = 0.083
MARGIN_M = 0.01  # 다른 구역과 keepout_extra 경계에서 두는 여유
SOLID_MARGIN_M = 0.03  # 벽과 진열대에서 두는 여유
MIN_LINE_M = 0.20  # 이보다 짧은 줄은 긋지 않는다
MAX_LINE_M = 0.60  # 이보다 긴 줄은 같은 방향으로 이어지는 구간으로 나눈다
LINE_SPACING_M = 0.10
CLEAN_WIDTH_M = 0.112
SPEED_MPS = 0.12
REGION_M = 0.60  # 지도에 clean_regions가 없으면 줄 방향으로 이 폭 이하의 영역으로 나눈다
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
    solid_margin_m: float = SOLID_MARGIN_M
    min_line_m: float = MIN_LINE_M
    max_line_m: float = MAX_LINE_M
    line_spacing_m: float = LINE_SPACING_M
    clean_width_m: float = CLEAN_WIDTH_M
    turn_radius_m: float = TURN_RADIUS_M
    speed_mps: float = SPEED_MPS
    region_width_m: float = REGION_M

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


# ---- 청소 격자: HALL만 바닥, 막힌 것은 (반지름 + 여유)만큼 키움. 벽과 진열대는 여유가 더 크다 ----
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
        self.solid_reach = params.robot_radius_m + params.solid_margin_m
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
        soft = [rect for zone in sorted(site.zones.values(), key=lambda z: z.id)
                if zone.id != FLOOR_ZONE for rect in zone.rects]
        soft += list(site.keepout_extra)
        # (사각형, 떨어질 거리): 다른 구역과 keepout_extra는 reach, 진열대는 solid_reach
        self.blocked = (*((rect, self.reach) for rect in soft),
                        *((rect, self.solid_reach) for rect in shelves))
        self.hall_rects = tuple(site.zones[FLOOR_ZONE].rects)
        # 키운 격자: 벽과 진열대에서 solid_reach, 다른 구역과 keepout_extra에서 reach 이상
        wall = np.minimum(np.minimum(xs - x0, x1 - xs), np.minimum(ys - y0, y1 - ys))
        free = hall & (wall >= self.solid_reach - _EPS)
        for rect, need in self.blocked:
            free &= _rect_distance(rect, xs, ys) >= need - _EPS
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


def segment_slack(grid: CleanGrid, a: Point, b: Point) -> float:
    """선분이 떨어질 거리보다 더 떨어진 만큼. 음수면 너무 가깝다."""
    # 벽 거리는 선분 위에서 끝점이 가장 작다
    return min(_wall(grid.box, a) - grid.solid_reach, _wall(grid.box, b) - grid.solid_reach,
               *(_segment_rect(a, b, rect) - need for rect, need in grid.blocked))


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
    """경로의 모든 점이 HALL 안이고 막힌 것과 벽에서 떨어질 거리 이상."""
    pts = _dedup(points)
    if len(pts) == 1:
        pts = [pts[0], pts[0]]
    return all(segment_slack(grid, a, b) >= -_EPS and _on_hall(grid, a, b)
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
# away: (점, 거리) 목록. 경로가 그 점들에서 그 거리 넘게 떨어진다
def _shorten(grid: CleanGrid, pts: list[Point], radius: float, away=()) -> list[Point]:
    def clear(points) -> bool:
        return path_clear(grid, points) and all(
            _point_segment(q, a, b) > gap for q, gap in away for a, b in pairwise(points))

    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = i + 1
        for k in range(len(pts) - 1, i + 1, -1):
            if clear([pts[i], pts[k]]) and clear(rounded([*out, pts[k]], radius)):
                j = k
                break
        out.append(pts[j])
        i = j
    if clear(rounded(out, radius)):
        return out
    return pts


def transit(grid: CleanGrid, free: np.ndarray, a: Point, b: Point, radius: float,
            away=()) -> list[Point]:
    if away:
        xs, ys = np.meshgrid(grid.xs, grid.ys)
        free = free.copy()
        for q, gap in away:
            free &= np.hypot(xs - q[0], ys - q[1]) > gap
    cells = astar(free, grid.cell(a), grid.cell(b))
    pts = _dedup([a, *(grid.centre(c) for c in cells[1:-1]), b])
    return _shorten(grid, pts, radius, away)


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


def decompose(free: np.ndarray, jump: int | None = None) -> list[list[tuple[int, int, int]]]:
    """셀 목록. 셀은 (j, i0, i1) 칸 구간을 j 순서로 담는다.

    jump: 아래 줄과 하나씩 이어져도 구간 끝이 이만큼 넘게(칸) 바뀌면 새 셀로 나눈다.
    """
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
            same = len(below) == 1 and len(touching(below[0], runs)) == 1
            if same and jump is not None:
                # 폭이 갑자기 바뀌면 고르게 고른 줄이 넓은 칸을 못 덮는다
                same = max(abs(run[0] - below[0][0]), abs(run[1] - below[0][1])) <= jump
            if same:
                index = owner[below[0]]
            else:
                index = len(cells)
                cells.append([])
            cells[index].append((j, *run))
            mine[run] = index
        prev, owner = runs, mine
    return cells


# ---- 셀마다 줄 고르기: 줄 간격 이하로 고르게. 짧은 줄은 빼고 긴 줄은 이어지는 구간으로 나눈다 ----
def cell_lines(cell: list[tuple[int, int, int]], params: Params) -> list[tuple[int, int, int]]:
    step = max(1, math.floor(params.line_spacing_m / params.grid_m + _EPS))
    j0, j1 = cell[0][0], cell[-1][0]
    span = j1 - j0
    count = math.ceil(span / step) if span else 0
    picks = sorted({j0 + math.floor(k * span / count + 0.5) for k in range(count + 1)}
                   if count else {j0})
    rows = {j: (i0, i1) for j, i0, i1 in cell}
    least = max(1, math.ceil(params.min_line_m / params.grid_m - _EPS))
    most = math.floor(params.max_line_m / params.grid_m + _EPS) if params.max_line_m > 0 else 0
    out = []
    for j in picks:
        i0, i1 = rows[j]
        if i1 - i0 < least:
            continue
        # 조각이 가장 짧은 줄보다 짧아지면 덜 나눈다
        count = max(1, min(math.ceil((i1 - i0) / most), (i1 - i0) // least)) if most else 1
        cuts = [i0 + math.floor(k * (i1 - i0) / count + 0.5) for k in range(count + 1)]
        out += [(j, a, b) for a, b in pairwise(cuts)]
    return out


# ---- 영역: 지도 clean_regions 사각형마다 빈칸 마스크. 없으면 줄 방향(x)으로 자른 띠(서쪽부터) ----
def regions(grid: CleanGrid, free: np.ndarray, params: Params, rects=()) -> list[np.ndarray]:
    if rects:
        xs, ys = np.meshgrid(grid.xs, grid.ys)
        left, out = free.copy(), []
        for rect in rects:
            out.append(left & _inside(rect, xs, ys))
            left = left & ~out[-1]
        # 어느 사각형에도 들지 않은 빈칸은 마지막 영역
        return [mask for mask in (*out, left) if mask.any()]
    cols = np.nonzero(free.any(axis=0))[0]
    if params.region_width_m <= 0 or not len(cols):
        return [free]
    i0, i1 = int(cols[0]), int(cols[-1]) + 1
    # 줄은 칸 중심을 잇는다: 영역 칸 수 - 1 칸이 region_width_m 이하
    most = math.floor(params.region_width_m / params.grid_m + _EPS) + 1
    count = max(1, math.ceil((i1 - i0) / most))
    cuts = [i0 + math.floor(k * (i1 - i0) / count + 0.5) for k in range(count + 1)]
    out = []
    for a, b in pairwise(cuts):
        mask = np.zeros_like(free)
        mask[:, a:b] = free[:, a:b]
        out.append(mask)
    return out


# ---- 다른 줄이 이미 덮는 빈칸만 덮는 줄은 짧은 것부터 뺀다 ----
def needed_lines(grid: CleanGrid, free: np.ndarray, params: Params, todo: list) -> list:
    picked = {(index, k): line for index, (_, _, picks) in enumerate(todo)
              for k, line in enumerate(picks)}
    masks = {}
    for key, (j, i0, i1) in picked.items():
        ends = [grid.centre((i0, j)), grid.centre((i1, j))]
        masks[key] = covered_by(grid, params, [ends]) & free
    times = np.zeros(free.shape, dtype=int)
    for mask in masks.values():
        times += mask
    drop = set()
    for key in sorted(picked, key=lambda key: (picked[key][2] - picked[key][1], *key)):
        if not (masks[key] & (times == 1)).any():
            times -= masks[key]
            drop.add(key)
    out = []
    for index, (region, raw, picks) in enumerate(todo):
        keep = [line for k, line in enumerate(picks) if (index, k) not in drop]
        if keep:
            out.append((region, raw, keep))
    return out


# ---- 계획: 영역 순서, 줄 순서와 방향, 이동 구간, 집 자리 왕복 ----
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
    jump = max(1, math.floor(params.line_spacing_m / params.grid_m + _EPS))
    rects = getattr(site, "clean_regions", ())
    for region, mask in enumerate(regions(grid, free, params, rects)):
        for raw in decompose(mask, jump):
            lines = cell_lines(raw, params)
            if lines:
                todo.append((region, raw, lines))
    todo = needed_lines(grid, free, params, todo)
    cells, lines, transits = [], [], []
    here, here_name = start, "home"
    radius = params.turn_radius_m
    left = [(index, line) for index, (_, _, picks) in enumerate(todo) for line in picks]
    numbers: dict[int, int] = {}
    region = None
    while left:
        dist = distances(free, grid.cell(here))
        # 지금 영역의 줄을 다 쓴 뒤에 번호가 가장 앞선 남은 영역으로 간다
        # 줄은 끝이 가장 가까운 것부터, 같으면 짧은 줄부터
        if not any(todo[index][0] == region for index, _ in left):
            region = min(todo[index][0] for index, _ in left)
        mine = [item for item in left if todo[item[0]][0] == region]
        best = None
        for item in mine:
            index, (j, i0, i1) = item
            for east in (True, False):
                key = (round(float(dist[j, i0 if east else i1]), 6), i1 - i0, index, j, not east)
                if best is None or key < best[0]:
                    best = (key, item, east)
        _, item, east = best
        left.remove(item)
        index, (j, i0, i1) = item
        region, raw, picks = todo[index]
        if index not in numbers:
            numbers[index] = len(cells)
            xs = [grid.xs[i] for _, a, b in raw for i in (a, b)]
            cells.append({"id": numbers[index], "region": region,
                          "box": [min(xs), grid.ys[raw[0][0]], max(xs), grid.ys[raw[-1][0]]],
                          "lines": len(picks)})
        a, b = grid.centre((i0, j)), grid.centre((i1, j))
        if not east:
            a, b = b, a
        name = f"line {len(lines)}"
        transits.append((here_name, name, transit(grid, free, here, a, radius)))
        lines.append((a, b, numbers[index]))
        here, here_name = b, name
    transits.append((here_name, "home", transit(grid, free, here, start, radius)))
    return Plan(params, grid, home, start, moved, unreachable, cells, lines, transits)


# ---- 둥글게 바꾼 경로 검사와 커버율 ----
def rounded_problems(plan: Plan) -> list[str]:
    """둥글게 바꾼 뒤 빈칸을 벗어나는 경로. 비면 통과."""
    bad = []
    for k, path in enumerate(plan.paths()):
        if not path_clear(plan.grid, rounded(path, plan.params.turn_radius_m)):
            bad.append(f"path {k} comes too close or leaves HALL after rounding: {path}")
    return bad


def covered(plan: Plan) -> np.ndarray:
    return covered_by(plan.grid, plan.params, plan.paths())


def covered_by(grid: CleanGrid, params: Params, paths) -> np.ndarray:
    """경로들을 둥글게 바꾼 뒤 청소 폭이 지나간 칸."""
    xs, ys = np.meshgrid(grid.xs, grid.ys)
    half = params.clean_width_m / 2.0
    hit = np.zeros(xs.shape, dtype=bool)
    for path in paths:
        pts = rounded(path, params.turn_radius_m)
        if len(pts) == 1:
            pts = [pts[0], pts[0]]
        for a, b in pairwise(pts):
            ux, uy = b[0] - a[0], b[1] - a[1]
            span = ux * ux + uy * uy
            t = np.zeros(xs.shape) if span <= _EPS else np.clip(
                ((xs - a[0]) * ux + (ys - a[1]) * uy) / span, 0.0, 1.0)
            hit |= np.hypot(xs - a[0] - t * ux, ys - a[1] - t * uy) <= half + _EPS
    return hit


def coverage(plan: Plan) -> tuple[float, float]:
    """(키우기 전 HALL 바닥 기준, 키운 격자 빈칸 기준) 커버율."""
    hit = covered(plan)
    grid = plan.grid
    return (float((hit & grid.floor).sum() / grid.floor.sum()),
            float((hit & grid.free).sum() / grid.free.sum()))


# ---- 결과 파일 내용 ----
def _mm(v: float) -> float:
    return round(float(v), 3) + 0.0


def _xy(v: float) -> float:
    # 칸 중심이 0.5 mm 단위라 좌표는 0.1 mm로 저장한다
    return round(float(v), 4) + 0.0


def _pt(p: Point) -> list[float]:
    return [_xy(p[0]), _xy(p[1])]


def summary(plan: Plan) -> dict:
    radius = plan.params.turn_radius_m
    line_m = sum(math.dist(a, b) for a, b, _ in plan.lines)
    move_m = sum(path_length(rounded(pts, radius)) for _, _, pts in plan.transits)
    floor_ratio, free_ratio = coverage(plan)
    return {
        "regions": len({c["region"] for c in plan.cells}),
        "cells": len(plan.cells),
        "lines": len(plan.lines),
        "line_length_m": _mm(line_m),
        "transit_length_m": _mm(move_m),
        "lap_time_s": round((line_m + move_m) / plan.params.speed_mps, 1),
        "coverage_hall_floor": round(floor_ratio, 4),
        "coverage_clean_grid": round(free_ratio, 4),
        "unreachable_free_cells": plan.unreachable_cells,
    }


def to_data(plan: Plan, layout: str) -> dict:
    p = plan.params
    return {
        "layout": layout,
        "command": f"python3 -m fleet_core.coverage --layout {layout} --write",
        "params": {
            "grid_m": p.grid_m, "robot_radius_m": p.robot_radius_m, "margin_m": p.margin_m,
            "solid_margin_m": p.solid_margin_m, "min_line_m": p.min_line_m,
            "max_line_m": p.max_line_m,
            "line_spacing_m": p.line_spacing_m, "clean_width_m": p.clean_width_m,
            "turn_radius_m": p.turn_radius_m, "speed_mps": p.speed_mps,
            "region_width_m": p.region_width_m,
            "line_direction": "x",
        },
        "home": {"node": HOME, "pose": _pt(plan.home), "start": _pt(plan.start),
                 "moved_to_nearest_free": plan.home_moved},
        "summary": summary(plan),
        "cells": [{"id": c["id"], "region": c["region"], "box": [_xy(v) for v in c["box"]],
                   "lines": c["lines"]}
                  for c in plan.cells],
        "lines": [{"cell": cell, "start": _pt(a), "end": _pt(b)} for a, b, cell in plan.lines],
        "transits": [{"from": a, "to": b, "points": [_pt(q) for q in pts],
                      "length_m": _mm(path_length(pts))} for a, b, pts in plan.transits],
    }


def to_yaml(plan: Plan, layout: str) -> str:
    head = (f"# 청소 로봇 커버리지 경로(배치 {layout}). 손으로 고치지 않는다.\n"
            f"# 다시 만들기: cd pc/fleet_core && python3 -m fleet_core.coverage "
            f"--layout {layout} --write\n")
    body = yaml.safe_dump(to_data(plan, layout), sort_keys=False, default_flow_style=None,
                          allow_unicode=True, width=100)
    return head + body


def data_path(layout: str) -> Path:
    return DATA / f"coverage_{layout}.yaml"


# ---- 명령줄: 요약 출력, --write면 data 파일 쓰기 ----
def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Cleaner coverage path for a fleet map layout")
    parser.add_argument("--layout", default="v7")
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--line-spacing", type=float, default=LINE_SPACING_M)
    args = parser.parse_args(argv)
    plan = make_plan(load_fleet_map(args.layout), Params(line_spacing_m=args.line_spacing))
    problems = rounded_problems(plan)
    print(yaml.safe_dump({"layout": args.layout, "home_moved": plan.home_moved,
                          **summary(plan)}, sort_keys=False), end="")
    for problem in problems:
        print(problem)
    if problems:
        return 1
    if args.write:
        data_path(args.layout).write_text(to_yaml(plan, args.layout), encoding="utf-8")
        print(f"wrote {data_path(args.layout)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
