"""청소 로봇 커버리지 경로: data 파일 일치, 둥근 경로 빈칸 검사, 영역과 줄, 여유, 커버율."""
import importlib.util
import math
import sys
from pathlib import Path

from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from fleet_core import coverage as cov
from fleet_core.coverage import (
    CleanGrid,
    Params,
    coverage,
    data_path,
    make_plan,
    path_clear,
    rounded,
    rounded_problems,
    to_yaml,
)
from fleet_core.fleet_map import FleetMapError, fleet_map_from_text, load_fleet_map

PODE_FSM = Path(__file__).resolve().parents[3] / "pode" / "pode_fsm" / "pode_fsm"
LAYOUTS = ("v7", "d2")


# ---- pode_fsm 파일을 패키지 없이 직접 읽기 ----
def _load(name: str):
    spec = importlib.util.spec_from_file_location(f"pode_fsm_{name}", PODE_FSM / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    # dataclass가 sys.modules에서 모듈을 찾는다
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        del sys.modules[spec.name]
    return module


@pytest.fixture(scope="module", params=LAYOUTS)
def layout_plan(request):
    return request.param, make_plan(load_fleet_map(request.param))


# ---- 커밋된 data 파일과 같다 ----
def test_data_file_matches(layout_plan):
    layout, plan = layout_plan
    assert to_yaml(plan, layout) == data_path(layout).read_text(encoding="utf-8"), (
        f"regenerate: python3 -m fleet_core.coverage --layout {layout} --write")


# ---- 둥글게 바꾼 뒤 모든 경로가 키운 격자의 빈칸 위 ----
def test_rounded_paths_stay_on_clean_grid(layout_plan):
    _, plan = layout_plan
    assert rounded_problems(plan) == []
    assert plan.lines and len(plan.transits) == len(plan.lines) + 1


def test_rounded_paths_with_narrow_spacing():
    for layout in LAYOUTS:
        plan = make_plan(load_fleet_map(layout), Params(line_spacing_m=0.068))
        assert rounded_problems(plan) == []


# ---- 커버율: 키운 격자 빈칸 기준 99 % 이상(줄과 이동 구간) ----
def test_clean_grid_coverage(layout_plan):
    _, plan = layout_plan
    _, on_grid = coverage(plan)
    assert on_grid >= 0.99


# ---- 영역 나누기: 지도 clean_regions(없으면 region_width_m 띠)마다 줄을 긋고 한 영역씩 끝낸다 ----
V7_REGIONS = ((0.0, 0.0, 0.65, 0.98), (0.65, 0.3675, 1.98, 0.98), (0.65, 0.0, 1.98, 0.3675))


def _region_order(plan):
    return [plan.cells[cell]["region"] for _, _, cell in plan.lines]


def _in(rect, p):
    return rect[0] <= p[0] <= rect[2] and rect[1] <= p[1] <= rect[3]


def test_v7_map_names_three_clean_regions():
    assert load_fleet_map("v7").clean_regions == V7_REGIONS
    assert load_fleet_map("d2").clean_regions == ()


def test_lines_stay_inside_one_region(layout_plan):
    layout, plan = layout_plan
    for (a, b, _), region in zip(plan.lines, _region_order(plan), strict=True):
        if layout == "v7":
            assert _in(V7_REGIONS[region], a) and _in(V7_REGIONS[region], b), (a, b, region)
        else:
            assert abs(b[0] - a[0]) <= plan.params.region_width_m + 1e-9


def test_regions_are_swept_one_after_another(layout_plan):
    _, plan = layout_plan
    seen = []
    for region in _region_order(plan):
        if not seen or seen[-1] != region:
            assert region not in seen, f"comes back to region {region}"
            seen.append(region)


def test_v7_sweeps_the_west_floor_then_the_upper_aisle_then_the_lower_aisle():
    plan = make_plan(load_fleet_map("v7"))
    assert list(dict.fromkeys(_region_order(plan))) == [0, 1, 2]
    # 통로의 한 칸 줄은 끝에서 끝까지 쓴다(구간으로 나눠도 합치면 1 m 이상)
    for region in (1, 2):
        rows = {}
        for (a, b, _), r in zip(plan.lines, _region_order(plan), strict=True):
            if r == region:
                rows[a[1]] = rows.get(a[1], 0.0) + abs(b[0] - a[0])
        assert max(rows.values()) >= 1.0


def test_regions_are_swept_in_the_order_the_map_lists_them():
    site = load_fleet_map("v7")
    # 아래 통로를 위 통로보다 먼저 적으면 아래 통로부터 돈다(가까운 영역 순서가 아니다)
    site.clean_regions = (V7_REGIONS[0], V7_REGIONS[2], V7_REGIONS[1])
    plan = make_plan(site)
    assert _region_order(plan) == sorted(_region_order(plan))
    assert list(dict.fromkeys(_region_order(plan))) == [0, 1, 2]
    first = {r: a[1] for (a, _, _), r in zip(plan.lines, _region_order(plan), strict=True)}
    assert first[1] < 0.3675 < first[2]
    # 집에서 먼 영역을 맨 앞에 적어도 그 영역부터 돈다
    site.clean_regions = (V7_REGIONS[1], V7_REGIONS[0], V7_REGIONS[2])
    far = _region_order(make_plan(site))
    assert far == sorted(far) and set(far) == {0, 1, 2}


def test_floor_outside_every_region_rectangle_is_the_last_region():
    site = load_fleet_map("v7")
    site.clean_regions = (V7_REGIONS[0],)
    plan = make_plan(site)
    assert list(dict.fromkeys(_region_order(plan))) == [0, 1]
    assert all(a[0] > 0.65 and b[0] > 0.65
               for (a, b, _), r in zip(plan.lines, _region_order(plan), strict=True) if r == 1)


def test_clean_regions_must_be_on_the_floor_and_apart():
    text = load_fleet_map("v7").source_text
    first = "  - [0.0, 0.0, 0.65, 0.98] "
    assert text.count(first) == 1
    with pytest.raises(FleetMapError, match="overlaps"):
        fleet_map_from_text(text.replace(first, "  - [0.0, 0.0, 0.70, 0.98] "))
    with pytest.raises(FleetMapError, match="outside the floor"):
        fleet_map_from_text(text.replace(first, "  - [-0.1, 0.0, 0.65, 0.98] "))


def test_lines_alone_cover_the_clean_grid(layout_plan):
    """관제는 줄만 덮은 것으로 센다: 이동 구간 없이도 키운 격자를 98 % 이상 덮는다."""
    _, plan = layout_plan
    hit = cov.covered_by(plan.grid, plan.params, [[a, b] for a, b, _ in plan.lines])
    free = plan.grid.free
    assert (hit & free).sum() / free.sum() >= 0.98


def test_region_width_zero_keeps_whole_lines():
    plan = make_plan(load_fleet_map("d2"), Params(region_width_m=0.0, max_line_m=0.0))
    assert max(abs(b[0] - a[0]) for a, b, _ in plan.lines) > 1.4
    assert {c["region"] for c in plan.cells} == {0}


# ---- 긴 줄 나누기: max_line_m 이하 구간으로 나누고, 구간은 방향을 바꾸지 않고 이어진다 ----
def test_no_line_is_longer_than_the_maximum(layout_plan):
    _, plan = layout_plan
    assert max(math.dist(a, b) for a, b, _ in plan.lines) <= plan.params.max_line_m + 1e-9


def test_the_pieces_of_a_long_row_follow_each_other_without_turning():
    site = load_fleet_map("v7")
    plan, whole = make_plan(site), make_plan(site, Params(max_line_m=0.0))
    assert len(plan.lines) > len(whole.lines)
    joined = 0
    for (a, b, _), (c, d, _) in zip(plan.lines, plan.lines[1:], strict=False):
        if math.dist(b, c) < 1e-9:
            joined += 1
            assert a[1] == d[1] and (b[0] - a[0]) * (d[0] - c[0]) > 0.0, (a, b, c, d)
    assert joined >= 1
    # 나눠도 덮는 칸은 같다
    def cover(p):
        return cov.covered_by(p.grid, p.params, [[a, b] for a, b, _ in p.lines])
    assert (cover(plan) == cover(whole)).all()


# ---- 줄 고르기: 짧은 줄과 다른 줄이 이미 덮는 줄은 긋지 않는다 ----
def test_no_line_is_shorter_than_the_minimum(layout_plan):
    _, plan = layout_plan
    assert min(math.dist(a, b) for a, b, _ in plan.lines) >= plan.params.min_line_m - 1e-9


@pytest.mark.parametrize("longest", [0.225, 0.30, 0.375])
def test_a_long_row_is_not_cut_into_pieces_shorter_than_the_minimum(longest):
    # 최대 길이로 나눈 조각이 최소 길이보다 짧아지면 덜 나눈다
    site = load_fleet_map("v7")
    plan = make_plan(site, Params(max_line_m=longest))
    assert min(math.dist(a, b) for a, b, _ in plan.lines) >= plan.params.min_line_m - 1e-9
    # 그래도 긴 줄은 나눈다
    assert len(plan.lines) > len(make_plan(site, Params(max_line_m=0.0)).lines)


def test_a_zero_minimum_brings_the_short_lines_back():
    site = load_fleet_map("v7")
    short = make_plan(site, Params(min_line_m=0.0))
    assert len(short.lines) > len(make_plan(site).lines)
    assert min(math.dist(a, b) for a, b, _ in short.lines) < Params().min_line_m


def test_every_line_covers_a_cell_no_other_line_covers(layout_plan):
    _, plan = layout_plan
    free = plan.grid.free
    masks = [cov.covered_by(plan.grid, plan.params, [[a, b]]) & free for a, b, _ in plan.lines]
    times = sum(mask.astype(int) for mask in masks)
    for k, mask in enumerate(masks):
        assert (mask & (times == 1)).any(), f"line {k} only covers what other lines cover"


def test_a_line_that_covers_no_cell_of_its_own_is_dropped():
    grid = cov.CleanGrid(load_fleet_map("v7"), Params())
    js, iz = grid.free.nonzero()
    j = int(js.min()) + 4
    i0, i1 = int(iz[js == j].min()), int(iz[js == j].max())
    row = (j, i0, i1)
    twin = cov.needed_lines(grid, grid.free, Params(), [(0, None, [row, row])])
    assert [len(picks) for _, _, picks in twin] == [1]
    # 줄 간격만큼 떨어진 줄은 둘 다 남는다
    far = (j + 4, i0, i1)
    apart = cov.needed_lines(grid, grid.free, Params(), [(0, None, [row, far])])
    assert [len(picks) for _, _, picks in apart] == [2]


# ---- 여유: 벽과 진열대에서는 solid_margin_m, 다른 구역 경계에서는 margin_m ----
def test_paths_keep_the_solid_margin_from_walls_and_shelves(layout_plan):
    layout, plan = layout_plan
    site = load_fleet_map(layout)
    need = plan.params.robot_radius_m + plan.params.solid_margin_m
    x0, y0, x1, y1 = site.bounds()
    for path in plan.paths():
        for x, y in rounded(path, plan.params.turn_radius_m):
            assert min(x - x0, x1 - x, y - y0, y1 - y) >= need - 1e-9, (x, y)
            for r in (*site.shelves, *site.sim_shelves):
                gap = math.hypot(max(r[0] - x, 0.0, x - r[2]), max(r[1] - y, 0.0, y - r[3]))
                assert gap >= need - 1e-9, (x, y, r)


def test_the_solid_margin_is_wider_than_the_zone_margin():
    p = Params()
    assert p.solid_margin_m >= 0.03 > p.margin_m


def _shelf_site():
    zones = {"HALL": SimpleNamespace(id="HALL", rects=((0.0, 0.0, 2.0, 0.79),)),
             "NORTH": SimpleNamespace(id="NORTH", rects=((0.0, 0.79, 2.0, 1.0),))}
    return SimpleNamespace(bounds=lambda: (0.0, 0.0, 2.0, 1.0), zones=zones,
                           shelves=((0.9, 0.3, 1.1, 0.34),), sim_shelves=(), keepout_extra=())


def test_path_clear_keeps_farther_from_a_shelf_than_from_a_zone_border():
    grid = CleanGrid(_shelf_site(), Params())
    # 구역 경계(y 0.79)에서 0.10은 되고, 진열대 윗면(y 0.34)과 벽에서 0.10은 안 된다
    assert path_clear(grid, [(0.3, 0.69), (0.8, 0.69)])
    assert not path_clear(grid, [(0.95, 0.44), (1.05, 0.44)])
    assert path_clear(grid, [(0.95, 0.46), (1.05, 0.46)])
    assert not path_clear(grid, [(0.3, 0.10), (0.8, 0.10)])
    assert path_clear(grid, [(0.3, 0.12), (0.8, 0.12)])


# ---- 정확한 거리 검사: 칸 중심이 아니라 경로의 모든 점 ----
def _fake_site():
    zones = {"HALL": SimpleNamespace(id="HALL", rects=((0.0, 0.0, 2.0, 0.79),)),
             "NORTH": SimpleNamespace(id="NORTH", rects=((0.0, 0.79, 2.0, 1.0),))}
    return SimpleNamespace(bounds=lambda: (0.0, 0.0, 2.0, 1.0), zones=zones, shelves=(),
                           sim_shelves=(), keepout_extra=((1.0, 0.3, 1.2, 0.5),))


def test_path_clear_sees_points_off_cell_centre():
    grid = CleanGrid(_fake_site(), Params())
    # 칸 중심(0.6875)은 0.1025 떨어지지만 y 0.699는 0.091
    assert not path_clear(grid, [(0.3, 0.699), (0.8, 0.699)])
    assert path_clear(grid, [(0.3, 0.69), (0.8, 0.69)])


def test_path_clear_sees_segment_between_samples():
    grid = CleanGrid(_fake_site(), Params())
    # 양 끝은 모서리 (1.0, 0.3)에서 0.117, 가운데는 0.085
    c, d, t = (1.0, 0.3), 0.085 / math.sqrt(2.0), 0.08 / math.sqrt(2.0)
    a = (c[0] - d - t, c[1] - d + t)
    b = (c[0] - d + t, c[1] - d - t)
    assert not path_clear(grid, [a, b])


def test_path_clear_sees_short_exit_from_hall():
    zones = {"HALL": SimpleNamespace(id="HALL", rects=((0.0, 0.0, 0.9995, 1.0),
                                                       (1.0005, 0.0, 2.0, 1.0)))}
    site = SimpleNamespace(bounds=lambda: (0.0, 0.0, 2.0, 1.0), zones=zones, shelves=(),
                           sim_shelves=(), keepout_extra=())
    grid = CleanGrid(site, Params())
    assert not path_clear(grid, [(0.999, 0.5), (1.001, 0.5)])
    assert path_clear(grid, [(0.3, 0.5), (0.9, 0.5)])


# ---- 저장된 좌표로 다시 둥글게 바꿔도 검사 통과 ----
def test_saved_paths_pass_check(layout_plan):
    layout, plan = layout_plan
    data = yaml.safe_load(data_path(layout).read_text(encoding="utf-8"))
    paths = [t["points"] for t in data["transits"]]
    paths += [[line["start"], line["end"]] for line in data["lines"]]
    for points in paths:
        pts = [tuple(p) for p in points]
        assert path_clear(plan.grid, rounded(pts, plan.params.turn_radius_m)), points


# ---- 줄 간격이 격자보다 작으면 거절, 검사 실패면 파일을 쓰지 않음 ----
def test_spacing_below_grid_is_rejected():
    with pytest.raises(ValueError):
        make_plan(load_fleet_map("v7"), Params(line_spacing_m=0.01))


def test_write_refuses_when_paths_fail(monkeypatch, tmp_path):
    target = tmp_path / "coverage_v7.yaml"
    monkeypatch.setattr(cov, "rounded_problems", lambda plan: ["bad path"])
    monkeypatch.setattr(cov, "data_path", lambda layout: target)
    assert cov.main(["--layout", "v7", "--write"]) == 1
    assert not target.exists()


# ---- pode_fsm rounded와 같은 점, 같은 반지름 ----
CORNER_PATHS = [
    [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
    [(0.0, 0.0), (0.5, 0.0), (0.6, 0.3), (0.2, 0.35), (0.25, 0.9)],
    [(0.0, 0.0), (0.3, 0.02), (0.6, 0.0), (0.6, 0.05), (0.0, 0.05)],
    [(0.1, 0.1), (0.4, 0.4), (0.4, 0.4), (0.9, 0.1), (1.5, 0.12), (1.5, 0.5)],
    [(0.0, 0.0), (0.05, 0.0), (0.05, 0.05)],
    [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0001)],
]


@pytest.mark.parametrize("points", CORNER_PATHS)
@pytest.mark.parametrize("radius", [0.0, 0.05, 0.10, 0.25])
def test_rounded_matches_pode_fsm(points, radius):
    theirs = _load("geometry").rounded(points, radius)
    ours = rounded(points, radius)
    assert len(ours) == len(theirs)
    assert all(math.dist(a, b) < 1e-12 for a, b in zip(ours, theirs, strict=True))


def test_turn_radius_matches_pode_fsm():
    assert Params().turn_radius_m == _load("params").FsmParams().turn_radius_m


# ---- 이동 구간: 막힌 것 옆을 지나칠 때는 way_slack_m을 더 둔다(틈처럼 그럴 자리가 없으면 한가운데로) ----
SHELF_A, S_WEST, S_EAST = load_fleet_map("v7").shelves[:3]
GAP_X = 1.1375  # 남쪽 진열대 틈 가운데에 가장 가까운 칸 중심


def _rect_gap(rect, path):
    return min(cov._segment_rect(a, b, rect) for a, b in zip(path, path[1:], strict=False))


def _between_the_shelf_faces(path):
    """경로를 5 mm마다 찍어 남쪽 진열대 두 면(y) 사이에 든 점의 x."""
    out = []
    for (ax, ay), (bx, by) in zip(path, path[1:], strict=False):
        n = max(1, math.ceil(math.dist((ax, ay), (bx, by)) / 0.005))
        for k in range(n + 1):
            x, y = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
            if S_EAST[1] <= y <= S_EAST[3]:
                out.append(x)
    return out


def _way(a, b, **changes):
    """v7에서 a→b 이동 구간: (꺾은선, 둥글게 바꾼 경로)."""
    params = Params(**changes)
    grid = CleanGrid(load_fleet_map("v7"), params)
    pts = cov.transit(grid, grid.free, a, b, params.turn_radius_m)
    path = cov.rounded(pts, params.turn_radius_m)
    assert cov.path_clear(grid, path)
    return pts, path


@pytest.mark.parametrize(("a", "b"), [((1.1375, 0.4625), (1.3125, 0.2125)),
                                      ((1.3125, 0.2125), (1.0625, 0.5125)),
                                      ((0.9125, 0.2125), (1.2125, 0.5125))])
def test_a_way_through_the_shelf_gap_goes_down_its_middle(a, b):
    _, path = _way(a, b)
    # 틈에서 빈칸은 x = GAP_X 한 줄이다: 진열대 두 면 사이에서는 그 줄 위로만 지난다
    inside = _between_the_shelf_faces(path)
    assert inside and max(abs(x - GAP_X) for x in inside) <= 0.003


@pytest.mark.parametrize(("a", "b", "shelf"), [
    ((0.5375, 0.6625), (0.9125, 0.5875), SHELF_A),   # 서쪽 넓은 곳 위쪽에서 위 통로로
    ((0.5625, 0.3375), (0.9125, 0.5375), S_WEST),    # 서쪽 넓은 곳 아래쪽에서 위 통로로
    ((0.9125, 0.5375), (0.5625, 0.3375), S_WEST)])
def test_a_way_round_a_shelf_end_keeps_the_extra_slack(a, b, shelf):
    wide = Params().robot_radius_m + Params().solid_margin_m + Params().way_slack_m
    pts, _ = _way(a, b)
    assert _rect_gap(shelf, pts) >= wide - 1e-6


def test_without_the_slack_a_way_hugs_the_shelf_end():
    _, path = _way((1.1375, 0.4625), (1.3125, 0.2125), way_slack_m=0.0)
    assert max(abs(x - GAP_X) for x in _between_the_shelf_faces(path)) > 0.01
    assert _rect_gap(S_EAST, path) < 0.12


def test_every_transit_of_the_plan_keeps_clear_after_rounding(layout_plan):
    _, plan = layout_plan
    assert cov.rounded_problems(plan) == []
    for _, _, pts in plan.transits:
        assert cov.path_clear(plan.grid, cov.rounded(pts, plan.params.turn_radius_m))


# ---- 막다른 자리: 한 점 둘레를 지나지 않고는 출발점에 닿지 못하는 빈칸 ----
def _pocket(centre, radius=0.18):
    site = load_fleet_map("v7")
    plan = cov.make_plan(site)
    return plan.grid, cov.pocket(plan.grid, plan.grid.free, plan.start, centre, radius)


def test_the_cells_behind_q2_are_shut_in_by_it():
    grid, shut = _pocket((1.66, 0.15))
    inside = [grid.centre((i, j)) for j, i in zip(*shut.nonzero())]
    assert (1.8625, 0.2125) in [(round(x, 4), round(y, 4)) for x, y in inside]
    # 모두 Q2 동쪽 구석이고 Q2 둘레 밖이다
    assert all(x > 1.66 and y < 0.375 and math.dist((x, y), (1.66, 0.15)) > 0.18 for x, y in inside)
    i, j = grid.cell((1.4125, 0.2125))
    assert grid.free[j, i] and not shut[j, i]


ALONE = (0.6125, 0.2875)  # v7: 남쪽 서쪽 진열대 모서리 옆에 홀로 남은 빈칸(이웃 네 칸이 막힘)


def test_a_free_cell_that_cannot_reach_the_start_at_all_is_in_no_pocket():
    site = load_fleet_map("v7")
    plan = cov.make_plan(site)
    grid = plan.grid
    i, j = grid.cell(ALONE)
    assert grid.free[j, i] and plan.unreachable_cells == 1
    assert not np.isfinite(cov.distances(grid.free, grid.cell(plan.start)))[j, i]
    for centre in ((1.66, 0.15), site.node_pose("O_IN"), site.node_pose("E-2")):
        assert not cov.pocket(grid, grid.free, plan.start, centre, 0.18)[j, i]


def test_o_in_shuts_in_nothing_since_the_middle_aisle_leaves_by_a_diagonal_past_it():
    """가운데 통로 서쪽 출구는 대각선 한 걸음(가운데 점이 O_IN에서 0.184 m)으로만 이어진다."""
    site = load_fleet_map("v7")
    plan = cov.make_plan(site)
    o_in = site.node_pose("O_IN")
    assert not cov.pocket(plan.grid, plan.grid.free, plan.start, o_in, 0.18).any()
    # 둘레가 그 가운데 점을 덮으면 두 통로가 다 막다른 쪽이다
    shut = cov.pocket(plan.grid, plan.grid.free, plan.start, o_in, 0.185)
    for p in ((1.1375, 0.5375), (1.4125, 0.2375)):
        i, j = plan.grid.cell(p)
        assert shut[j, i]


def test_the_whole_south_aisle_is_shut_in_by_the_shelf_gap():
    grid, shut = _pocket((1.1125, 0.2125))
    for x in (0.8125, 1.4125, 1.8625):
        i, j = grid.cell((x, 0.2125))
        assert grid.free[j, i] and shut[j, i]
    i, j = grid.cell((1.1125, 0.5125))
    assert grid.free[j, i] and not shut[j, i]


def test_no_cell_is_shut_in_when_the_start_is_inside_the_circle():
    site = load_fleet_map("v7")
    plan = cov.make_plan(site)
    assert not cov.pocket(plan.grid, plan.grid.free, plan.start, plan.start, 0.18).any()


def test_the_start_point_itself_decides_whether_it_is_inside_the_circle():
    """출발점은 Q2 둘레 안(0.168 m)이고 그 칸의 중심은 밖(0.181 m)이다: 막다른 칸은 없다."""
    site = load_fleet_map("v7")
    plan = cov.make_plan(site)
    start, q2 = (1.826, 0.176), (1.66, 0.15)
    assert math.dist(start, q2) < 0.18 < math.dist(plan.grid.centre(plan.grid.cell(start)), q2)
    assert not cov.pocket(plan.grid, plan.grid.free, start, q2, 0.18).any()


def test_a_start_point_outside_the_circle_counts_even_when_its_cell_centre_is_inside():
    """출발점은 Q2 둘레 밖(0.189 m) 동쪽 구석이고 그 칸의 중심은 안(0.178 m)이다: 구석 밖이 막다른 쪽이다."""
    site = load_fleet_map("v7")
    plan = cov.make_plan(site)
    grid, start, q2 = plan.grid, (1.849, 0.151), (1.66, 0.15)
    assert math.dist(grid.centre(grid.cell(start)), q2) < 0.18 < math.dist(start, q2)
    shut = cov.pocket(grid, grid.free, start, q2, 0.18)
    i, j = grid.cell((1.4125, 0.2125))
    assert shut[j, i]
    i, j = grid.cell((1.8625, 0.2125))
    assert grid.free[j, i] and not shut[j, i]
