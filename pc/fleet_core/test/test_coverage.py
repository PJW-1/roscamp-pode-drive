"""청소 로봇 커버리지 경로: data 파일 일치, 둥근 경로 빈칸 검사, pode_fsm 둥글게와 같음, 커버율."""
import importlib.util
import math
import sys
from pathlib import Path

from types import SimpleNamespace

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
from fleet_core.fleet_map import load_fleet_map

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


# ---- 커버율: 키운 격자 빈칸 기준 99 % 이상 ----
def test_clean_grid_coverage(layout_plan):
    _, plan = layout_plan
    _, on_grid = coverage(plan)
    assert on_grid >= 0.99


# ---- 영역 나누기: 줄을 region_width_m 띠 안에서 긋고 한 영역씩 끝낸다 ----
def _region_order(plan):
    return [plan.cells[cell]["region"] for _, _, cell in plan.lines]


def test_lines_stay_inside_one_region(layout_plan):
    _, plan = layout_plan
    for a, b, _ in plan.lines:
        assert abs(b[0] - a[0]) <= plan.params.region_width_m + 1e-9


def test_regions_are_swept_one_after_another(layout_plan):
    _, plan = layout_plan
    seen = []
    for region in _region_order(plan):
        if not seen or seen[-1] != region:
            assert region not in seen, f"comes back to region {region}"
            seen.append(region)


def test_v7_is_cut_into_regions_from_the_home_side():
    plan = make_plan(load_fleet_map("v7"))
    order = list(dict.fromkeys(_region_order(plan)))
    assert len(order) >= 3
    # 집 자리(서쪽)에서 가까운 영역부터
    west = [min(a[0], b[0]) for a, b, _ in plan.lines]
    firsts = [west[_region_order(plan).index(r)] for r in order]
    assert firsts == sorted(firsts)


def test_lines_alone_cover_the_clean_grid(layout_plan):
    """관제는 줄만 덮은 것으로 센다: 이동 구간 없이도 키운 격자를 99 % 이상 덮는다."""
    _, plan = layout_plan
    hit = cov.covered_by(plan.grid, plan.params, [[a, b] for a, b, _ in plan.lines])
    free = plan.grid.free
    assert (hit & free).sum() / free.sum() >= 0.99


def test_region_width_zero_keeps_whole_lines():
    plan = make_plan(load_fleet_map("v7"), Params(region_width_m=0.0))
    assert max(abs(b[0] - a[0]) for a, b, _ in plan.lines) > 1.4
    assert {c["region"] for c in plan.cells} == {0}


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
