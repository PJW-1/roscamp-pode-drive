"""청소 로봇 커버리지 경로: data 파일 일치, 둥근 경로 빈칸 검사, pode_fsm 둥글게와 같음, 커버율."""
import importlib.util
import math
import sys
from pathlib import Path

import pytest
from fleet_core.coverage import (
    Params,
    coverage,
    data_path,
    make_plan,
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


# ---- 커버율: 키운 격자 빈칸 기준 90 % 이상 ----
def test_clean_grid_coverage(layout_plan):
    _, plan = layout_plan
    _, on_grid = coverage(plan)
    assert on_grid >= 0.90


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
