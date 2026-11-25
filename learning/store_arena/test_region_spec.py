"""영역 테스트(시뮬 없이): v7 의 통로, 틈, 열린 곳을 성질로 확인한다."""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arena_spec as A  # noqa: E402
import region_spec as G  # noqa: E402


@pytest.fixture(scope="module")
def v7():
    site = A.load_site(A.ARENA)
    boxes = A.arena_boxes(A.ARENA, site)
    rects = np.array([(b.x0, b.y0, b.x1, b.y1) for b in boxes])
    shelves = [b for b in boxes if b.kind == "shelf"]
    shelf_a = max(shelves, key=lambda b: b.x1 - b.x0)  # 가장 긴 선반 = 북쪽 선반 A
    # 남쪽 선반 둘: A 다음으로 긴, 같은 높이의 두 선반
    south = sorted(sorted(shelves, key=lambda b: b.x1 - b.x0)[-3:-1], key=lambda b: b.x0)
    return site, rects, G.build_regions(site.bounds(), rects), shelf_a, south


def _aisles(reg):
    return [reg.regions[i] for i in reg.ids("aisle")]


def test_south_shelves_are_a_pair_below_a(v7):
    _, _, _, a, (s1, s2) = v7
    assert s1.y0 == s2.y0 and s1.y1 == s2.y1 and s1.y1 < a.y0 and s1.x1 < s2.x0


def test_middle_aisle_lies_between_the_south_shelves_and_a(v7):
    _, _, reg, a, (s1, s2) = v7
    mid = [r for r in _aisles(reg) if r.rect[1] == pytest.approx(s1.y1) and
           r.rect[3] == pytest.approx(a.y0)]
    assert len(mid) == 1
    m = mid[0]
    assert m.axis == 0 and m.width == pytest.approx(0.35, abs=0.01)
    assert m.rect[0] == pytest.approx(s1.x0) and m.rect[2] == pytest.approx(min(s2.x1, a.x1))
    assert m.openings == ((pytest.approx(s1.x1), pytest.approx(s2.x0)),)


def test_south_aisle_and_north_lane(v7):
    site, _, reg, a, (s1, s2) = v7
    x0, y0, x1, y1 = site.bounds()
    south = [r for r in _aisles(reg) if r.rect[1] == pytest.approx(y0)]
    assert len(south) == 1 and south[0].rect[3] == pytest.approx(s1.y0)
    assert south[0].width == pytest.approx(0.37, abs=0.01)
    north = [r for r in _aisles(reg) if r.rect[3] == pytest.approx(y1)]
    assert len(north) == 1 and north[0].rect[1] == pytest.approx(a.y1)
    assert north[0].width == pytest.approx(0.19, abs=0.01)
    assert north[0].rect[0] == pytest.approx(a.x0) and north[0].rect[2] == pytest.approx(a.x1)
    assert len(_aisles(reg)) == 3  # 열린 곳 쪽 선반 끝과 벽 사이는 통로 아님


def test_gap_between_the_south_shelves_is_an_opening_not_an_aisle(v7):
    _, _, reg, _, (s1, s2) = v7
    ops = [reg.regions[i] for i in reg.ids("opening")]
    assert len(ops) == 1
    o = ops[0].rect
    assert o == pytest.approx((s1.x1, s1.y0, s2.x0, s1.y1))
    k = reg.index_of(((o[0] + o[2]) / 2, (o[1] + o[3]) / 2))
    assert reg.regions[k].kind == "opening"


def test_open_areas_west_and_east_of_the_shelves(v7):
    site, rects, reg, a, (s1, s2) = v7
    x0, y0, x1, y1 = site.bounds()
    opens = reg.ids("open")
    assert len(opens) == 2
    west = reg.index_of(((x0 + min(a.x0, s1.x0)) / 2, (y0 + y1) / 2))
    east = reg.index_of(((x1 + max(a.x1, s2.x1)) / 2, (y0 + y1) / 2))
    assert west in opens and east in opens and west != east
    # 모든 빈 셀은 정확히 한 영역 (장애물 밖이면 -1 이 아님)
    for x in np.arange(x0 + 0.005, x1, 0.02):
        for y in np.arange(y0 + 0.005, y1, 0.02):
            inside = ((rects[:, 0] <= x) & (x <= rects[:, 2]) & (rects[:, 1] <= y)
                      & (y <= rects[:, 3])).any()
            assert inside or reg.index_of((x, y)) >= 0


def test_passing_gap_and_corridor(v7):
    _, _, reg, a, (s1, s2) = v7
    k = [i for i in reg.ids("aisle") if reg.regions[i].width == pytest.approx(0.35, abs=0.01)][0]
    m = reg.regions[k]
    c = 0.5 * (m.rect[1] + m.rect[3])
    path = np.stack((np.linspace(m.rect[0], m.rect[2], 50), np.full(50, c)), -1)
    # 아래 면에 붙은 0.2 m 박스: 위쪽 틈 0.15, 몸체 띠(경로 +- 0.047)에 걸침
    box = np.array((s1.x0 + 0.05, m.rect[1], s1.x0 + 0.25, m.rect[1] + 0.20))
    assert G.passing_gap(reg, box, path, 0.047) == pytest.approx(m.width - 0.20)
    # 아래 면에 붙은 0.1 m 박스: 경로에서 0.075 떨어져 몸체 띠 밖
    small = np.array((s1.x0 + 0.05, m.rect[1], s1.x0 + 0.15, m.rect[1] + 0.10))
    assert np.isnan(G.passing_gap(reg, small, path, 0.047))
    assert np.isnan(G.passing_gap(reg, box + np.array((0, 2, 0, 2)), path, 0.047))
    assert G.path_length_in(path, m.rect) == pytest.approx(m.rect[2] - m.rect[0])
