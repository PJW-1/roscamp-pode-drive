"""청소 로봇이 좁은 통로에서 운반 로봇 길을 막으면 한쪽 면으로 붙어 서서 옆으로 지나가게 한다.

교통 층(ZoneTraffic)만 v7 지도로 세우고 보고와 결과를 손으로 넣는다(test_zone_cleaning의 World).
"""
from __future__ import annotations

import math
from dataclasses import replace

import pytest

from fleet_core.common.commands import RetreatCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.zone.cleaning import LINE, TUCK, YIELD
from test_zone_cleaning import CLEANER_NAME, World

LINE_Y, TUCK_Y = 0.2125, 0.2615  # 남쪽 통로 위쪽 줄, 진열대 면(0.3475)에서 3 cm 띄운 자리
WAY = [(0.64, 0.24), (1.66, 0.15)]  # 운반 로봇 길: E-3에서 Q2로
S = 0.05  # 줄 시작에서 이만큼 간 자리: 붙어 설 자리가 Q2 둘레 밖이다


def _south_line(w):
    """남쪽 통로 위쪽 줄 가운데 동쪽으로 가는 동쪽 줄(충전 칸의 운반 로봇에서 멀다)."""
    lines = w.t._clean_plan.lines
    return max((k for k, (a, b) in enumerate(lines)
                if a[0] < b[0] and abs(a[1] - LINE_Y) < 1e-6 and abs(b[1] - LINE_Y) < 1e-6),
               key=lambda k: lines[k][0][0])


def _on_the_line(monkeypatch, s=S, params=None):
    """청소 로봇은 남쪽 통로 줄 위 s m 지점을 달리고, pode1은 P-3에 서서 그 통로로 갈 길을 받았다."""
    w = World({"pode1": "P-3", CLEANER_NAME: "P-1"}, params)
    k, a, b, _ = w.to_line(_south_line(w))
    w.pos[CLEANER_NAME] = w.at(a, b, s)
    w.step()
    t = w.t
    monkeypatch.setattr(t, "_clean_band", lambda other, full=False: list(WAY))
    monkeypatch.setattr(t, "_wanted", lambda other: [])
    return w, t, t._robots[CLEANER_NAME], t._robots["pode1"], k, a, b


def _arrive(w, cmd):
    w.pos[CLEANER_NAME] = cmd.path[-1][:2]
    w.state[CLEANER_NAME] = "IDLE"
    w.report()
    w.finish(cmd.goal_id)


# ---- 붙어 서러 가기 ----
def test_blocking_a_transport_in_the_south_aisle_tucks_to_the_shelf_side(monkeypatch):
    w, t, m, other, k, a, b = _on_the_line(monkeypatch)
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    cmd = w.moves("follow_path")[-1]
    pts = [pt[:2] for pt in cmd.path]
    here = w.at(a, b, S)
    assert cmd.target_id == TUCK and w.job.leg.kind == YIELD
    assert pts[0] == pytest.approx(here) and pts[-1][1] == pytest.approx(TUCK_Y)
    assert pts[-1][0] > here[0] and cmd.path[-1][2] == pytest.approx(0.0)  # 가던 쪽으로, 통로와 나란히
    assert not [o for o in w.out if isinstance(o, RetreatCmd) and o.robot == CLEANER_NAME]
    assert any("tucks in for pode1" in e.detail for e in w.events("CLEAN_YIELD"))


def test_the_tucked_cleaner_stands_while_the_transport_still_meets_it(monkeypatch):
    w, t, m, other, k, a, b = _on_the_line(monkeypatch)
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    cmd = w.moves("follow_path")[-1]
    _arrive(w, cmd)
    sent = len(w.moves())
    for _ in range(round((w.params.clean_yield_wait_s + 5.0) / 0.1)):
        w.step()
    assert len(w.moves()) == sent and not w.events("CLEAN_HOME")
    assert w.job.tuck is not None and w.job.goal_id is None


def test_a_transport_that_never_gets_past_sends_the_tucked_cleaner_home(monkeypatch):
    w, t, m, other, k, a, b = _on_the_line(monkeypatch)
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    _arrive(w, w.moves("follow_path")[-1])
    for _ in range(round((w.params.clean_tuck_wait_s + 2.0) / 0.1)):
        w.step()
    assert w.events("CLEAN_HOME")


# ---- 줄로 돌아가기 ----
def test_after_the_transport_is_gone_the_line_goes_on_from_ahead_of_the_tuck(monkeypatch):
    w, t, m, other, k, a, b = _on_the_line(monkeypatch)
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    cmd = w.moves("follow_path")[-1]
    spot = cmd.path[-1][:2]
    back = w.job.segs[k][0]
    here = w.at(a, b, S)
    # 복귀점은 줄 위, 붙어 선 자리보다 앞이다
    assert back[1] == pytest.approx(LINE_Y) and back[0] > spot[0] > here[0]
    assert w.job.queue[0] == k and w.job.tuck_line == k
    _arrive(w, cmd)
    monkeypatch.setattr(t, "_clean_band", lambda other, full=False: None)
    for _ in range(5):
        w.step()
    again = w.moves("follow_path")[-1]
    pts = [pt[:2] for pt in again.path]
    assert w.job.leg.kind == LINE and w.job.leg.line == k
    assert pts[0] == pytest.approx(spot) and pts[1] == pytest.approx(back)
    assert pts[-1] == pytest.approx(b) and len(pts) == 3
    assert w.job.tuck is None


def test_the_skipped_stretch_near_the_tuck_line_counts_as_cleaned(monkeypatch):
    """줄에서 4.9 cm 옆으로 붙어 지난 구간은 청소 폭(반폭 5.6 cm) 안이라 빈틈으로 남기지 않는다."""
    w, t, m, other, k, a, b = _on_the_line(monkeypatch)
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    half = t._clean_plan.params.clean_width_m / 2
    assert TUCK_Y - LINE_Y < half
    left = sum(math.dist(*gap) for gap in w.job.gaps)
    skipped = w.job.segs[k][0][0] - w.at(a, b, S)[0]
    assert left < skipped  # 건너뛴 구간 전체가 빈틈이 되지는 않는다


# ---- 붙지 않는 때 ----
def test_no_tuck_when_the_rows_parameter_is_zero(monkeypatch):
    params = replace(load_params(), clean_tuck_rows=0)
    w, t, m, other, k, a, b = _on_the_line(monkeypatch, params=params)
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    assert w.job.tuck is None and not any("tucks in" in e.detail for e in w.events("CLEAN_YIELD"))


def test_no_tuck_when_the_transport_is_already_close(monkeypatch):
    w, t, m, other, k, a, b = _on_the_line(monkeypatch)
    here = w.at(a, b, S)
    w.pos["pode1"] = (here[0] + 0.35, 0.10)  # 붙으러 가는 길에서 clean_yield_near_m 안
    w.step()
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    assert w.job.tuck is None


def test_no_tuck_next_to_a_node_the_transport_will_pass(monkeypatch):
    w, t, m, other, k, a, b = _on_the_line(monkeypatch)
    monkeypatch.setattr(t, "_wanted", lambda other: ["Q2"])
    w.pos[CLEANER_NAME] = w.at(a, b, 0.15)  # 붙어 설 자리가 Q2(1.66, 0.15) 둘레에 든다
    w.step()
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    assert w.job.tuck is None


def test_a_failed_tuck_is_not_tried_again_for_the_same_transport(monkeypatch):
    w, t, m, other, k, a, b = _on_the_line(monkeypatch)
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    cmd = w.moves("follow_path")[-1]
    # 가다가 멈춰 실패: 붙어 설 자리에서 멀다
    w.state[CLEANER_NAME] = "IDLE"
    w.report()
    w.finish(cmd.goal_id, success=False, reason="NAV_ABORTED")
    w.now += w.params.move_retry_wait_s + 0.1
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    tucks = [e for e in w.events("CLEAN_YIELD") if "tucks in" in e.detail]
    assert len(tucks) == 1


def test_no_tuck_when_the_lane_would_cover_a_node_the_transport_must_reach(monkeypatch):
    """붙어 선 몸에서 운반 로봇 쪽으로 찍히는 띠가 그 로봇이 갈 틈 아래 지점(GAP_S)을 덮는다."""
    w, t, m, other, k, a, b = _on_the_line(monkeypatch)
    monkeypatch.setattr(t, "_wanted", lambda other: ["GAP_S"])
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    assert w.job.tuck is None


def test_the_lane_check_is_off_with_no_lane(monkeypatch):
    params = replace(load_params(), clean_tuck_lane_m=0.0)
    w, t, m, other, k, a, b = _on_the_line(monkeypatch, params=params)
    monkeypatch.setattr(t, "_wanted", lambda other: ["GAP_S"])
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    assert w.job.tuck is not None
