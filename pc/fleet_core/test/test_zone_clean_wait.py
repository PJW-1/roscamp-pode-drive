"""청소 로봇이 운반 로봇과 겹칠 때: 다음 줄 앞에서 기다리기, 오래 막히면 미루기,
못 한 줄은 집에 가지 않고 다음 바퀴 앞에 두기, 길을 막으면 뒤로 빠지기.

교통 층(ZoneTraffic)만 v7 지도로 세우고 보고와 결과를 손으로 넣는다(test_zone_cleaning의 World).
"""
from __future__ import annotations

import math

from fleet_core.common.commands import RetreatCmd
from fleet_core.traffic.zone.cleaning import BACK, LINE, LINES, YIELD, _path_gap
from test_zone_cleaning import CLEANER_NAME, World


def _out_done():
    """운반 pode1은 P-3에 서 있고, 청소 로봇은 나가기를 끝내 첫 줄 앞에 있다."""
    w = World({"pode1": "P-3", CLEANER_NAME: "P-1"})
    w.step()
    out = w.moves()[-1]
    w.pos[CLEANER_NAME] = (out.x, out.y)
    w.report()
    w.finish(out.goal_id)
    assert w.job.stage == LINES
    return w


def _head_blocked(w):
    """동쪽 끝 줄 k를 맨 앞에 두고 pode1을 그 줄 가운데 0.10 m 옆에 세운다(R3, 청소 로봇에서 멀다)."""
    job = w.job
    k = max(job.queue, key=lambda i: job.segs[i][0][0])
    job.queue = [k] + [i for i in job.queue if i != k]
    a, b = job.segs[k]
    w.pos["pode1"] = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + 0.10)
    return k


def _clear(w):
    w.pos["pode1"] = w.site.node_pose("P-3")


def _lines_sent(w):
    return [e for e in w.events("CLEAN_SENT") if e.detail.startswith("line ")]


def _run(w, seconds):
    for _ in range(round(seconds / 0.1)):
        w.step()


# ---- 기다리기: 다른 줄로 건너뛰지 않는다 ----
def test_a_line_crossing_a_held_node_waits_instead_of_jumping():
    w = _out_done()
    k = _head_blocked(w)
    _run(w, 5.0)
    assert not _lines_sent(w)
    waits = w.events("CLEAN_LINE_WAIT")
    assert len(waits) == 1 and waits[0].detail == f"line {k}: pode1 stands near"
    assert not w.events("CLEAN_LINE_DEFERRED") and w.job.queue[0] == k


def test_the_line_goes_as_soon_as_the_node_is_free():
    w = _out_done()
    k = _head_blocked(w)
    _run(w, 3.0)
    _clear(w)
    _run(w, w.params.clean_wait_check_s + 0.2)
    sent = _lines_sent(w)
    assert sent and sent[0].detail.startswith(f"line {k} ")


def test_a_long_wait_defers_the_line_and_goes_on():
    w = _out_done()
    k = _head_blocked(w)
    _run(w, w.params.clean_wait_max_s + 1.0)
    deferred = w.events("CLEAN_LINE_DEFERRED")
    assert deferred and deferred[0].detail.startswith(f"line {k} (1): ")
    sent = _lines_sent(w)
    assert sent and not sent[0].detail.startswith(f"line {k} ")
    assert k in w.job.waiting


def test_a_path_that_never_clears_is_deferred_at_once(monkeypatch):
    w = _out_done()
    k = w.job.queue[0]
    real = w.t._line_leg
    monkeypatch.setattr(w.t, "_line_leg",
                        lambda m, job, i: None if i == k else real(m, job, i))
    w.step()
    assert not w.events("CLEAN_LINE_WAIT")
    assert w.events("CLEAN_LINE_DEFERRED")[0].detail.startswith(f"line {k} (1): ")
    assert _lines_sent(w)


# ---- 못 한 줄: 집에 가지 않고 다음 바퀴 맨 앞 ----
def _lap_end(w, skipped):
    job = w.job
    job.queue, job.waiting = [], []
    job.skipped = list(skipped)
    job.done = [k for k in range(len(job.segs)) if k not in skipped]


def test_a_lap_with_skipped_lines_starts_the_next_lap_from_them():
    w = _out_done()
    _lap_end(w, [3, 7])
    w.step()
    assert w.events("CLEAN_LAP_DONE") and "skipped [3, 7]" in w.events("CLEAN_LAP_DONE")[0].detail
    starts = w.events("CLEAN_LAP_START")
    assert len(starts) == 2 and starts[-1].detail.startswith("lap 2,")
    assert w.job.stage == LINES and w.job.lap == 2
    sent = _lines_sent(w)
    assert sent and sent[0].detail.startswith("line 3 ")
    assert w.job.queue[:1] == [7] and len(w.job.queue) == len(w.job.segs) - 1
    assert not [e for e in w.events("CLEAN_SENT") if e.detail.startswith("back")]


def test_a_full_lap_still_goes_home():
    w = _out_done()
    _lap_end(w, [])
    w.step()
    assert w.job.stage == BACK
    assert [e for e in w.events("CLEAN_SENT") if e.detail.startswith("back")]


# ---- 뒤로 빠지기: 길을 막으면 몸 방향 그대로 후진해 띠 밖으로 ----
def _on_line_facing_a_band(w, monkeypatch, band):
    t, m = w.t, w.t._robots[CLEANER_NAME]
    w.pos[CLEANER_NAME] = (1.0, 0.5625)  # 가운데 영역 윗줄 위, 몸은 동쪽(yaw 0)을 본다
    w.pos["pode1"] = band[0]  # 띠는 운반 로봇 자리에서 시작한다
    w.step()
    monkeypatch.setattr(t, "_clean_band", lambda other, full=False: list(band))
    monkeypatch.setattr(t, "_wanted", lambda other: [])
    return t, m, t._robots["pode1"]


def test_blocking_a_transport_backs_straight_out_of_its_way(monkeypatch):
    w = _out_done()
    band = [(1.15, 0.40), (1.15, 0.70)]  # 0.15 m 앞을 지나는 운반 로봇 길
    t, m, other = _on_line_facing_a_band(w, monkeypatch, band)
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    backs = [o for o in w.out if isinstance(o, RetreatCmd) and o.robot == CLEANER_NAME]
    assert backs, "no reverse command"
    end = backs[-1].path[-1][:2]
    p = w.params
    assert end[1] == 0.5625 and end[0] < 1.0  # 같은 줄 위로 뒤로
    assert 1.0 - end[0] <= p.clean_backup_max_m + 1e-9
    assert _path_gap(end, band) > t._band_reach() + p.clean_yield_margin_m
    assert w.job.leg.kind == YIELD
    assert any("backs up" in e.detail for e in w.events("CLEAN_YIELD"))


def test_no_room_behind_falls_back_to_the_yield_point(monkeypatch):
    w = _out_done()
    band = [(0.70, 0.60), (1.30, 0.60)]  # 줄을 따라 바로 위를 지나는 길: 뒤로 가도 띠 안
    t, m, other = _on_line_facing_a_band(w, monkeypatch, band)
    assert t._clean_make_way(m, other, "test")
    w.out += t.take_output()
    assert not [o for o in w.out if isinstance(o, RetreatCmd) and o.robot == CLEANER_NAME]
    assert w.job.leg is not None and w.job.leg.kind == YIELD
    assert math.dist(w.job.leg.points[-1], (1.0, 0.5625)) > 0.0


def test_after_backing_up_the_line_goes_on_from_its_progress():
    w = World({"pode1": "P-2", CLEANER_NAME: "P-1"})
    k, a, b, line = w.to_line()
    assert a[0] < b[0]  # 보고 yaw 0(동쪽)이 줄 방향이다
    w.pos[CLEANER_NAME] = w.at(a, b, 0.3)
    w.pos["pode1"] = w.at(a, b, 0.5)
    w.step()
    t, job = w.t, w.job
    assert t._clean_make_way(t._robots[CLEANER_NAME], t._robots["pode1"], "test")
    w.out += t.take_output()
    back = [o for o in w.out if isinstance(o, RetreatCmd) and o.robot == CLEANER_NAME][-1]
    end = back.path[-1][:2]
    assert math.dist(end, w.pos["pode1"]) > t._band_reach() + w.params.clean_yield_margin_m
    assert job.queue[0] == k and math.dist(job.segs[k][0], w.at(a, b, 0.3)) < 1e-6
    # 운반 로봇이 떠나면 진행점부터 그 줄을 잇는다
    w.pos["pode1"] = w.site.node_pose("P-2")
    w.pos[CLEANER_NAME] = end
    w.state[CLEANER_NAME] = "IDLE"
    w.step()
    t.on_retreat_result(CLEANER_NAME, back.goal_id, True)
    w.out += t.take_output()
    for _ in range(3):
        w.step()
    again = w.moves("follow_path")[-1]
    assert job.leg is not None and job.leg.kind == LINE and job.leg.line == k
    pts = [pt[:2] for pt in again.path]
    assert any(math.dist(pt, w.at(a, b, 0.3)) < 1e-6 for pt in pts)
    assert math.dist(pts[-1], b) < 1e-6
