"""cleaner_defer_sweep의 첫 바퀴 자르기."""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cleaner_defer_sweep import first_lap  # noqa: E402


def ev(t, code, detail):
    return (t, SimpleNamespace(code=code, detail=detail, robot="pode3"))


def test_a_second_lap_at_the_same_time_is_not_in_the_first():
    events = [
        ev(0.0, "CLEAN_LAP_START", "lap 1, 2 lines (0 gaps)"),
        ev(1.0, "CLEAN_LINE_DEFERRED", "line 0 deferred 1: pode1 stands near"),
        ev(5.0, "CLEAN_LAP_DONE", "lap 1 in 5.0 s: 2/2 lines, 1 deferrals, skipped [], 0 gaps"),
        ev(5.0, "CLEAN_LAP_START", "lap 2, 2 lines (0 gaps)"),
        ev(5.0, "CLEAN_LINE_DEFERRED", "line 0 deferred 1: pode1 stands near"),
    ]
    part, done = first_lap(events)
    assert [e.code for _, e in part] == ["CLEAN_LAP_START", "CLEAN_LINE_DEFERRED",
                                         "CLEAN_LAP_DONE"]
    assert done[1].detail.startswith("lap 1 ")


def test_an_unfinished_first_lap_runs_to_the_end():
    events = [
        ev(0.0, "CLEAN_LINE_DEFERRED", "line 0 deferred 1: before the lap"),
        ev(1.0, "CLEAN_LAP_START", "lap 1, 2 lines (0 gaps)"),
        ev(2.0, "CLEAN_LINE_DEFERRED", "line 0 deferred 1: pode1 stands near"),
    ]
    part, done = first_lap(events)
    assert len(part) == 2 and done is None


def test_no_lap_start_means_no_events():
    assert first_lap([ev(1.0, "CLEAN_LINE_DEFERRED", "line 0 deferred 1: x")]) == ([], None)
