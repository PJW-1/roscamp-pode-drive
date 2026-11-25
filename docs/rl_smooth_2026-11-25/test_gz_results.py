"""gz_results.w_rates 시험(합성 명령 계열, ROS 없이)."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gz_results as gr  # noqa: E402

DT = 0.05


def series(ws, t0=0.0, dt=DT):
    return [(t0 + i * dt, 0.1, w) for i, w in enumerate(ws)]


def test_a_ramp_has_constant_acceleration_and_no_jerk():
    acc, jerk = gr.w_rates(series([0.02 * i for i in range(40)]))
    assert acc == pytest.approx(0.4)
    assert jerk == pytest.approx(0.0, abs=1e-9)


def test_alternating_sign_gives_the_closed_form():
    acc, jerk = gr.w_rates(series([(-1) ** i * 0.5 for i in range(21)]))
    assert acc == pytest.approx(1.0 / DT)
    assert jerk == pytest.approx(2.0 / DT ** 2)


def test_a_sine_matches_the_discrete_derivative_amplitude():
    a, f = 0.3, 0.5
    ws = [a * math.sin(2 * math.pi * f * i * DT) for i in range(int(20 / DT))]
    acc, jerk = gr.w_rates(series(ws))
    k = 2 * math.sin(math.pi * f * DT) / DT
    assert acc == pytest.approx(a * k / math.sqrt(2), rel=0.01)
    assert jerk == pytest.approx(a * k * k / math.sqrt(2), rel=0.01)


def test_a_gap_splits_the_series_so_the_jump_across_it_is_not_counted():
    first = series([0.0] * 10)
    second = series([1.0] * 10, t0=first[-1][0] + 4 * DT)
    acc, jerk = gr.w_rates(first + second)
    assert acc == 0.0 and jerk == 0.0
    acc, _ = gr.w_rates(first + series([1.0] * 10, t0=first[-1][0] + DT))
    assert acc == pytest.approx(math.sqrt(1 / DT ** 2 / 19))


def test_too_short_gives_none():
    assert gr.w_rates([]) == (None, None)
    assert gr.w_rates(series([0.4])) == (None, None)
    acc, jerk = gr.w_rates(series([0.0, 0.1]))
    assert acc == pytest.approx(2.0) and jerk is None
