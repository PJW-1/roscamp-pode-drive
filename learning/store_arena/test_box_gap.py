"""틈별 평가 요약 테스트 (시뮬 없이)."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import box_gap as bg  # noqa: E402

KEYS = ("success", "collision", "oob", "timeout")


def test_route_gaps_take_the_smallest_finite_slot():
    g = np.array([[np.nan, 0.2, 0.15], [np.nan, np.nan, np.nan], [0.3, np.nan, np.nan]])
    out = bg.route_gaps(g)
    assert out[0] == 0.15 and np.isnan(out[1]) and out[2] == 0.3


def test_bins_partition_every_episode_once():
    g = np.array([np.nan, 0.10, 0.13, 0.159, 0.16, 0.2, 0.2499, 0.25, 0.4])
    b = bg.bins(g)
    stack = np.stack(list(b.values()))
    assert (stack.sum(0) == 1).all()
    assert b["0.13-0.16"].tolist() == [False, False, True, True] + [False] * 5
    assert b["0.22-0.25"][6] and b[">= 0.25"][7] and b["< 0.13"][1] and b["no aisle box"][0]


def test_by_gap_and_in_aisle_pass():
    g = np.array([np.nan, 0.14, 0.15, 0.20, np.nan])
    codes = np.array([0, 0, 1, 3, 2])
    out = bg.by_gap(g, codes, KEYS)
    assert out["0.13-0.16"] == {"episodes": 2, "success": 0.5, "collision": 0.5, "oob": 0.0,
                                "timeout": 0.0}
    assert out["no aisle box"]["episodes"] == 2 and out["no aisle box"]["oob"] == 0.5
    assert sum(v["episodes"] for v in out.values()) == len(g)
    assert bg.in_aisle_pass(g, codes) == {"episodes": 3, "success": 1, "rate": 0.3333}
    assert bg.in_aisle_pass(np.full(3, np.nan), codes[:3])["rate"] is None
