"""건넘 구간(mover_cross_prob) 평가 집계(시뮬레이터 없이): 에피소드 표시와 표시별 결과 비율."""

from __future__ import annotations

import numpy as np
import torch


def episode_flags(mover_legs, ids) -> torch.Tensor:
    """(n, 2) bool: 나온 mover 중 건넘 구간이 있음, 그 건넘 구간이 우리 전역 경로를 덮음."""
    cross = torch.zeros(len(ids), dtype=torch.bool, device=mover_legs[0]["active"].device)
    on_path = torch.zeros_like(cross)
    for legs in mover_legs:
        act = legs["active"][ids]
        cross |= act & legs["cross"][ids]
        on_path |= act & legs["cross_path"][ids]
    return torch.stack((cross, on_path), -1)


def rates_by_cross(codes, cross, cross_path, keys) -> dict:
    """결과 코드 (E,) 를 나눈 에피소드 수와 비율.
    나눔: 건넘이 경로를 덮음, 건넘이 경로 밖, 건넘 없음."""
    codes = np.asarray(codes)
    cross = np.asarray(cross, dtype=bool)
    cross_path = np.asarray(cross_path, dtype=bool)
    groups = {
        "cross_on_path": cross & cross_path,
        "cross_off_path": cross & ~cross_path,
        "no_cross": ~cross,
    }
    out = {}
    for name, sel in groups.items():
        m = max(int(sel.sum()), 1)
        out[name] = {
            "episodes": int(sel.sum()),
            **{k: round(float(((codes == i) & sel).sum()) / m, 4) for i, k in enumerate(keys)},
        }
    return out
