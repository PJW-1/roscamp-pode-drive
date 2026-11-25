"""통로 박스 지나갈 틈별 평가 요약 (play.py, 시뮬레이터 없이).

경로별 틈은 route_spec.Routes.obstacle_gap 의 슬롯 중 가장 작은 값 (통로 박스가 경로 위에 없으면 NaN).
"""

from __future__ import annotations

import numpy as np

EDGES = (0.13, 0.16, 0.19, 0.22, 0.25)  # 틈 칸 경계 [m]


def route_gaps(obstacle_gap: np.ndarray) -> np.ndarray:
    """(R, K) 슬롯별 틈에서 경로별 가장 작은 틈 (R,), 없으면 NaN."""
    g = np.where(np.isnan(obstacle_gap), np.inf, obstacle_gap).min(-1)
    return np.where(np.isinf(g), np.nan, g)


def _rates(sel, codes, keys) -> dict:
    n = int(sel.sum())
    return {"episodes": n,
            **{k: round(float(((codes == i) & sel).sum()) / max(n, 1), 4)
               for i, k in enumerate(keys)}}


def bins(gaps: np.ndarray) -> dict:
    """칸 이름 -> 에피소드 선택 (bool): 통로 박스 없음, EDGES 앞, 칸들, EDGES 뒤."""
    g = np.asarray(gaps, dtype=float)
    fin = np.isfinite(g)
    out = {"no aisle box": ~fin, f"< {EDGES[0]:.2f}": fin & (g < EDGES[0])}
    for lo, hi in zip(EDGES[:-1], EDGES[1:], strict=True):
        out[f"{lo:.2f}-{hi:.2f}"] = fin & (g >= lo) & (g < hi)
    out[f">= {EDGES[-1]:.2f}"] = fin & (g >= EDGES[-1])
    return out


def by_gap(gaps, codes, keys) -> dict:
    """칸별 에피소드 수와 결과 비율."""
    codes = np.asarray(codes)
    return {name: _rates(sel, codes, keys) for name, sel in bins(gaps).items()}


def in_aisle_pass(gaps, codes, success_code: int = 0) -> dict:
    """경로 위 통로 박스가 있는 에피소드 중 성공 비율 (전역 경로가 통로 안이라 성공 = 통로 안에서 지나감)."""
    sel = np.isfinite(np.asarray(gaps, dtype=float))
    n = int(sel.sum())
    ok = int((sel & (np.asarray(codes) == success_code)).sum())
    return {"episodes": n, "success": ok, "rate": round(ok / n, 4) if n else None}
