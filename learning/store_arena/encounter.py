"""플릿 로봇과의 만남 지표(시뮬레이터 없이): 에피소드별 최소 간격, 만남 시간, 교착.

env 가 제어 스텝마다 update 를 부르고 리셋 때 에피소드 값을 넘긴다. play.py 는 summary 로 eval.json 에 쓴다.
간격은 우리 몸체 박스와 로봇 박스 사이 거리 (arena_spec.box_clearance, 겹치면 0).
  만남 시간: 간격이 처음 ENCOUNTER_M 아래로 떨어진 때부터 다시 넘거나 에피소드가 끝날 때까지 [s].
  교착: 우리와 같은 로봇이 둘 다 STILL_MPS 보다 느리고 간격이 ENCOUNTER_M 아래인 상태가 DEADLOCK_S 넘게 이어짐.
"""

from __future__ import annotations

import numpy as np
import torch

# ---- 기준 (한 곳) ----
ENCOUNTER_M = 0.15  # 몸체 간 간격 [m]
STILL_MPS = 0.02  # 이보다 느리면 멈춤 [m/s]
DEADLOCK_S = 5.0  # 둘 다 멈춘 채 가까운 시간 [s]


def new_state(n: int, k: int, device="cpu") -> dict:
    """env n 개, 로봇 k 대의 지표 상태."""
    f = dict(device=device, dtype=torch.float32)
    return {
        "min_gap": torch.full((n,), float("inf"), **f),
        "enc_s": torch.zeros(n, **f),  # 첫 만남 시간
        "enc_on": torch.zeros(n, dtype=torch.bool, device=device),  # 첫 만남 중
        "enc_seen": torch.zeros(n, dtype=torch.bool, device=device),  # 첫 만남이 시작됨
        "still_s": torch.zeros(n, k, **f),  # 로봇별 둘 다 멈춰 가까운 연속 시간
        "deadlock": torch.zeros(n, dtype=torch.bool, device=device),
    }


def reset(st: dict, ids) -> None:
    st["min_gap"][ids] = float("inf")
    for key in ("enc_s", "still_s"):
        st[key][ids] = 0.0
    for key in ("enc_on", "enc_seen", "deadlock"):
        st[key][ids] = False


def update(st: dict, gaps: torch.Tensor, v_ours: torch.Tensor, v_robots: torch.Tensor,
           dt: float) -> None:
    """한 스텝: gaps (n, k) 로봇별 몸체 간격 (비활성 inf), v_ours (n,) 우리 속력, v_robots (n, k) 로봇 속력."""
    if gaps.shape[1] == 0:
        return
    g = gaps.amin(-1)
    st["min_gap"] = torch.minimum(st["min_gap"], g)
    near = g < ENCOUNTER_M
    start = near & ~st["enc_seen"]
    st["enc_seen"] = st["enc_seen"] | start
    st["enc_on"] = (st["enc_on"] | start) & near  # 넘으면 닫히고 다시 안 열림
    st["enc_s"] = st["enc_s"] + dt * st["enc_on"].float()
    stuck = (gaps < ENCOUNTER_M) & (v_robots < STILL_MPS) & (v_ours < STILL_MPS)[:, None]
    st["still_s"] = torch.where(stuck, st["still_s"] + dt, torch.zeros_like(st["still_s"]))
    # float32 누적 오차로 딱 DEADLOCK_S 가 넘었다고 보지 않게 1e-4 s 여유
    st["deadlock"] = st["deadlock"] | (st["still_s"] > DEADLOCK_S + 1e-4).any(-1)


def snapshot(st: dict, ids) -> dict:
    """끝난 에피소드 값 (리셋 전에 부름)."""
    return {k: st[k][ids].clone() for k in ("min_gap", "enc_s", "enc_seen", "deadlock")}


def summary(min_gap, enc_s, enc_seen, deadlock) -> dict:
    """에피소드별 배열로 eval.json 요약. 로봇이 없던 에피소드 (min_gap inf) 는 간격 분포에서 뺌."""
    min_gap = np.asarray(min_gap, dtype=float)
    enc_s = np.asarray(enc_s, dtype=float)
    enc_seen = np.asarray(enc_seen, dtype=bool)
    deadlock = np.asarray(deadlock, dtype=bool)
    fin = min_gap[np.isfinite(min_gap)]

    def q(x, p):
        return round(float(np.percentile(x, p)), 4) if len(x) else None

    n = max(len(min_gap), 1)
    return {
        "thresholds": {"encounter_m": ENCOUNTER_M, "still_mps": STILL_MPS,
                       "deadlock_s": DEADLOCK_S},
        "min_robot_gap_m": {
            "episodes_with_robot": int(len(fin)),
            "mean": round(float(fin.mean()), 4) if len(fin) else None,
            "p10": q(fin, 10), "p50": q(fin, 50), "p90": q(fin, 90),
            "min": round(float(fin.min()), 4) if len(fin) else None,
            "share_touching": round(float((fin <= 0.0).mean()), 4) if len(fin) else None,
            "share_under_encounter": round(float((fin < ENCOUNTER_M).mean()), 4)
            if len(fin) else None,
        },
        "encounter_s": {
            "episodes": int(enc_seen.sum()),
            "mean": round(float(enc_s[enc_seen].mean()), 3) if enc_seen.any() else None,
            "p50": q(enc_s[enc_seen], 50), "p90": q(enc_s[enc_seen], 90),
        },
        "deadlocks": int(deadlock.sum()),
        "deadlock_rate": round(float(deadlock.sum()) / n, 4),
    }
