"""움직이는 것 근처 보상 항(시뮬레이터 없이): 불편 거리, 양보, 서로 막힘. 모두 가중치 0 이면 꺼짐.

설정은 ArenaSpec 의 near_dyn_m / w_near_dyn, yield_m / w_yield, block_m / w_block / block_v_mps 한 곳이다.
간격은 우리 몸체 박스와 mover/로봇 표면 사이 거리 (footprint_clearance 와 같음, 겹치면 0, 비활성 inf).
모두 제어 스텝마다 더하는 항이고 성공/충돌 같은 끝 보상은 건드리지 않는다.
"""

from __future__ import annotations

import torch

# 한 스텝 이동이 이보다 작으면 거리가 줄지 않은 것으로 봄 [m] (0.02 m/s x 0.05 s)
STEP_EPS_M = 1e-3


def near_dyn_penalty(gap: torch.Tensor, near_m: float, weight: float) -> torch.Tensor:
    """(n,) 불편 거리 벌점: gap (n,) 이 near_m 미만이면 weight * (near_m - gap) / near_m.
    CrowdNav/SARL 의 discomfort distance 를 몸체 간격에 적용. near_m <= 0 또는 weight 0 이면 0."""
    if near_m <= 0 or weight == 0:
        return torch.zeros_like(gap)
    return weight * ((near_m - gap) / near_m).clamp(min=0.0)


def yield_reward(
    gap: torch.Tensor,
    priority: torch.Tensor,
    rob_prev: torch.Tensor,
    rob_now: torch.Tensor,
    our_prev: torch.Tensor,
    our_now: torch.Tensor,
    yield_m: float,
    weight: float,
) -> torch.Tensor:
    """(n,) 양보 보상. gap (n, k) 로봇별 몸체 간격, priority (n, k) 로봇이 우리보다 우선,
    rob_prev/rob_now (n, k, 2) 와 our_prev/our_now (n, 2) 는 이번 스텝 전후 중심 위치.

    로봇 k 가 '다가옴': 로봇 자신의 이번 스텝 이동이 우리 직전 위치와의 중심 거리를 STEP_EPS_M 넘게 줄임
        |rob_now - our_prev| < |rob_prev - our_prev| - STEP_EPS_M.
    우리가 '간격을 지킴': 우리 이번 스텝 이동이 로봇 현재 위치와의 중심 거리를 STEP_EPS_M 넘게 줄이지 않음
        |rob_now - our_now| >= |rob_now - our_prev| - STEP_EPS_M (서 있거나 물러남).
    우선 로봇이 yield_m 이내 (gap < yield_m) 에서 다가오는 경우가 하나라도 있고, 그 모두에 대해 간격을
    지키면 weight, 아니면 0. yield_m <= 0 또는 weight 0 이면 0."""
    out = torch.zeros(gap.shape[0], device=gap.device)
    if yield_m <= 0 or weight == 0 or gap.shape[1] == 0:
        return out
    d_before = (rob_prev - our_prev[:, None]).norm(dim=-1)
    d_robot_moved = (rob_now - our_prev[:, None]).norm(dim=-1)
    d_now = (rob_now - our_now[:, None]).norm(dim=-1)
    approaching = d_robot_moved < d_before - STEP_EPS_M
    keeps = d_now >= d_robot_moved - STEP_EPS_M
    relevant = priority & (gap < yield_m) & approaching
    ok = relevant.any(-1) & (keeps | ~relevant).all(-1)
    return torch.where(ok, out + weight, out)


def block_penalty(
    gap: torch.Tensor,
    v_ours: torch.Tensor,
    v_robots: torch.Tensor,
    block_m: float,
    v_still: float,
    weight: float,
) -> torch.Tensor:
    """(n,) 서로 막힘 벌점: 어떤 로봇 k 와 gap (n, k) < block_m 이고 우리 속력 v_ours (n,) 와 그 로봇 속력
    v_robots (n, k) 가 모두 v_still 미만이면 weight. block_m <= 0 또는 weight 0 이면 0."""
    out = torch.zeros(gap.shape[0], device=gap.device)
    if block_m <= 0 or weight == 0 or gap.shape[1] == 0:
        return out
    stuck = (gap < block_m) & (v_robots < v_still) & (v_ours < v_still)[:, None]
    return torch.where(stuck.any(-1), out + weight, out)
