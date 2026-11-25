"""자기 대국(ghost pairing, 시뮬레이터 없이): 짝 env 의 학습 로봇을 플릿 로봇 칸 0 으로 보여 주는 규칙.

env 2k 와 2k+1 이 짝이다. env i 의 로봇 칸 0 (ghost) 은 제어 스텝마다 짝 env 의 학습 로봇이 그 스텝 시작
(물리 전)에 있던 자세로 옮긴다: 짝 로봇보다 한 제어 스텝 늦다 (최고속도 0.12 m/s 에서 6 mm).
두 env 는 늘 같이 리셋한다 (pair_dones): 한쪽이 끝나면 다른 쪽은 시간초과로 끊고, 둘이 서로 부딪힌 충돌이면
둘 다 충돌로 끝낸다. 그래서 ghost 가 에피소드 중에 순간이동하지 않고 리셋 때 짝 경로를 함께 뽑는다.
짝 에피소드는 장애물 배치 0 (박스 없음) 을 쓴다: 박스는 경로마다 따로 뽑혀 두 env 가 같은 박스를 못 가짐.
"""

from __future__ import annotations

import numpy as np
import torch

KINDS = ("random", "headon", "scenario")
# 충돌한 env 의 몸체와 ghost 상자 여유가 이 이하일 때만 서로 부딪힌 충돌로 봄 [m]
HIT_M = 0.01


def partner(ids: torch.Tensor) -> torch.Tensor:
    """env 번호의 짝 (2k <-> 2k+1)."""
    return ids ^ 1


def pair_tables(routes, start_sep_m: float, scen_pairs: np.ndarray | None = None,
                device="cpu") -> dict:
    """짝 뽑기 표 (P = 시작/목표 쌍 수, 경로 인덱스는 배치 0 = 쌍 * layouts).
    ok (P, P): 무작위 짝으로 쓸 수 있음 (시작끼리, 목표끼리 start_sep_m 이상).
    reverse (P,): 시작과 목표를 바꾼 쌍 (정면), 없으면 -1.
    scen (C, 2): 시나리오 짝의 쌍 인덱스 (scenario_spec.build_pair_scenario 를 쌍으로 바꿈)."""
    pairs = np.array(routes.pairs)
    xy = routes.node_xy
    s, g = xy[pairs[:, 0]], xy[pairs[:, 1]]
    ds = np.linalg.norm(s[:, None] - s[None], axis=-1)
    dg = np.linalg.norm(g[:, None] - g[None], axis=-1)
    ok = (ds >= start_sep_m) & (dg >= start_sep_m)
    index = {tuple(p): i for i, p in enumerate(pairs.tolist())}
    rev = np.array([index.get((b, a), -1) for a, b in pairs.tolist()])
    scen = np.zeros((0, 2), dtype=np.int64) if scen_pairs is None else scen_pairs // routes.layouts
    return {
        "ok": torch.as_tensor(ok, device=device),
        "reverse": torch.as_tensor(rev, device=device),
        "scen": torch.as_tensor(scen, device=device),
        "layouts": int(routes.layouts),
    }


def draw_pairs(tables: dict, m: int, headon_p: float, scen_p: float, gen=None):
    """짝 m 개의 (쌍 a (m,), 쌍 b (m,), 종류 (m,) KINDS 인덱스).
    종류는 headon_p, scen_p, 나머지 무작위. 정면은 b = a 를 뒤집은 쌍, 시나리오는 후보 하나를 고르고 순서를 섞음.
    무작위 a 는 무작위 짝이 하나라도 있는 쌍에서 고름."""
    ok = tables["ok"]
    dev = ok.device
    p = ok.shape[0]
    u = torch.rand(m, device=dev, generator=gen)
    kind = torch.zeros(m, dtype=torch.long, device=dev)
    kind = torch.where(u < headon_p + scen_p, 2, kind)
    kind = torch.where(u < headon_p, 1, kind)
    has = ok.any(-1)
    a = torch.multinomial(has.float().expand(m, p), 1, generator=gen)[:, 0]
    b = torch.multinomial(ok[a].float(), 1, generator=gen)[:, 0]
    rev = tables["reverse"][a]
    head = (kind == 1) & (rev >= 0)
    b = torch.where(head, rev, b)
    kind = torch.where((kind == 1) & ~head, 0, kind)
    sc = tables["scen"]
    if len(sc):
        row = torch.randint(0, len(sc), (m,), device=dev, generator=gen)
        swap = torch.rand(m, device=dev, generator=gen) < 0.5
        sa = torch.where(swap, sc[row, 1], sc[row, 0])
        sb = torch.where(swap, sc[row, 0], sc[row, 1])
        is_sc = kind == 2
        a, b = torch.where(is_sc, sa, a), torch.where(is_sc, sb, b)
    else:
        kind = torch.where(kind == 2, 0, kind)
    return a, b, kind


def pair_priority(m: int, prob: float, gen=None, device="cpu"):
    """짝 m 개의 우선권 (a 가 보는 ghost 가 우선 (m,), b 가 보는 ghost 가 우선 (m,)).
    prob 확률로 짝에 순서가 있고 둘 중 하나가 고르게 우선, 아니면 둘 다 아님. 둘이 동시에 우선인 일은 없음."""
    if prob <= 0:
        z = torch.zeros(m, dtype=torch.bool, device=device)
        return z, z.clone()
    ordered = torch.rand(m, device=device, generator=gen) < prob
    b_first = torch.rand(m, device=device, generator=gen) < 0.5
    return ordered & b_first, ordered & ~b_first


def pair_dones(terminated: torch.Tensor, time_out: torch.Tensor, mutual: torch.Tensor):
    """짝 env 를 같이 끝냄. mutual (n,) 은 이 env 가 ghost 와 부딪혀 끝남.
    반환 (terminated, time_out, 짝 때문에 충돌이 된 env (n,)): 짝이 ghost 와 부딪혔으면 이 env 도 충돌로 끝나고
    (같은 스텝에 성공했어도: env 가 success 를 끔), 짝이 다른 까닭으로 끝나면 이 env 는 (아직 안 끝났다면)
    시간초과로 끊는다."""
    idx = partner(torch.arange(len(terminated), device=terminated.device))
    hit_by = mutual[idx] & ~mutual
    term = terminated | hit_by
    done_p = terminated[idx] | time_out[idx]
    out_to = (time_out | done_p) & ~term
    return term, out_to, hit_by


def path_progress(path: torch.Tensor, cum: torch.Tensor, npts: torch.Tensor, prog: torch.Tensor,
                  xy: torch.Tensor) -> torch.Tensor:
    """(n,) 지나온 호장: 진행 인덱스 prog 의 점 + 다음 선분에 xy 를 투영한 길이 (route_spec.follow 와 같은 투영)."""
    rows = torch.arange(len(prog), device=path.device)
    p0 = path[rows, prog]
    nxt = torch.minimum(prog + 1, npts - 1)
    seg = path[rows, nxt] - p0
    seg_len = seg.norm(dim=-1)
    t = ((xy - p0) * seg).sum(-1) / seg_len.clamp(min=1e-9) ** 2
    t = torch.where(seg_len > 0, t.clamp(0.0, 1.0), torch.zeros_like(t))
    return cum[rows, prog] + t * seg_len


def path_ahead(path: torch.Tensor, cum: torch.Tensor, npts: torch.Tensor, done_arc: torch.Tensor,
               spacing_m: float, points: int) -> torch.Tensor:
    """경로 (n, L, 2) 에서 지나온 호장 done_arc (n,) 뒤 1..points * spacing_m 지점 (n, points, 2), 목표에서 멈춤.
    ghost 의 intent 는 짝 로봇의 자기 전역 경로 남은 부분이다 (스크립트 로봇의 intent_points 와 같은 간격)."""
    n, lmax = cum.shape
    rows = torch.arange(n, device=path.device)
    total = cum[rows, npts - 1]
    s = done_arc[:, None] + spacing_m * torch.arange(1, points + 1, device=path.device)[None]
    s = torch.minimum(s, total[:, None])
    i = torch.searchsorted(cum.contiguous(), s.contiguous(), right=True).clamp(1, lmax - 1)
    i = torch.minimum(i, (npts - 1).clamp(min=1)[:, None])
    s0, s1 = cum.gather(1, i - 1), cum.gather(1, i)
    f = ((s - s0) / (s1 - s0).clamp(min=1e-9)).clamp(0.0, 1.0)
    p0 = path[rows[:, None], i - 1]
    p1 = path[rows[:, None], i]
    return p0 + f[..., None] * (p1 - p0)
