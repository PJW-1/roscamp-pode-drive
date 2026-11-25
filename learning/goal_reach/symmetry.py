"""좌우 거울 대칭: 관측, 크리틱 특권 항목, 행동의 거울 변환 (Isaac 없이, 순수 torch).

거울 = 로봇 정면 축(base x) 기준 좌우 뒤집기: y -> -y, yaw 각 -> -각, w -> -w.
라이다 섹터 k (레이 10k..10k+9, -180+10k..-171+10k deg) 의 거울은 섹터 35-k 로 둔다.
정확한 거울은 레이 하나(1 deg) 어긋난 창이라 섹터로는 만들 수 없다 (크기는 test_symmetry.py).
모든 변환은 자기 역변환이다 (두 번 하면 원래 값).
"""

from __future__ import annotations

import functools
import re

import task_spec as ts
import torch

# obs_blocks 가 내는 선택 블록 중 이 모듈이 거울을 아는 것: 새 블록은 mirror_table 에서 멈춤
KNOWN_BLOCKS = ("prev_action", "lidar_history", "speed_cap", "intent")

# 관측 항목 이름 -> (거울 짝 이름, 부호). 섹터는 (본뜬 접두어, 섹터 번호)로 따로 처리
_FIXED = {
    "goal_dist": ("goal_dist", 1.0),
    "goal_sin": ("goal_sin", -1.0),
    "goal_cos": ("goal_cos", 1.0),
    "v": ("v", 1.0),
    "w": ("w", -1.0),
    "prev_v": ("prev_v", 1.0),
    "prev_w": ("prev_w", -1.0),
    "speed_cap": ("speed_cap", 1.0),
}
_SECTOR = re.compile(r"^(lidar_sector_|lidar_hist\d+_sector_)(\d+)$")
_ROBOT = re.compile(r"^robot\d+_(present|priority|p\d+_x|p\d+_y)$")

ACTION_SIGN = (1.0, -1.0)  # (v, w): 같은 속도, 반대 회전
PRIV_SIGN = (1.0, 1.0, -1.0, 1.0, -1.0)  # 물체마다 [존재, x, y, vx, vy]


def mirror_sector(k: int, sectors: int) -> int:
    """섹터 k 의 거울 섹터 (근사: 정확한 거울 창과 레이 하나 어긋남)."""
    return sectors - 1 - k


def _partner(name: str, sectors: int) -> tuple[str, float]:
    if name in _FIXED:
        return _FIXED[name]
    m = _SECTOR.match(name)
    if m:
        return f"{m.group(1)}{mirror_sector(int(m.group(2)), sectors)}", 1.0
    m = _ROBOT.match(name)
    if m:
        return name, -1.0 if m.group(1).endswith("_y") else 1.0
    raise KeyError(f"no mirror rule for observation entry {name!r}")


@functools.lru_cache(maxsize=None)
def mirror_table(spec: ts.TaskSpec = ts.SPEC) -> tuple[tuple[int, ...], tuple[float, ...]]:
    """(perm, sign): 거울 관측 [..., i] = obs[..., perm[i]] * sign[i].
    모르는 블록이나 항목이면 멈춘다."""
    unknown = set(ts.obs_blocks(spec)) - set(KNOWN_BLOCKS)
    if unknown:
        raise KeyError(f"no mirror rule for observation blocks {sorted(unknown)}")
    names = ts.obs_names(spec)
    if len(names) != ts.obs_dim(spec):
        raise ValueError(f"obs_names has {len(names)} entries, obs_dim is {ts.obs_dim(spec)}")
    index = {n: i for i, n in enumerate(names)}
    perm, sign = [], []
    for n in names:
        partner, s = _partner(n, spec.lidar_sectors)
        perm.append(index[partner])
        sign.append(s)
    return tuple(perm), tuple(sign)


def mirror_obs(obs: torch.Tensor, spec: ts.TaskSpec = ts.SPEC) -> torch.Tensor:
    """액터 관측 (..., obs_dim) 의 거울."""
    perm, sign = mirror_table(spec)
    if obs.shape[-1] != len(perm):
        raise ValueError(f"observation has {obs.shape[-1]} entries, spec expects {len(perm)}")
    p = torch.tensor(perm, device=obs.device)
    s = torch.tensor(sign, device=obs.device, dtype=obs.dtype)
    return obs[..., p] * s


def mirror_privileged(priv: torch.Tensor) -> torch.Tensor:
    """크리틱 특권 항목 (..., 5 x 물체 수) 의 거울: 물체마다 y, vy 부호 반전."""
    k = ts.PRIV_PER_OBJECT
    if priv.shape[-1] % k:
        raise ValueError(f"privileged block of {priv.shape[-1]} entries is not {k} per object")
    s = torch.tensor(PRIV_SIGN, device=priv.device, dtype=priv.dtype)
    return (priv.unflatten(-1, (-1, k)) * s).flatten(-2)


def mirror_actions(actions: torch.Tensor) -> torch.Tensor:
    """행동 (..., 2) = (v, w) 의 거울."""
    return actions * torch.tensor(ACTION_SIGN, device=actions.device, dtype=actions.dtype)


def mirror_groups(obs, spec: ts.TaskSpec):
    """관측 TensorDict ("policy", 선택 "critic") 의 거울 사본. 모르는 그룹이면 멈춤."""
    out = obs.clone()
    for key in obs.keys():
        if key == "policy":
            out[key] = mirror_obs(obs[key], spec)
        elif key == "critic":
            out[key] = mirror_privileged(obs[key])
        else:
            raise KeyError(f"no mirror rule for observation group {key!r}")
    return out


def augment(env=None, obs=None, actions=None):
    """rsl_rl Symmetry 의 data_augmentation_func: 원본 뒤에 거울 사본을 붙인다 (배치 2 배).

    obs 는 TensorDict, actions 는 (B, 2). 어느 쪽이든 None 이면 None 을 돌려준다.
    TaskSpec 은 env.unwrapped.spec (매장 아레나 env) 에서 읽는다."""
    out_obs = out_act = None
    if obs is not None:
        spec = env.unwrapped.spec
        if not isinstance(spec, ts.TaskSpec):  # gym.Env.spec (EnvSpec) 가 아닌 과제 정의여야 함
            raise TypeError(f"env.unwrapped.spec is {type(spec).__name__}, expected TaskSpec")
        out_obs = torch.cat((obs, mirror_groups(obs, spec)), dim=0)
    if actions is not None:
        out_act = torch.cat((actions, mirror_actions(actions)), dim=0)
    return out_obs, out_act
