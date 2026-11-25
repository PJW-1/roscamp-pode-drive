"""학습한 정책이 좌우 거울 대칭에서 얼마나 벗어나는지 잰다 (Isaac 불필요, CPU torch).

play.py --dump_obs N 이 쓴 obs_dump.npz 의 실제 관측 o 에서 액터 MLP 를 돌려
|a_v(o) - a_v(M o)| 와 |a_w(o) + a_w(M o)| (행동 단위, [-1, 1] 로 자른 값) 를 보고하고,
라이다 최소 섹터가 --near_m 미만인 관측(장애물 가까이)과 나머지로도 나눈다.
회전 치우침 = (a_w(o) + a_w(M o)) / 2 의 평균 (+ 는 왼쪽으로 돎).
액터는 rsl_rl 과 같이 (o - mean) / (std + 0.01) 뒤 ELU MLP.
덤프의 결정적 행동과 --tol 넘게 다르면 멈춘다.

실행: python learning/store_arena/symmetry_check.py --checkpoint model_X.pt \\
    --dump <out>/obs_dump.npz
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))
import symmetry as sym  # noqa: E402
import task_spec as ts  # noqa: E402

NORMALIZER_EPS = 1e-2  # rsl_rl EmpiricalNormalization 기본값
PERCENTILES = (50, 90, 95, 99)


def load_actor(checkpoint: str):
    """체크포인트의 액터를 순수 torch 함수로: obs (N, D) -> 행동 평균 (N, 2), 클램프 없음."""
    sd = torch.load(checkpoint, map_location="cpu", weights_only=False)["actor_state_dict"]
    mean = sd.get("obs_normalizer._mean")
    std = sd.get("obs_normalizer._std")
    ids = sorted(int(k.split(".")[1]) for k in sd if k.startswith("mlp.") and k.endswith(".weight"))
    layers = [(sd[f"mlp.{i}.weight"].float(), sd[f"mlp.{i}.bias"].float()) for i in ids]

    def run(obs: torch.Tensor) -> torch.Tensor:
        x = obs.float()
        if mean is not None:
            x = (x - mean) / (std + NORMALIZER_EPS)
        for j, (w, b) in enumerate(layers):
            x = x @ w.T + b
            if j < len(layers) - 1:
                x = torch.nn.functional.elu(x)
        return x

    return run


def stats(x: torch.Tensor) -> dict:
    if x.numel() == 0:
        return {"n": 0}
    out = {"n": int(x.numel()), "mean": round(float(x.mean()), 5)}
    for p in PERCENTILES:
        out[f"p{p}"] = round(float(torch.quantile(x, p / 100.0)), 5)
    out["max"] = round(float(x.max()), 5)
    return out


def equivariance(run, obs: torch.Tensor, spec: ts.TaskSpec, near_m: float = 0.25) -> dict:
    """거울 등변 오차 (행동 단위). obs (N, D) 는 액터 관측."""
    a = run(obs).clamp(-1.0, 1.0)
    am = run(sym.mirror_obs(obs, spec)).clamp(-1.0, 1.0)
    ev = (a[:, 0] - am[:, 0]).abs()
    ew = (a[:, 1] + am[:, 1]).abs()
    bias = 0.5 * (a[:, 1] + am[:, 1])
    lidar_min_m = obs[:, : spec.lidar_sectors].amin(-1) * spec.lidar_obs_max
    near = lidar_min_m < near_m
    out = {}
    for label, sel in (("all", torch.ones_like(near)), ("near", near), ("clear", ~near)):
        out[label] = {
            "v_error": stats(ev[sel]),
            "w_error": stats(ew[sel]),
            "turn_bias_mean": round(float(bias[sel].mean()), 5) if sel.any() else None,
        }
    out["near_m"] = near_m
    out["near_share"] = round(float(near.float().mean()), 4)
    return out


def load_dump(path: str):
    """(관측 (N, D), 결정적 행동 (N, 2), TaskSpec). 스텝·env 축을 한 축으로 편다."""
    d = np.load(path, allow_pickle=False)
    obs = torch.as_tensor(d["obs"]).reshape(-1, d["obs"].shape[-1]).float()
    act = torch.as_tensor(d["actions"]).reshape(-1, d["actions"].shape[-1]).float()
    over = json.loads(str(d["spec_overrides"]))  # JSON 은 튜플 필드를 리스트로 씀
    spec = ts.make_spec({k: tuple(v) if isinstance(v, list) else v for k, v in over.items()})
    if obs.shape[-1] != ts.obs_dim(spec):
        raise SystemExit(
            f"dump has {obs.shape[-1]} entries per observation, spec {ts.obs_dim(spec)}"
        )
    return obs, act, spec


def check(checkpoint: str, dump: str, near_m: float = 0.25, tol: float = 1e-4) -> dict:
    obs, act, spec = load_dump(dump)
    run = load_actor(checkpoint)
    with torch.no_grad():
        diff = float((run(obs) - act).abs().max())
        if diff > tol:
            raise SystemExit(
                f"actor forward pass differs from the dumped actions by {diff:.3g} > {tol:g}: "
                "wrong checkpoint, dump or network layout"
            )
        out = equivariance(run, obs, spec, near_m)
    out["forward_max_abs_diff"] = diff
    out["checkpoint"] = checkpoint
    out["dump"] = dump
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--dump", required=True, help="obs_dump.npz from play.py --dump_obs N")
    p.add_argument("--near_m", type=float, default=0.25, help="near: min sector below this")
    p.add_argument("--tol", type=float, default=1e-4, help="max |forward - dumped action|")
    p.add_argument("--json", default="", help="also write the result here")
    args = p.parse_args(argv)
    out = check(args.checkpoint, args.dump, args.near_m, args.tol)
    text = json.dumps(out, indent=1)
    print(text)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            f.write(text + "\n")
    return out


if __name__ == "__main__":
    main()
