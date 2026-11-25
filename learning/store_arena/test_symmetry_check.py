"""symmetry_check.py 테스트: 손으로 만든 체크포인트 (Isaac 없이).
실행: python -m pytest learning -q"""

import json
import os
import sys

import numpy as np
import pytest
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import symmetry_check as sc  # noqa: E402

ts, sym = sc.ts, sc.sym
OVER = {"prev_action_obs": 1, "intent_robots": 1, "lidar_scale_range": (0.9, 1.1)}


def mirror_matrix(spec):
    perm, sign = sym.mirror_table(spec)
    m = torch.zeros(len(perm), len(perm))
    m[torch.arange(len(perm)), torch.tensor(perm)] = torch.tensor(sign)
    return m  # M x = mirror_obs(x)


def equivariant_state_dict(spec, w_bias=0.0, seed=0):
    """정확히 등변인 ELU MLP: 은닉 두 반쪽이 x 와 M x 를 보고, 거울은 두 반쪽을 맞바꾼다.
    출력 v = u.(g1 + g2), w = u.(g1 - g2) + w_bias. 정규화 mean 은 거울 불변, std 는 짝끼리 같음."""
    g = torch.Generator().manual_seed(seed)
    D, H = ts.obs_dim(spec), 64
    M = mirror_matrix(spec)
    a = 0.1 * torch.randn(H, D, generator=g)
    c = 0.1 * torch.randn(H, generator=g)
    b1, b2 = 0.1 * torch.randn(H, H, generator=g), 0.1 * torch.randn(H, H, generator=g)
    e = 0.1 * torch.randn(H, generator=g)
    u = 0.03 * torch.randn(H, generator=g)
    mean = torch.rand(D, generator=g)
    mean = 0.5 * (mean + M @ mean)
    std = 0.5 + torch.rand(D, generator=g)
    std = 0.5 * (std + std[list(sym.mirror_table(spec)[0])])
    return {
        "obs_normalizer._mean": mean[None],
        "obs_normalizer._std": std[None],
        "obs_normalizer.count": torch.tensor(1),
        "mlp.0.weight": torch.cat((a, a @ M)),
        "mlp.0.bias": torch.cat((c, c)),
        "mlp.2.weight": torch.cat((torch.cat((b1, b2), 1), torch.cat((b2, b1), 1))),
        "mlp.2.bias": torch.cat((e, e)),
        "mlp.4.weight": torch.stack((torch.cat((u, u)), torch.cat((u, -u)))),
        "mlp.4.bias": torch.tensor([0.05, w_bias]),
    }


def write_case(tmp_path, sd, spec_over, n_steps=6, n_env=50, seed=1, perturb=0.0):
    spec = ts.make_spec(spec_over)
    ckpt = tmp_path / "model.pt"
    torch.save({"actor_state_dict": sd}, ckpt)
    g = torch.Generator().manual_seed(seed)
    obs = torch.rand(n_steps, n_env, ts.obs_dim(spec), generator=g) * 2 - 1
    obs[..., : spec.lidar_sectors] = torch.rand(n_steps, n_env, spec.lidar_sectors, generator=g)
    act = sc.load_actor(str(ckpt))(obs.reshape(-1, obs.shape[-1])).reshape(n_steps, n_env, 2)
    dump = tmp_path / "obs_dump.npz"
    np.savez_compressed(
        dump, obs=obs.numpy(), actions=act.numpy() + perturb, spec_overrides=json.dumps(spec_over)
    )
    return str(ckpt), str(dump)


def test_equivariant_policy_scores_zero(tmp_path):
    spec = ts.make_spec(OVER)
    ckpt, dump = write_case(tmp_path, equivariant_state_dict(spec), OVER)
    out = sc.check(ckpt, dump)
    assert out["forward_max_abs_diff"] == 0.0
    assert out["all"]["v_error"]["max"] < 1e-5
    assert out["all"]["w_error"]["max"] < 1e-5
    assert abs(out["all"]["turn_bias_mean"]) < 1e-5
    assert 0 < out["near_share"] < 1
    assert out["near"]["v_error"]["n"] + out["clear"]["v_error"]["n"] == out["all"]["v_error"]["n"]


def test_known_turn_bias_is_measured(tmp_path):
    spec = ts.make_spec(OVER)
    sd = equivariant_state_dict(spec, w_bias=0.2)
    ckpt, dump = write_case(tmp_path, sd, OVER)
    run = sc.load_actor(ckpt)
    obs, _, _ = sc.load_dump(dump)
    assert run(obs).abs().max() < 0.8  # 클램프가 치우침을 깎지 않는 범위
    out = sc.check(ckpt, dump)
    assert out["all"]["turn_bias_mean"] == pytest.approx(0.2, abs=1e-5)
    assert out["all"]["w_error"]["mean"] == pytest.approx(0.4, abs=1e-5)
    assert out["all"]["w_error"]["max"] == pytest.approx(0.4, abs=1e-5)
    assert out["all"]["v_error"]["max"] < 1e-5


def test_forward_mismatch_stops(tmp_path):
    spec = ts.make_spec(OVER)
    ckpt, dump = write_case(tmp_path, equivariant_state_dict(spec), OVER, perturb=1e-3)
    with pytest.raises(SystemExit, match="differs from the dumped actions"):
        sc.check(ckpt, dump)


def test_forward_matches_rsl_rl_actor(tmp_path):
    """Isaac 가상환경에서만: rsl_rl MLPModel (정규화 켬, Gaussian) 의 결정적 출력과 같음."""
    td = pytest.importorskip("tensordict")
    models = pytest.importorskip("rsl_rl.models")
    spec = ts.make_spec(OVER)
    D = ts.obs_dim(spec)
    g = torch.Generator().manual_seed(3)
    obs = torch.randn(256, D, generator=g) * 0.7 + 0.2
    batch = td.TensorDict({"policy": obs}, batch_size=[256])
    actor = models.MLPModel(
        batch, {"actor": ["policy"]}, "actor", 2, hidden_dims=[128, 128], activation="elu",
        obs_normalization=True,
        distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.5},
    )
    actor.update_normalization(batch)  # 평균·표준편차가 0, 1 이 아니게
    with torch.no_grad():
        for p in actor.mlp.parameters():
            p.add_(0.1 * torch.randn(p.shape, generator=g))
        want = actor(batch)
    ckpt = tmp_path / "model.pt"
    torch.save({"actor_state_dict": actor.state_dict()}, ckpt)
    got = sc.load_actor(str(ckpt))(batch["policy"])
    assert (got - want).abs().max() < 1e-5


def test_dump_of_other_layout_stops(tmp_path):
    spec = ts.make_spec(OVER)
    ckpt, dump = write_case(tmp_path, equivariant_state_dict(spec), OVER)
    d = dict(np.load(dump))
    d["spec_overrides"] = json.dumps({})
    np.savez_compressed(dump, **d)
    with pytest.raises(SystemExit, match="entries per observation"):
        sc.check(ckpt, dump)
