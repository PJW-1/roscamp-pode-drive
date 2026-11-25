"""caps_ppo.py 시험: CAPS 손실 값, 에피소드 끝 쌍 제외, 기울기가 가는 곳, 계수 0 이면 PPO 와 같음.
TensorDict, rsl_rl 이 필요해 Isaac 가상환경 인터프리터로만 돈다 (없으면 건너뜀)."""

import os
import sys
from types import SimpleNamespace

import pytest
import torch

td = pytest.importorskip("tensordict")
pytest.importorskip("rsl_rl")
from rsl_rl.algorithms import PPO  # noqa: E402
from rsl_rl.models import MLPModel  # noqa: E402
from rsl_rl.storage import RolloutStorage  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import caps_ppo as cp  # noqa: E402

T, N, D_POL, D_CRIT, A = 6, 4, 5, 3, 2
OBS_GROUPS = {"actor": ["policy"], "critic": ["policy", "critic"]}
PPO_KW = dict(num_learning_epochs=2, num_mini_batches=2, learning_rate=1e-3, device="cpu")


def obs_like(gen, dims=(D_POL, D_CRIT), n=N):
    return td.TensorDict(
        {"policy": torch.randn(n, dims[0], generator=gen), "critic": torch.randn(n, dims[1], generator=gen)},
        batch_size=[n],
    )


def make_alg(cls=PPO, seed=0, dims=(D_POL, D_CRIT), **kw):
    """같은 seed 면 같은 가중치와 같은 롤아웃 (비대칭 크리틱: critic 은 policy+critic 그룹)."""
    torch.manual_seed(seed)
    gen = torch.Generator().manual_seed(seed)
    obs = obs_like(gen, dims)
    actor = MLPModel(
        obs, OBS_GROUPS, "actor", A, hidden_dims=[16, 16], obs_normalization=True,
        distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.5},
    )
    critic = MLPModel(obs, OBS_GROUPS, "critic", 1, hidden_dims=[16, 16], obs_normalization=True)
    storage = RolloutStorage("rl", N, T, obs, [A])
    alg = cls(actor, critic, storage, **PPO_KW, **kw)
    return alg, obs, gen


def fill(alg, obs, gen, dims=(D_POL, D_CRIT)):
    for t in range(T):
        alg.act(obs)
        obs = obs_like(gen, dims)
        dones = torch.zeros(N, dtype=torch.long)
        if t == 2:
            dones[1] = 1
        alg.process_env_step(obs, torch.randn(N, generator=gen), dones, {})
    alg.compute_returns(obs)


def params(alg):
    return {k: v.clone() for k, v in alg.save()["actor_state_dict"].items()} | {
        "c." + k: v.clone() for k, v in alg.save()["critic_state_dict"].items()
    }


def run_update(cls, seed=0, **kw):
    alg, obs, gen = make_alg(cls, seed, **kw)
    fill(alg, obs, gen)
    torch.manual_seed(seed + 100)
    losses = alg.update()
    return alg, losses


def test_zero_coefficients_equal_stock_ppo():
    stock, l0 = run_update(PPO)
    caps, l1 = run_update(cp.CapsPPO, caps_temporal=0.0, caps_spatial=0.0, caps_sigma=0.0)
    p0, p1 = params(stock), params(caps)
    assert p0.keys() == p1.keys()
    for k in p0:
        assert torch.equal(p0[k], p1[k]), k
    assert l0 == l1


def test_caps_update_changes_the_actor_and_restores_zero_grad():
    stock, _ = run_update(PPO)
    caps, losses = run_update(cp.CapsPPO, caps_temporal=1.0, caps_spatial=1.0, caps_sigma=0.1)
    assert losses["caps_temporal"] > 0 and losses["caps_spatial"] > 0
    assert "zero_grad" not in caps.optimizer.__dict__
    p0, p1 = params(stock), params(caps)
    assert not torch.equal(p0["mlp.0.weight"], p1["mlp.0.weight"])


def identity_actor(d=2):
    """정규화 없음, MLP [d] 를 항등으로 (ELU 의 양수 구간): 행동 평균 = 관측."""
    obs = td.TensorDict({"policy": torch.zeros(1, d)}, batch_size=[1])
    actor = MLPModel(
        obs, {"actor": ["policy"]}, "actor", d, hidden_dims=[d],
        distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.5},
    )
    with torch.no_grad():
        actor.mlp[0].weight.copy_(torch.eye(d))
        actor.mlp[0].bias.fill_(10.0)
        actor.mlp[2].weight.copy_(torch.eye(d))
        actor.mlp[2].bias.fill_(-10.0)
    return actor


def flat_obs(x):
    return td.TensorDict({"policy": x}, batch_size=[x.shape[0]])


def test_terms_equal_hand_values():
    actor = identity_actor()
    # 평탄 순서 t*N+n, N=2: t0=[(0,0),(1,1)], t1=[(1,2),(1,-1)]
    x = torch.tensor([[0.0, 0.0], [1.0, 1.0], [1.0, 2.0], [1.0, -1.0]])
    loss = cp.temporal_loss(actor, flat_obs(x), torch.tensor([0, 1]), num_envs=2)
    # 쌍 0: (0,0)-(1,2) -> 1+4, 쌍 1: (1,1)-(1,-1) -> 0+4; 4 원소 평균 = 9/4
    assert torch.isclose(loss, torch.tensor(9.0 / 4.0))
    torch.manual_seed(5)
    s = cp.spatial_loss(actor, flat_obs(x), sigma=0.3)
    torch.manual_seed(5)
    eps = torch.randn(4, 2)
    assert torch.isclose(s, (0.3 * eps).pow(2).mean())


def test_pairs_across_an_episode_end_are_excluded():
    dones = torch.zeros(3, 2, 1, dtype=torch.uint8)
    dones[0, 1] = 1  # env 1 은 t=0 에서 끝남: (t0,env1)-(t1,env1) 은 다른 에피소드
    idx = cp.temporal_pair_index(dones)
    assert sorted(idx.tolist()) == [0, 2, 3]  # t<2: (0,0)=0, (1,0)=2, (1,1)=3
    actor = identity_actor()
    x = torch.zeros(6, 2)
    x[3] = 100.0  # 끝난 뒤 새 에피소드의 첫 관측
    loss = cp.temporal_loss(actor, flat_obs(x), idx, num_envs=2)
    # 남은 쌍 중 (1,1)=3 -> (2,1)=5 만 0 이 아님 (100 -> 0), 끝난 쌍 1 -> 3 은 없음
    assert torch.isclose(loss, torch.tensor(100.0**2 * 2 / 6))


def test_terms_reach_only_the_actor_mean():
    alg, obs, gen = make_alg(cp.CapsPPO, caps_temporal=1.0, caps_spatial=1.0, caps_sigma=0.1)
    fill(alg, obs, gen)
    st = alg.storage
    alg.optimizer.zero_grad(set_to_none=True)
    loss, lt, ls = alg.caps_loss(st.observations.flatten(0, 1), cp.temporal_pair_index(st.dones), N, 8)
    assert lt > 0 and ls > 0
    loss.backward()
    dist = alg.actor.distribution
    assert dist.std_param.grad is None or not dist.std_param.grad.any()
    assert all(p.grad is None or not p.grad.any() for p in alg.critic.parameters())
    assert alg.actor.mlp[0].weight.grad.abs().sum() > 0


def test_bad_settings_stop():
    with pytest.raises(ValueError):
        make_alg(cp.CapsPPO, caps_spatial=1.0)
    with pytest.raises(ValueError):
        make_alg(cp.CapsPPO, caps_temporal=-1.0)


def test_runs_with_symmetry_and_a_reset_std():
    """symmetry 확장(거울 사본 + 거울 손실)과 --init_std 처럼 키운 std 와 함께 돈다."""
    sym = pytest.importorskip("symmetry")
    ts = pytest.importorskip("task_spec")
    spec = ts.make_spec(dict(prev_action_obs=1, speed_cap_obs=1, critic_privileged=1))
    env = SimpleNamespace(unwrapped=SimpleNamespace(spec=spec))
    dims = (ts.obs_dim(spec), ts.PRIV_PER_OBJECT * 2)
    sym_cfg = dict(
        env=env, data_augmentation_func=sym.augment, use_data_augmentation=True,
        use_mirror_loss=True, mirror_loss_coeff=1.0,
    )
    alg, obs, gen = make_alg(
        cp.CapsPPO, dims=dims, symmetry_cfg=sym_cfg, caps_temporal=0.1, caps_spatial=0.1, caps_sigma=0.1
    )
    with torch.no_grad():
        alg.actor.distribution.std_param[:] = 0.3
    fill(alg, obs, gen, dims)
    losses = alg.update()
    assert {"symmetry", "caps_temporal", "caps_spatial"} <= set(losses)
    assert all(torch.isfinite(torch.tensor(v)) for v in losses.values())
