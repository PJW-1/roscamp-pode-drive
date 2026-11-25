"""train.py --symmetry 시험: run_config 설정과 rsl_rl 의 data_augmentation_func (symmetry.augment).
TensorDict, rsl_rl 이 필요한 시험은 Isaac 가상환경 인터프리터로만 돈다 (없으면 건너뜀)."""

import os
import sys
from types import SimpleNamespace

import pytest
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))
import run_config as rc  # noqa: E402
import symmetry as sym  # noqa: E402
import task_spec as ts  # noqa: E402

SPEC = ts.make_spec(
    dict(prev_action_obs=1, lidar_frames=2, speed_cap_obs=1, intent_robots=2, intent_priority=1,
         critic_privileged=1)
)
N_PRIV = 5  # 물체 수 (mover 3 + 로봇 2 로 가정)


def fake_env(spec=SPEC):
    return SimpleNamespace(unwrapped=SimpleNamespace(spec=spec))


def test_modes_map_to_symmetry_cfg_arguments():
    assert rc.symmetry_settings("off") is None
    assert rc.symmetry_settings("aug") == {
        "use_data_augmentation": True, "use_mirror_loss": False, "mirror_loss_coeff": 0.0
    }
    assert rc.symmetry_settings("loss", 0.5) == {
        "use_data_augmentation": False, "use_mirror_loss": True, "mirror_loss_coeff": 0.5
    }
    assert rc.symmetry_settings("both")["mirror_loss_coeff"] == rc.MIRROR_LOSS_COEFF
    with pytest.raises(ValueError):
        rc.symmetry_settings("left")
    with pytest.raises(ValueError):
        rc.symmetry_settings("loss", 0.0)


def test_actions_only_and_none():
    a = torch.randn(7, 2)
    obs, out = sym.augment(env=None, obs=None, actions=a)
    assert obs is None and out.shape == (14, 2)
    assert torch.equal(out[:7], a) and torch.equal(out[7:], a * torch.tensor([1.0, -1.0]))
    assert sym.augment(env=None, obs=None, actions=None) == (None, None)


def make_obs(B, critic=True):
    td = pytest.importorskip("tensordict")
    g = torch.Generator().manual_seed(0)
    groups = {"policy": torch.randn(B, ts.obs_dim(SPEC), generator=g)}
    if critic:
        groups["critic"] = torch.randn(B, ts.PRIV_PER_OBJECT * N_PRIV, generator=g)
    return td.TensorDict(groups, batch_size=[B])


@pytest.mark.parametrize("critic", [True, False])
def test_augment_tensordict_order_shapes_values(critic):
    B = 11
    obs = make_obs(B, critic)
    act = torch.randn(B, 2)
    out_obs, out_act = sym.augment(env=fake_env(), obs=obs, actions=act)
    assert out_obs.batch_size[0] == 2 * B and out_act.shape == (2 * B, 2)
    assert set(out_obs.keys()) == set(obs.keys())
    # 원본이 먼저, 거울 사본이 뒤
    assert torch.equal(out_obs["policy"][:B], obs["policy"])
    assert torch.equal(out_obs["policy"][B:], sym.mirror_obs(obs["policy"], SPEC))
    if critic:
        assert torch.equal(out_obs["critic"][:B], obs["critic"])
        assert torch.equal(out_obs["critic"][B:], sym.mirror_privileged(obs["critic"]))
    assert torch.equal(out_act[B:], sym.mirror_actions(act))
    # 입력은 바뀌지 않음
    assert torch.equal(obs["policy"], out_obs["policy"][:B])
    # 거울 사본을 다시 거울하면 원본
    back, _ = sym.augment(env=fake_env(), obs=out_obs[B:], actions=None)
    assert torch.equal(back["policy"][B:], obs["policy"])


def test_unknown_group_stops():
    td = pytest.importorskip("tensordict")
    obs = td.TensorDict({"policy": torch.zeros(2, ts.obs_dim(SPEC)), "x": torch.zeros(2, 3)}, [2])
    with pytest.raises(KeyError):
        sym.augment(env=fake_env(), obs=obs, actions=None)


def test_rsl_rl_symmetry_extension_uses_it():
    """rsl_rl Symmetry 가 문자열 "symmetry:augment" (Isaac Lab to_dict 가 만드는 형태) 로 찾고,
    augment_batch 가 원본 뒤에 거울을 붙이며 다른 텐서를 두 배로 늘리고, 손실이 계산된다."""
    pytest.importorskip("tensordict")
    ext = pytest.importorskip("rsl_rl.extensions.symmetry")
    models = pytest.importorskip("rsl_rl.models")
    B = 16
    obs = make_obs(B)
    batch = SimpleNamespace(
        observations=obs.clone(),
        actions=torch.randn(B, 2),
        old_actions_log_prob=torch.randn(B, 1),
        values=torch.randn(B, 1),
        advantages=torch.randn(B, 1),
        returns=torch.randn(B, 1),
    )
    s = ext.Symmetry(
        fake_env(), "symmetry:augment", use_data_augmentation=True, use_mirror_loss=True,
        mirror_loss_coeff=1.0,
    )
    assert s.data_augmentation_func is sym.augment
    s.augment_batch(batch, B)
    assert batch.observations.batch_size[0] == 2 * B and batch.values.shape == (2 * B, 1)
    assert torch.equal(batch.observations["policy"][B:], sym.mirror_obs(obs["policy"], SPEC))
    actor = models.MLPModel(
        batch.observations, {"actor": ["policy"]}, "actor", 2, hidden_dims=[16, 16],
        distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.5},
    )
    loss = s.compute_loss(actor, batch, B)
    with torch.no_grad():
        m = actor(obs)
        mm = actor(batch.observations[B:])
    want = torch.nn.functional.mse_loss(mm, sym.mirror_actions(m))
    assert loss.requires_grad and torch.allclose(loss, want, atol=1e-6)
