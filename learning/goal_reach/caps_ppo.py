"""CAPS (Mysore et al. 2021): 행동 평균의 매끄러움 항을 보상이 아니라 PPO 손실에 더한다 (torch, rsl_rl 만).

시간 항 = 같은 env 의 t 와 t+1 관측에서 낸 행동 평균의 차 제곱 평균 (t 에서 에피소드가 끝난 쌍은 뺌).
공간 항 = 관측과 그 관측에 가우스 잡음을 더한 관측에서 낸 행동 평균의 차 제곱 평균.
잡음은 액터의 관측 정규화 뒤(정규화된 단위)에 더한다: 관측 항목마다 단위와 크기가 달라
sigma 하나를 정규화 전에 쓰면 항목마다 뜻이 다르기 때문이다.
두 항은 행동 평균(액터 MLP)만 지나므로 std 파라미터와 크리틱에는 기울기가 가지 않는다.

train.py 는 계수가 0 보다 클 때만 class_name 을 "caps_ppo:CapsPPO" 로 둔다 (아니면 rsl_rl PPO 그대로).
손실은 PPO.update 의 미니배치마다 optimizer.zero_grad() 바로 뒤에 backward 해서 PPO 손실의
기울기에 더한다: 합의 기울기와 같고, 기울기 자르기(max_grad_norm)와 optimizer.step 을 함께 거친다.
"""

from __future__ import annotations

import torch

from rsl_rl.algorithms import PPO


def action_mean(actor, obs, noise_sigma: float = 0.0) -> torch.Tensor:
    """액터의 행동 평균. noise_sigma > 0 이면 정규화된 관측에 N(0, sigma) 잡음을 더한다."""
    latent = actor.get_latent(obs)
    if noise_sigma > 0:
        latent = latent + noise_sigma * torch.randn_like(latent)
    return actor.distribution.deterministic_output(actor.mlp(latent))


def temporal_pair_index(dones: torch.Tensor) -> torch.Tensor:
    """[T, N, 1] dones 에서 쌍 (t, t+1) 을 쓸 수 있는 평탄 인덱스 t*N+n (t < T-1, t 에서 안 끝남)."""
    done = dones.reshape(dones.shape[0], dones.shape[1]).bool()
    ok = ~done[:-1]
    t, n = torch.nonzero(ok, as_tuple=True)
    return t * done.shape[1] + n


def temporal_loss(actor, obs_flat, pair_idx: torch.Tensor, num_envs: int) -> torch.Tensor:
    """평탄 인덱스 i 와 i+num_envs (같은 env 의 다음 스텝) 의 행동 평균 차 제곱 평균."""
    if pair_idx.numel() == 0:
        return torch.zeros((), device=pair_idx.device)
    a = action_mean(actor, obs_flat[pair_idx])
    b = action_mean(actor, obs_flat[pair_idx + num_envs])
    return (a - b).pow(2).mean()


def spatial_loss(actor, obs, sigma: float) -> torch.Tensor:
    """관측과 잡음 더한 관측의 행동 평균 차 제곱 평균."""
    return (action_mean(actor, obs) - action_mean(actor, obs, sigma)).pow(2).mean()


class CapsPPO(PPO):
    """rsl_rl PPO + CAPS 시간/공간 매끄러움 손실 (피드포워드 정책만)."""

    def __init__(
        self,
        actor,
        critic,
        storage,
        caps_temporal: float = 0.0,
        caps_spatial: float = 0.0,
        caps_sigma: float = 0.0,
        **kwargs,
    ) -> None:
        super().__init__(actor, critic, storage, **kwargs)
        if min(caps_temporal, caps_spatial, caps_sigma) < 0:
            raise ValueError("CAPS coefficients and sigma must be >= 0")
        if caps_spatial > 0 and not caps_sigma > 0:
            raise ValueError("caps_spatial needs caps_sigma above 0")
        if actor.is_recurrent:
            raise ValueError("CAPS is implemented for feedforward actors only")
        self.caps_temporal = float(caps_temporal)
        self.caps_spatial = float(caps_spatial)
        self.caps_sigma = float(caps_sigma)

    def caps_loss(self, obs_flat, pair_idx: torch.Tensor, num_envs: int, size: int):
        """롤아웃에서 size 개를 뽑아 (가중합 손실, 시간 항, 공간 항)."""
        actor = self._raw_actor
        device = pair_idx.device
        zero = torch.zeros((), device=device)
        loss_t = zero
        if self.caps_temporal > 0 and pair_idx.numel() > 0:
            pick = pair_idx[torch.randint(pair_idx.numel(), (size,), device=device)]
            loss_t = temporal_loss(actor, obs_flat, pick, num_envs)
        loss_s = zero
        if self.caps_spatial > 0:
            pick = torch.randint(obs_flat.batch_size[0], (size,), device=device)
            loss_s = spatial_loss(actor, obs_flat[pick], self.caps_sigma)
        return self.caps_temporal * loss_t + self.caps_spatial * loss_s, loss_t, loss_s

    def update(self) -> dict[str, float]:
        if not (self.caps_temporal > 0 or self.caps_spatial > 0):
            return super().update()
        # ---- 롤아웃 전체에서 쌍과 관측 준비 (PPO 미니배치 크기만큼 뽑음) ----
        st = self.storage
        obs_flat = st.observations.flatten(0, 1)
        pair_idx = temporal_pair_index(st.dones)
        size = st.num_envs * st.num_transitions_per_env // self.num_mini_batches
        sums = {"caps_temporal": 0.0, "caps_spatial": 0.0}
        stock_zero_grad = self.optimizer.zero_grad
        own = self.optimizer.__dict__.get("zero_grad")

        # ---- 미니배치마다 zero_grad 뒤에 CAPS 기울기를 먼저 쌓는다 ----
        def zero_grad_then_caps(*args, **kwargs):
            stock_zero_grad(*args, **kwargs)
            loss, loss_t, loss_s = self.caps_loss(obs_flat, pair_idx, st.num_envs, size)
            if loss.requires_grad:
                loss.backward()
            sums["caps_temporal"] += loss_t.item()
            sums["caps_spatial"] += loss_s.item()

        self.optimizer.zero_grad = zero_grad_then_caps
        try:
            loss_dict = super().update()
        finally:
            if own is None:
                del self.optimizer.zero_grad
            else:
                self.optimizer.zero_grad = own
        num_updates = self.num_learning_epochs * self.num_mini_batches
        loss_dict.update({k: v / num_updates for k, v in sums.items()})
        return loss_dict
