"""시뮬 없는 과제 규칙 테스트. 실행: <venv python> -m pytest learning/goal_reach -q"""

import dataclasses
import math
import os
import sys

import pytest
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import task_spec as ts  # noqa: E402


def test_outcome_is_exclusive_and_failure_wins():
    # env 0: 도달; 1: 충돌하며 도달; 2: 이탈하며 도달; 3: 충돌만
    dist = torch.tensor([0.01, 0.01, 0.01, 1.0])
    force = torch.tensor([0.0, 5.0, 0.0, 5.0])
    xy = torch.tensor([[1.0, 0.0], [1.0, 0.0], [3.5, 0.0], [0.5, 0.0]])
    succ, coll, oob = ts.episode_outcome(dist, force, xy, torch.zeros(4, 2))
    assert succ.tolist() == [True, False, False, False]
    assert coll.tolist() == [False, True, False, True]
    assert oob.tolist() == [False, False, True, False]
    assert ((succ.int() + coll.int() + oob.int()) <= 1).all()


def test_oob_with_collision_counts_as_collision_only():
    succ, coll, oob = ts.episode_outcome(
        torch.tensor([1.0]), torch.tensor([5.0]), torch.tensor([[3.5, 0.0]]), torch.zeros(1, 2)
    )
    assert (succ.item(), coll.item(), oob.item()) == (False, True, False)


def test_reward_no_arrival_bonus_on_collision():
    succ, coll, oob = ts.episode_outcome(
        torch.tensor([0.01]), torch.tensor([5.0]), torch.tensor([[1.0, 0.0]]), torch.zeros(1, 2)
    )
    _, terms = ts.compute_reward(torch.tensor([0.02]), torch.tensor([0.01]), succ, coll | oob)
    assert terms["arrival"].item() == 0.0
    assert terms["collision"].item() == ts.SPEC.w_collision


def test_spec_overrides_reject_env_shaping_fields():
    assert ts.make_spec({"w_time": -0.2}).w_time == -0.2
    for k in ("control_dt", "episode_s", "lidar_sectors", "lidar_obs_max"):
        with pytest.raises(ValueError):
            ts.make_spec({k: 1.0})


def test_rate_limit_and_wheels():
    cmd = ts.rate_limit(torch.zeros(1, 2), torch.tensor([[0.25, 1.0]]), 0.05)
    assert torch.allclose(cmd, torch.tensor([[0.1, 0.1]]))
    w = ts.twist_to_wheels(torch.tensor([[0.0, 1.0]]))
    assert torch.allclose(w, torch.tensor([[-0.0961 / 2 / 0.028, 0.0961 / 2 / 0.028]]))


def test_lidar_sectors_index_and_inf():
    r = torch.full((1, 360), float("inf"))
    r[0, 185] = 0.3  # 레이 185 = +5 deg, 섹터 18 은 레이 180..189
    s = ts.lidar_sectors(r)
    assert s[0, 18].item() == pytest.approx(0.3)
    assert (s[0, :18] == ts.SPEC.lidar_obs_max).all()


def test_a_tuple_field_is_set_with_comma_separated_numbers():
    got = ts.parse_overrides(["goal_dist_range=0.5,1.0", "w_time=-0.2"])
    assert got == {"goal_dist_range": (0.5, 1.0), "w_time": -0.2}
    assert ts.make_spec(got).goal_dist_range == (0.5, 1.0)


def test_a_tuple_field_with_the_wrong_count_is_refused():
    with pytest.raises(ValueError, match="goal_dist_range"):
        ts.parse_overrides(["goal_dist_range=1.5"])


def test_an_outcome_on_the_last_step_is_not_also_a_timeout():
    # env 0: 마지막 스텝 성공; 1: 마지막 스텝에도 주행 중; 2: 일찍 성공
    done = torch.tensor([True, False, True])
    length = torch.tensor([400, 400, 120])
    terminated, time_out = ts.split_dones(done, length, 400)
    assert terminated.tolist() == [True, False, True]
    assert time_out.tolist() == [False, True, False]


def _obs_inputs(n=2):
    return torch.full((n, 360), 1.0), torch.ones(n), torch.zeros(n), torch.zeros(n, 2)


def test_the_default_observation_is_the_41_dim_layout():
    assert ts.OBS_DIM == ts.obs_dim(ts.SPEC) == 41 == len(ts.obs_names())
    obs = ts.build_observation(*_obs_inputs())
    assert obs.shape == (2, 41)


def test_intent_observation_layout_mask_and_clip():
    spec = ts.make_spec(ts.parse_overrides(["intent_robots=2"]))
    assert spec.intent_robots == 2 and isinstance(spec.intent_robots, int)
    assert ts.obs_dim(spec) == 41 + 2 * 7 == len(ts.obs_names(spec))
    intent = torch.zeros(2, 2, 3, 2)
    intent[:, 0] = torch.tensor([[1.0, 0.0], [3.0, 0.5], [-5.0, -1.0]])  # m, 로봇 프레임
    intent[:, 1] = 9.0  # 없는 로봇: 0 으로 나와야 함
    mask = torch.tensor([[True, False], [True, False]])
    obs = ts.build_observation(*_obs_inputs(), spec, intent=intent, intent_mask=mask)
    names = ts.obs_names(spec)
    row = dict(zip(names, obs[0].tolist(), strict=True))
    assert row["robot0_present"] == 1.0 and row["robot1_present"] == 0.0
    assert (row["robot0_p1_x"], row["robot0_p2_x"], row["robot0_p3_x"]) == (0.5, 1.0, -1.0)
    assert row["robot0_p2_y"] == 0.25
    assert all(row[f"robot1_p{k}_{a}"] == 0.0 for k in (1, 2, 3) for a in "xy")


def test_intent_layout_without_intent_input_is_all_absent():
    # 플릿 로봇이 없는 env (goal-reach) 도 넓은 배치를 만듦: 로봇은 없음
    spec = ts.make_spec({"intent_robots": 2})
    obs = ts.build_observation(*_obs_inputs(), spec)
    assert obs.shape == (2, 55)
    assert (obs[:, 41:] == 0).all()


def test_the_top_speed_is_the_one_source(tmp_path):
    import yaml

    with open(ts.NAV_SPEED_FILE, encoding="utf-8") as f:
        want = yaml.safe_load(f)["nav_speed_mps"]
    assert ts.ACTION_LIMITS.v_max == ts.NAV_SPEED_MPS == want
    other = tmp_path / "nav_speed.yaml"
    other.write_text("nav_speed_mps: 0.2\n", encoding="utf-8")
    assert ts.nav_speed_mps(str(other)) == 0.2
    other.write_text("nav_speed_mps: 0\n", encoding="utf-8")
    with pytest.raises(ValueError):
        ts.nav_speed_mps(str(other))


def test_times_stretch_so_the_robot_covers_the_tuned_distance():
    """규칙: TIMES_TUNED_AT_MPS (0.25 m/s) 에 맞춘 시간은 0.25 / v 배."""
    assert ts.TIMES_TUNED_AT_MPS == 0.25
    assert ts.travel_time(20.0, 0.25) == pytest.approx(20.0)
    assert ts.travel_time(20.0, 0.12) == pytest.approx(20.0 * 0.25 / 0.12)
    assert ts.SPEC.episode_s == pytest.approx(ts.travel_time(20.0))
    assert ts.SPEC.episode_s * ts.NAV_SPEED_MPS == pytest.approx(20.0 * 0.25)


def test_priority_flag_follows_the_present_flag_when_on():
    spec = ts.make_spec(ts.parse_overrides(["intent_robots=2", "intent_priority=1"]))
    names = ts.obs_names(spec)
    assert ts.obs_dim(spec) == 41 + 2 * 8 == len(names)
    assert names[41:43] == ["robot0_present", "robot0_priority"]
    intent = torch.ones(2, 2, 3, 2)
    mask = torch.tensor([[True, True], [True, False]])
    prio = torch.tensor([[True, False], [True, True]])
    obs = ts.build_observation(*_obs_inputs(), spec, intent=intent, intent_mask=mask,
                               intent_priority=prio)
    row1 = dict(zip(names, obs[1].tolist(), strict=True))
    assert row1["robot0_priority"] == 1.0 and row1["robot1_priority"] == 0.0  # 없는 로봇: 0
    off = ts.make_spec(ts.parse_overrides(["intent_robots=2"]))
    assert "robot0_priority" not in ts.obs_names(off)  # 기본 꺼짐: 이전과 같은 55차원


def _base_inputs(n=3, seed=0):
    g = torch.Generator().manual_seed(seed)
    ranges = 0.1 + 2.5 * torch.rand(n, 360, generator=g)
    vel = torch.rand(n, 2, generator=g)
    return ranges, torch.rand(n, generator=g), torch.rand(n, generator=g), vel


def test_optional_blocks_go_between_base_and_intent_in_order():
    spec = dataclasses.replace(
        ts.SPEC, prev_action_obs=1, lidar_frames=3, speed_cap_obs=1, intent_robots=1
    )
    assert ts.obs_blocks(spec) == ["prev_action", "lidar_history", "speed_cap", "intent"]
    names = ts.obs_names(spec)
    assert len(names) == ts.obs_dim(spec) == 41 + 2 + 72 + 1 + 7
    assert names[41:43] == ["prev_v", "prev_w"]
    assert names[43] == "lidar_hist1_sector_0" and names[43 + 36] == "lidar_hist2_sector_0"
    assert names[115] == "speed_cap" and names[116] == "robot0_present"
    ranges, d, a, vel = _base_inputs()
    hist = torch.rand(3, 2, 36)
    obs = ts.build_observation(
        ranges, d, a, vel, spec, prev_target=torch.tensor([[0.06, -0.5]] * 3),
        lidar_history=hist, speed_cap=torch.full((3,), 0.16),
    )
    base = ts.build_observation(ranges, d, a, vel)
    assert obs.shape == (3, ts.obs_dim(spec))
    assert torch.equal(obs[:, :41], base)  # 기본 항목은 그대로
    v_scale = ts.ACTION_LIMITS.v_max
    assert torch.allclose(obs[:, 41:43], torch.tensor([0.06 / v_scale, -0.5]).expand(3, 2))
    assert torch.equal(obs[:, 43:115], hist.reshape(3, -1))  # 최신 먼저, 준 그대로
    assert torch.allclose(obs[:, 115], torch.full((3,), 0.16 / v_scale))
    assert torch.equal(obs[:, 116:], torch.zeros(3, 7))  # intent None: 없음


def test_missing_block_inputs_take_their_episode_start_values():
    spec = dataclasses.replace(ts.SPEC, prev_action_obs=1, lidar_frames=3, speed_cap_obs=1)
    ranges, d, a, vel = _base_inputs()
    obs = ts.build_observation(ranges, d, a, vel, spec)
    assert torch.equal(obs[:, 41:43], torch.zeros(3, 2))
    assert torch.equal(obs[:, 43:79], obs[:, :36]) and torch.equal(obs[:, 79:115], obs[:, :36])
    assert torch.allclose(obs[:, 115], torch.ones(3))


def test_default_spec_keeps_the_ppo_k_layout():
    assert ts.obs_blocks(ts.SPEC) == []
    assert ts.obs_dim(ts.SPEC) == ts.BASE_DIM == 41


def test_speed_cap_sets_the_v_upper_limit_and_still_action_stops():
    lim = ts.ACTION_LIMITS
    cap = torch.tensor([0.08, 0.12, 0.16])
    full = ts.action_to_target(torch.ones(3, 2), v_cap=cap)
    assert torch.allclose(full[:, 0], cap)
    low = ts.action_to_target(-torch.ones(3, 2), v_cap=cap)
    assert torch.allclose(low[:, 0], torch.full((3,), lim.v_min))
    assert torch.allclose(ts.action_to_target(torch.ones(1, 2))[:, 0], torch.tensor([lim.v_max]))
    still = ts.action_to_target(ts.still_action(3, v_cap=cap), v_cap=cap)
    assert torch.allclose(still, torch.zeros(3, 2), atol=1e-7)
    assert torch.allclose(ts.action_to_target(ts.still_action(2)), torch.zeros(2, 2), atol=1e-7)


def test_capped_action_is_the_full_speed_target_clamped_to_the_cap():
    lim = ts.ACTION_LIMITS
    cap = torch.tensor([0.08, 0.08, 0.16, 0.16, 0.12])
    a = torch.tensor([[1.0, 0.3], [-1.0, -0.5], [1.0, 1.0], [0.2, 0.0], [1.5, -2.0]])
    full = ts.action_to_target(a)
    got = ts.action_to_target(ts.capped_action(a, cap), v_cap=cap)
    assert torch.allclose(got[:, 0], torch.minimum(full[:, 0], cap), atol=1e-7)
    assert torch.allclose(got[:, 1], full[:, 1])
    assert torch.allclose(got[2, 0], torch.tensor(lim.v_max))  # 자기 최고속도보다 빠르지 않음
    assert torch.allclose(got[4], full[4], atol=1e-7)  # 학습한 최고속도에서는 그대로


def test_block_forward_zeroes_the_forward_v_only_when_the_front_cone_is_near():
    rays = torch.full((6, 360), 2.0)  # 레이는 -180 deg 부터 1 deg 씩: 정면은 index 180
    rays[0, 180] = 0.15  # 정면이 가깝다
    rays[1, 180 + 30] = 0.15  # 부채꼴의 끝 레이 (포함)
    rays[2, 180 + 31] = 0.15  # 부채꼴 바로 밖
    rays[3, 180] = 0.15  # 가깝지만 후진 목표
    rays[4, 180] = 0.20  # 거리가 문턱과 같다 (막지 않음)
    rays[5, 0] = 0.05  # 뒤만 가깝다
    target = torch.tensor(
        [[0.12, 0.4], [0.12, -0.4], [0.12, 0.4], [-0.05, 0.4], [0.12, 0.4], [0.12, 0.4]]
    )
    out, blocked = ts.block_forward(target, rays, 0.20, 30.0)
    assert blocked.tolist() == [True, True, False, False, False, False]
    assert torch.equal(out[:2, 0], torch.zeros(2))
    assert torch.equal(out[:, 1], target[:, 1])  # w 는 그대로
    assert torch.equal(out[2:], target[2:])
    assert torch.equal(target[0], torch.tensor([0.12, 0.4]))  # 입력을 고치지 않음


def test_block_forward_is_off_at_zero_range_and_follows_the_ray_count():
    rays = torch.full((2, 360), 0.06)
    target = torch.tensor([[0.12, 0.0], [0.08, 1.0]])
    out, blocked = ts.block_forward(target, rays, 0.0, 30.0)
    assert not blocked.any() and torch.equal(out, target)
    coarse = torch.full((1, 36), 2.0)  # 10 deg 간격: 정면은 index 18
    coarse[0, 18 + 6] = 0.1  # +60 deg
    assert ts.block_forward(target[:1], coarse, 0.2, 60.0)[1].tolist() == [True]
    assert ts.block_forward(target[:1], coarse, 0.2, 59.0)[1].tolist() == [False]


def test_parse_front_stop_wants_a_finite_positive_range_and_a_half_width_up_to_180():
    assert ts.parse_front_stop("0.20,30") == (0.2, 30.0)
    assert ts.parse_front_stop(" 0.15 , 180 ") == (0.15, 180.0)
    for bad in ("0.2", "0.2,30,1", "0,30", "-0.1,30", "nan,30", "inf,30", "0.2,0", "0.2,181",
                "0.2,nan", "a,b", ""):
        with pytest.raises(ValueError):
            ts.parse_front_stop(bad)


def test_speed_cap_draw_stays_in_range_and_a_single_value_draws_nothing():
    spec = dataclasses.replace(ts.SPEC, speed_cap_range=(0.08, 0.16))
    torch.manual_seed(0)
    c = ts.sample_speed_cap(10000, "cpu", spec)
    assert c.min() >= 0.08 and c.max() <= 0.16 and abs(c.mean().item() - 0.12) < 0.002
    state = torch.random.get_rng_state()
    assert torch.equal(ts.sample_speed_cap(4, "cpu"), torch.full((4,), ts.NAV_SPEED_MPS))
    assert torch.equal(state, torch.random.get_rng_state())  # 고정 상한: 난수 흐름 그대로


def test_lidar_scale_draw_stays_in_range_and_a_single_value_draws_nothing():
    spec = ts.make_spec(ts.parse_overrides(["lidar_scale_range=0.95,1.03"]))
    assert spec.lidar_scale_range == (0.95, 1.03)
    torch.manual_seed(0)
    s = ts.sample_lidar_scale(10000, "cpu", spec)
    assert s.min() >= 0.95 and s.max() <= 1.03 and abs(s.mean().item() - 0.99) < 0.002
    state = torch.random.get_rng_state()
    assert torch.equal(ts.sample_lidar_scale(4, "cpu"), torch.ones(4))
    fixed = dataclasses.replace(ts.SPEC, lidar_scale_range=(0.958, 0.958))
    assert torch.equal(ts.sample_lidar_scale(3, "cpu", fixed), torch.full((3,), 0.958))
    assert torch.equal(state, torch.random.get_rng_state())  # 한 값: 난수 흐름 그대로


def test_lidar_scale_multiplies_ranges_before_the_sector_clip():
    # 레이 360 개, 섹터 0 에 1.0 m, 섹터 1 에 1.95 m, 나머지는 inf (자르기로 2.0 m)
    r = torch.full((2, 360), float("inf"))
    r[:, 0] = 1.0
    r[:, 10] = 1.95
    scale = torch.tensor([0.958, 1.03])
    lid = ts.lidar_sectors(ts.scale_ranges(r, scale)) / ts.SPEC.lidar_obs_max
    assert torch.allclose(lid[:, 0], scale * 1.0 / 2.0)
    # 1.95 × 1.03 = 2.0085 는 자르기 뒤 1, 1.95 × 0.958 은 그대로
    assert torch.allclose(lid[:, 1], torch.tensor([1.95 * 0.958 / 2.0, 1.0]))
    assert torch.equal(lid[:, 2:], torch.ones(2, 34))
    obs = ts.build_observation(
        ts.scale_ranges(r, scale), torch.ones(2), torch.zeros(2), torch.zeros(2, 2)
    )
    assert torch.equal(obs[:, : ts.OBS_LIDAR], lid)


def test_lidar_scale_default_leaves_the_observation_unchanged():
    torch.manual_seed(1)
    r = 0.05 + 3.0 * torch.rand(5, 360)
    r[0, 7] = float("nan")
    one = ts.sample_lidar_scale(5, "cpu")
    args = (torch.rand(5), torch.rand(5), torch.rand(5, 2))
    spec = dataclasses.replace(ts.SPEC, lidar_frames=3)
    got = ts.build_observation(ts.scale_ranges(r, one), *args, spec)
    assert torch.equal(got, ts.build_observation(r, *args, spec))


def test_flip_step_counts_like_nav_bench_sign_changes():
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(here, "..", "..", "tools", "nav_bench"))
    import metrics

    g = torch.Generator().manual_seed(3)
    seq = (torch.rand(200, 5, generator=g) - 0.5) * 0.2  # deadband 근처 값이 많음
    seq[:, 0] = 0.0
    seq[50, 1] = 0.02  # deadband 와 같으면 셈 (nav_bench 는 |w| < deadband 만 건너뜀)
    last = torch.zeros(5)
    count = torch.zeros(5)
    for w in seq:
        f, last = ts.flip_step(w, last, 0.02)
        count += f.float()
    for e in range(5):
        assert count[e].item() == metrics.sign_changes(seq[:, e].tolist(), 0.02)


def test_action_rate_terms():
    spec = dataclasses.replace(ts.SPEC, w_action_rate=-0.1, w_w_flip=-0.5)
    lim = ts.ACTION_LIMITS
    t = torch.tensor([[lim.v_max, 1.0], [0.0, 0.0]])
    p = torch.tensor([[0.0, -1.0], [0.0, 0.0]])
    terms = ts.action_rate_terms(t, p, torch.tensor([True, False]), spec)
    assert torch.allclose(terms["action_rate"], torch.tensor([-0.1 * (1 + 4), 0.0]))
    assert torch.allclose(terms["w_flip"], torch.tensor([-0.5, 0.0]))


def test_progress_reward_scales_with_the_episode_cap():
    lim = ts.ACTION_LIMITS
    dt = ts.SPEC.control_dt
    no = torch.zeros(2, dtype=torch.bool)
    cap = torch.tensor([0.08, 0.16])
    _, terms = ts.compute_reward(cap * dt, torch.zeros(2), no, no, v_cap=cap)
    assert torch.allclose(terms["progress"], torch.ones(2))
    _, terms = ts.compute_reward(torch.tensor([lim.v_max * dt]), torch.zeros(1), no[:1], no[:1])
    assert torch.allclose(terms["progress"], torch.ones(1))


def test_privileged_block_is_in_our_frame_and_zero_when_absent():
    xy = torch.tensor([[1.0, 1.0]])
    yaw = torch.tensor([math.pi / 2])  # +y 를 봄
    pos = torch.tensor([[[1.0, 2.0], [5.0, 5.0]]])  # 1 m 앞, 멀리
    vel = torch.tensor([[[0.25, 0.0], [0.1, 0.1]]])  # +x 로 움직임 = 우리 오른쪽
    out = ts.privileged_block(pos, vel, torch.tensor([[True, False]]), xy, yaw)
    assert out.shape == (1, 2 * ts.PRIV_PER_OBJECT)
    assert torch.allclose(out[0, :5], torch.tensor([1.0, 0.5, 0.0, 0.0, -0.5]), atol=1e-6)
    assert torch.equal(out[0, 5:], torch.zeros(5))


# ---- 명령 응답 모델 (cmd_*_range) ----
def _run_response(targets, delay, tau, gain, spec, dt=0.05):
    n = targets.shape[1]
    buf = torch.zeros(n, ts.cmd_delay_slots(spec), 2)
    lag = torch.zeros(n, 2)
    out = []
    for x in targets:
        lag, buf = ts.cmd_response_step(buf, lag, x, delay, tau, gain, dt)
        out.append(lag)
    return torch.stack(out)


def test_cmd_delay_of_k_steps_shifts_the_input_by_k():
    spec = dataclasses.replace(ts.SPEC, cmd_delay_steps_range=(0, 3))
    torch.manual_seed(0)
    x = torch.randn(12, 4, 2)
    delay = torch.tensor([0, 1, 2, 3])
    y = _run_response(x, delay, torch.zeros(4), torch.ones(4, 2), spec)
    for e, k in enumerate(delay.tolist()):
        assert torch.equal(y[k:, e], x[: 12 - k, e])
        assert torch.equal(y[:k, e], torch.zeros(k, 2))


def test_cmd_lag_matches_the_first_order_step_response():
    dt = 0.05
    tau = torch.tensor([0.02, 0.065, 0.12])
    x = torch.ones(20, 3, 2)
    y = _run_response(x, torch.zeros(3, dtype=torch.long), tau, torch.ones(3, 2), ts.SPEC, dt)
    t = dt * torch.arange(1, 21, dtype=torch.float32)[:, None]
    want = 1.0 - torch.exp(-t / tau)
    assert torch.allclose(y[..., 0], want, atol=1e-6)
    assert torch.allclose(y[..., 1], want, atol=1e-6)


def test_cmd_gain_scales_the_steady_state():
    x = torch.tensor([[0.1, -0.5]]).expand(40, 1, 2)
    gain = torch.tensor([[0.95, 1.1]])
    spec = dataclasses.replace(ts.SPEC, cmd_delay_steps_range=(0, 2))
    y = _run_response(x, torch.tensor([2]), torch.tensor([0.07]), gain, spec)
    assert torch.allclose(y[-1, 0], torch.tensor([0.095, -0.55]), atol=1e-6)


def test_cmd_response_identity_is_bit_for_bit():
    assert not ts.cmd_response_active(ts.SPEC)
    delay, tau, gain = ts.sample_cmd_response(5, "cpu")
    assert delay.tolist() == [0] * 5 and tau.tolist() == [0.0] * 5
    assert torch.equal(gain, torch.ones(5, 2))
    torch.manual_seed(1)
    x = torch.randn(30, 5, 2) * torch.tensor([0.1, 1.0])
    x[3, 0, 1] = -0.0
    y = _run_response(x, delay, tau, gain, ts.SPEC)
    assert torch.equal(y, x)
    assert torch.equal(torch.signbit(y), torch.signbit(x))


def test_cmd_response_overrides_and_draws():
    spec = ts.make_spec(
        ts.parse_overrides(
            [
                "cmd_delay_steps_range=0,2",
                "cmd_lag_tau_range=0.0,0.12",
                "cmd_gain_v_range=0.95,1.05",
                "cmd_gain_w_range=0.85,1.15",
            ]
        )
    )
    assert spec.cmd_delay_steps_range == (0, 2)
    assert all(isinstance(k, int) for k in spec.cmd_delay_steps_range)
    assert ts.cmd_response_active(spec) and ts.cmd_delay_slots(spec) == 3
    delay, tau, gain = ts.sample_cmd_response(20000, "cpu", spec)
    assert set(delay.tolist()) == {0, 1, 2}
    assert 0.0 <= tau.min() and tau.max() <= 0.12
    assert 0.95 <= gain[:, 0].min() and gain[:, 0].max() <= 1.05
    assert 0.85 <= gain[:, 1].min() and gain[:, 1].max() <= 1.15
    # 한 값 범위는 난수를 쓰지 않음
    fixed = dataclasses.replace(
        ts.SPEC, cmd_delay_steps_range=(1, 1), cmd_lag_tau_range=(0.065, 0.065)
    )
    torch.manual_seed(3)
    before = torch.rand(1)
    torch.manual_seed(3)
    d, t, _ = ts.sample_cmd_response(4, "cpu", fixed)
    assert torch.equal(torch.rand(1), before)
    assert d.tolist() == [1] * 4 and torch.allclose(t, torch.full((4,), 0.065))
    with pytest.raises(ValueError):
        ts.sample_cmd_response(1, "cpu", dataclasses.replace(ts.SPEC, cmd_delay_steps_range=(2, 1)))


def test_cmd_response_resamples_only_reset_envs():
    """env 의 리셋 경로처럼 reset 한 env 만 새 값으로 덮는다."""
    spec = dataclasses.replace(
        ts.SPEC,
        cmd_delay_steps_range=(0, 5),
        cmd_lag_tau_range=(0.0, 0.2),
        cmd_gain_v_range=(0.5, 1.5),
        cmd_gain_w_range=(0.5, 1.5),
    )
    torch.manual_seed(4)
    delay, tau, gain = ts.sample_cmd_response(64, "cpu", spec)
    old = (delay.clone(), tau.clone(), gain.clone())
    ids = torch.tensor([3, 10, 40])
    d, t, g = ts.sample_cmd_response(len(ids), "cpu", spec)
    delay[ids], tau[ids], gain[ids] = d, t, g
    keep = torch.ones(64, dtype=torch.bool)
    keep[ids] = False
    assert torch.equal(delay[keep], old[0][keep]) and torch.equal(tau[keep], old[1][keep])
    assert torch.equal(gain[keep], old[2][keep])
    assert not torch.equal(tau[ids], old[1][ids])


def test_rate_limit_still_bounds_the_wheel_command_after_the_response_model():
    spec = dataclasses.replace(ts.SPEC, cmd_delay_steps_range=(1, 1))
    delay, tau, gain = ts.sample_cmd_response(2, "cpu", spec)
    gain = torch.tensor([[1.5, 1.5], [1.0, 1.0]])
    buf, lag, cmd = torch.zeros(2, 2, 2), torch.zeros(2, 2), torch.zeros(2, 2)
    lim = ts.ACTION_LIMITS
    step = torch.tensor((lim.acc_v * 0.05, lim.acc_w * 0.05))
    for x in (torch.tensor([[0.12, 1.0], [-0.05, -1.0]]),) * 5:
        r, buf = ts.cmd_response_step(buf, lag, x, delay, tau, gain, 0.05)
        lag = r
        new = ts.rate_limit(cmd, r, 0.05)
        assert ((new - cmd).abs() <= step + 1e-7).all()
        cmd = new


def test_cmd_delay_override_rejects_non_integer_steps():
    for text in ("cmd_delay_steps_range=-0.9,1.9", "cmd_delay_steps_range=0,1.5"):
        with pytest.raises(ValueError):
            ts.parse_overrides([text])
    assert ts.parse_overrides(["cmd_delay_steps_range=1.0,2"])["cmd_delay_steps_range"] == (1, 2)


# ---- 움직이는 것 앞에서만 서는 규칙 (평가 전용) ----
LIDAR_X = -0.017  # 시험 스캔의 라이다 자리 (base 에서 앞쪽 m)
ROOM = (-1.0, -1.0, 1.5, 1.2)
BOX = (0.30, -0.06, 0.42, 0.10)


def _scan(origin, yaw, rects=(), circles=(), n_rays=360, far=9.0):
    """시험용 2D 스캔 (R,): 원점과 yaw 에서 축에 나란한 사각형 (x0, y0, x1, y1) 과 원 (x, y, r)
    까지의 레이 거리. 원점이 사각형 안이면 그 사각형은 방의 벽이다 (나가는 쪽 거리)."""
    step = 360.0 / n_rays
    ang = yaw + torch.deg2rad(-180.0 + torch.arange(n_rays, dtype=torch.float64) * step)
    dx, dy = torch.cos(ang), torch.sin(ang)
    ox, oy = float(origin[0]), float(origin[1])
    best = torch.full((n_rays,), far, dtype=torch.float64)
    miss = torch.full((n_rays,), float("inf"), dtype=torch.float64)
    for x0, y0, x1, y1 in rects:
        tx = torch.stack(((x0 - ox) / dx, (x1 - ox) / dx))
        ty = torch.stack(((y0 - oy) / dy, (y1 - oy) / dy))
        enter = torch.maximum(tx.amin(0), ty.amin(0))
        leave = torch.minimum(tx.amax(0), ty.amax(0))
        hit = leave >= torch.clamp(enter, min=0.0)
        best = torch.minimum(best, torch.where(hit, torch.where(enter > 0.0, enter, leave), miss))
    for cx, cy, r in circles:
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - r * r)
        t = -b - torch.sqrt(torch.clamp(disc, min=0.0))
        best = torch.minimum(best, torch.where((disc >= 0.0) & (t > 0.0), t, miss))
    return best.float()


def _drive(guard, poses, scene, target=(0.12, 0.3)):
    """자세 열 (x, y, yaw) 을 따라 스캔을 만들어 guard.step 에 넣는다. scene(i) -> (사각형들, 원들).
    돌려주는 값: 스텝별 (막혔는지, 목표 [v, w], 정면 +-60 deg 의 최소 거리)."""
    out = []
    for i, (x, y, yaw) in enumerate(poses):
        rects, circles = scene(i)
        lidar = (x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw))
        rays = _scan(lidar, yaw, rects, circles)[None]
        tgt, blocked = guard.step(
            torch.tensor([target]), rays, torch.tensor([[x, y]]), torch.tensor([yaw])
        )
        out.append((bool(blocked[0]), tgt[0].tolist(), float(rays[0, 120:241].min())))
    return out


def test_static_ranges_moves_an_old_scan_into_the_pose_of_now():
    rays = torch.full((1, 360), 5.0)
    rays[0, 180] = 1.0  # 정면 1 m 에 점 하나
    at0, yaw0 = torch.zeros(1, 2), torch.zeros(1)
    # 0.1 m 앞으로 갔다: 그 점은 정면 0.9 m
    out = ts.static_ranges(rays, at0, yaw0, torch.tensor([[0.1, 0.0]]), yaw0, 0)
    assert out[0, 180].item() == pytest.approx(0.9, abs=1e-5)
    # 왼쪽으로 10 deg 돌았다: 그 점은 오른쪽 10 deg (index 170)
    out = ts.static_ranges(rays, at0, yaw0, at0, torch.tensor([math.radians(10.0)]), 0)
    assert out[0, 170].item() == pytest.approx(1.0, abs=1e-5)
    assert out[0, 180].item() == pytest.approx(5.0, abs=1e-5)
    # 이웃 +-2 칸의 최솟값
    out = ts.static_ranges(rays, at0, yaw0, at0, yaw0, 2)
    assert out[0, 178:183].tolist() == pytest.approx([1.0] * 5, abs=1e-5)
    assert out[0, [177, 183]].tolist() == pytest.approx([5.0, 5.0], abs=1e-5)
    # 이웃은 한 바퀴를 넘어 이어진다 (index 0 은 -180 deg, 359 는 +179 deg)
    back = torch.full((1, 360), 5.0)
    back[0, 0] = 1.0
    out = ts.static_ranges(back, at0, yaw0, at0, yaw0, 2)
    assert out[0, [358, 359, 0, 1, 2]].tolist() == pytest.approx([1.0] * 5, abs=1e-5)
    assert out[0, [357, 3]].tolist() == pytest.approx([5.0, 5.0], abs=1e-5)


def test_closing_guard_stays_quiet_in_a_static_scene_while_the_robot_drives_and_turns():
    guard = ts.ClosingGuard(0.05, 0.50, 90.0, lidar_x=LIDAR_X)  # 방 안의 거의 모든 것이 "가깝다"
    x, y, yaw, poses = 0.0, 0.0, -0.5, []
    for _ in range(30):  # 0.12 m/s 로 가며 0.8 rad/s 로 돈다: 상자 옆 0.1 m 쯤까지 다가간다
        poses.append((x, y, yaw))
        x, y, yaw = x + 0.006 * math.cos(yaw), y + 0.006 * math.sin(yaw), yaw + 0.04
    steps = _drive(guard, poses, lambda i: ((ROOM, BOX), ()))
    assert min(s[2] for s in steps) < 0.20  # 실제로 가까이 갔다
    assert not any(s[0] for s in steps)
    assert all(s[1] == pytest.approx([0.12, 0.3]) for s in steps)
    # 같은 길에서 거리만 보는 앞 막기는 걸린다 (두 규칙이 다른 것을 본다는 대조)
    last = poses[-1]
    lidar = (last[0] + LIDAR_X * math.cos(last[2]), last[1] + LIDAR_X * math.sin(last[2]))
    rays = _scan(lidar, last[2], (ROOM, BOX))[None]
    assert ts.block_forward(torch.tensor([[0.12, 0.3]]), rays, 0.50, 90.0)[1].item()


def test_closing_guard_stops_for_something_that_comes_closer_inside_the_cone():
    def person(start, toward, speed=0.10):
        """start 에서 toward 방향 (단위 벡터로 고침) 으로 걷는 반지름 0.04 m 원."""
        n = math.hypot(*toward)
        ux, uy = toward[0] / n, toward[1] / n
        step = speed * 0.05
        return lambda i: ((ROOM,), ((start[0] + ux * step * i, start[1] + uy * step * i, 0.04),))

    still = [(0.0, 0.0, 0.0)] * 60

    def guard(half_deg=60.0):
        return ts.ClosingGuard(0.05, 0.26, half_deg, lidar_x=LIDAR_X)

    # 앞 왼쪽에서 로봇 쪽으로 온다: 정면 +-60 deg 안에서 0.26 m 보다 가까워진 스텝부터 막는다
    steps = _drive(guard(), still, person((0.50, 0.15), (-0.50, -0.15)))
    first = next(i for i, s in enumerate(steps) if s[2] < 0.26)
    assert 10 < first < 55
    assert [s[0] for s in steps[:first]] == [False] * first
    assert all(s[0] for s in steps[first:])
    assert steps[first][1] == pytest.approx([0.0, 0.3])  # v 만 0, w 는 그대로
    # 같은 자리에서 멀어진다: 가깝지만 막지 않는다
    steps = _drive(guard(), still, person((0.15, 0.04), (0.50, 0.15)))
    assert steps[0][2] < 0.26 and not any(s[0] for s in steps)
    # 뒤에서 온다: 부채꼴 밖이라 막지 않고, 부채꼴을 한 바퀴로 넓히면 막는다
    behind = person((-0.50, -0.15), (0.50, 0.15))
    assert not any(s[0] for s in _drive(guard(), still, behind))
    assert any(s[0] for s in _drive(guard(180.0), still, behind))


def test_closing_guard_ignores_a_standing_thing_the_robot_drives_up_to():
    guard = ts.ClosingGuard(0.05, 0.26, 60.0, lidar_x=LIDAR_X)
    poses = [(0.006 * i, 0.0, 0.0) for i in range(40)]  # 0.12 m/s 로 서 있는 사람에게 다가간다
    steps = _drive(guard, poses, lambda i: ((ROOM,), ((0.45, 0.02, 0.04),)))
    assert steps[-1][2] < 0.20  # 0.26 m 안으로 들어갔다
    assert not any(s[0] for s in steps)


def _ramp_scan(front_m, at=slice(172, 189)):
    """정면 +-8 deg (또는 at 의 레이) 만 front_m 이고 나머지는 1 m 인 스캔 (1, 360)."""
    rays = torch.full((1, 360), 1.0)
    rays[0, at] = front_m
    return rays


def test_closing_guard_holds_after_the_last_trigger_and_never_blocks_a_backward_target():
    guard = ts.ClosingGuard(0.05, 0.26, 60.0)
    at0, yaw0, fwd = torch.zeros(1, 2), torch.zeros(1), torch.tensor([[0.12, 0.3]])
    blocked = []
    for i in range(34):  # 0.40 m 에서 0.2 m/s 로 다가오다가 스텝 20 에 0.20 m 에서 선다
        front = 0.40 - 0.01 * min(i, 20)
        blocked.append(bool(guard.step(fwd, _ramp_scan(front), at0, yaw0)[1][0]))
    # 0.26 m 미만이 되는 스텝 15 부터 걸리고, 옛 스캔 (0.2 s 전) 과의 차이가 0.04 m/s 를 넘는
    # 마지막은 스텝 23 이다. 그 뒤 0.3 s (6 스텝) 를 유지하고 스텝 30 에 풀린다
    assert blocked == [False] * 15 + [True] * 15 + [False] * 4
    # 걸린 채로도 후진 목표와 0 은 건드리지 않고 막았다고 세지 않는다
    guard = ts.ClosingGuard(0.05, 0.26, 60.0)
    for i in range(18):
        back = torch.tensor([[-0.05, 0.3]]) if i % 2 else torch.tensor([[0.0, 0.3]])
        out, b = guard.step(back, _ramp_scan(0.40 - 0.01 * i), at0, yaw0)
        assert out.tolist() == back.tolist() and not b.item()


def test_closing_guard_treats_rays_without_a_return_as_far():
    at0, yaw0, fwd = torch.zeros(1, 2), torch.zeros(1), torch.tensor([[0.12, 0.3]])
    blocked = []
    for where in (slice(172, 189), slice(344, 360)):  # 정면과 뒤 왼쪽 (부채꼴은 한 바퀴)
        guard = ts.ClosingGuard(0.05, 0.26, 180.0)
        for i in range(18):  # 0.40 m 에서 0.2 m/s 로 다가오고, 다른 쪽 레이 몇은 돌아오지 않는다
            rays = _ramp_scan(0.40 - 0.01 * i, where)
            rays[0, 100:110] = float("nan")
            rays[0, 250:260] = float("inf")
            rays[0, 60:70] = float("-inf")
            blocked.append(bool(guard.step(fwd, rays, at0, yaw0)[1][0]))
    assert blocked == ([False] * 15 + [True] * 3) * 2
    # 돌아오지 않게 된 레이는 가까워진 것이 아니다
    guard = ts.ClosingGuard(0.05, 0.26, 60.0)
    for i in range(12):
        rays = _ramp_scan(0.30 if i < 6 else float("nan"))
        assert not guard.step(fwd, rays, at0, yaw0)[1].item()


def test_closing_guard_needs_fresh_scans_after_a_reset():
    guard = ts.ClosingGuard(0.05, 0.26, 60.0)
    at0, yaw0 = torch.zeros(2, 2), torch.zeros(2)
    fwd = torch.tensor([[0.12, 0.0], [0.12, 0.0]])
    blocked = []
    for i in range(20):  # 두 env 가 같은 것을 본다: 0.30 m 에서 0.08 m/s 로 다가온다
        rays = _ramp_scan(0.30 - 0.004 * i).repeat(2, 1)
        blocked.append(guard.step(fwd, rays, at0, yaw0)[1].tolist())
        if i == 12:
            guard.reset(torch.tensor([0]))  # env 0 만 리셋
    # 0.26 m 미만은 스텝 11 부터다. env 1 은 그때부터 계속 막히고, env 0 은 리셋 뒤 다섯 스텝
    # (리셋 직후의 스캔은 기준으로 쓰지 않고, 그 뒤 0.2 s 를 채워야 한다) 은 풀려 있다
    assert [b[1] for b in blocked] == [False] * 11 + [True] * 9
    assert [b[0] for b in blocked] == [False] * 11 + [True] * 2 + [False] * 5 + [True] * 2
    # 처음 다섯 스텝도 같다: 기준이 없으면 아무리 가까워져도 막지 않는다
    guard = ts.ClosingGuard(0.05, 0.26, 60.0)
    first = []
    for i in range(6):
        b = guard.step(fwd[:1], _ramp_scan(0.25 - 0.04 * i), at0[:1], yaw0[:1])[1]
        first.append(b.item())
    assert first == [False] * 5 + [True]


def test_closing_guard_with_a_long_lookback_keeps_blocking_for_a_person_crossing_dead_ahead():
    speed, steps = 0.08, 110

    def scene(i):  # 통로에 선 로봇 앞 0.22 m 를 사람 (반지름 0.04 m) 이 +y 에서 -y 로 가로지른다
        return [(-1.0, -0.4, 2.0, 0.4)], [(0.22, 0.30 - speed * 0.05 * i, 0.04)]

    poses = [(0.0, 0.0, 0.0)] * steps
    short = ts.ClosingGuard(0.05, 0.26, 60.0, lidar_x=LIDAR_X)
    wide = ts.ClosingGuard(
        0.05, 0.26, 60.0, lidar_x=LIDAR_X, lookback_s=1.0, spread_deg=4.0, closing_mps=0.08
    )
    a, b = _drive(short, poses, scene), _drive(wide, poses, scene)
    ahead = [i for i in range(steps) if abs(0.30 - speed * 0.05 * i) < 0.03]  # 정면 +-8 deg 안
    assert len(ahead) > 10
    # 시선을 가로지르는 동안에는 다가오는 속도가 0 에 가까워 짧은 창은 놓친다
    assert not any(a[i][0] for i in ahead)
    # 긴 창은 1 s 전에 비어 있던 자리에 들어온 것을 잡는다
    assert all(b[i][0] for i in ahead)


def test_closing_guard_min_rays_wants_that_many_neighbouring_rays():
    at0, yaw0, fwd = torch.zeros(1, 2), torch.zeros(1), torch.tensor([[0.12, 0.3]])

    def stops(width, min_rays):
        guard = ts.ClosingGuard(0.05, 0.26, 60.0, min_rays=min_rays)
        at = slice(180, 180 + width)
        return any(
            bool(guard.step(fwd, _ramp_scan(0.40 - 0.01 * i, at=at), at0, yaw0)[1][0])
            for i in range(18)
        )

    assert stops(1, 1) and stops(2, 2) and stops(3, 3)
    assert not stops(1, 2) and not stops(2, 3)
    # 떨어진 레이 둘은 이웃이 아니다
    guard = ts.ClosingGuard(0.05, 0.26, 60.0, min_rays=2)
    for i in range(18):
        rays = _ramp_scan(0.40 - 0.01 * i, at=slice(170, 171))
        rays[0, 190] = 0.40 - 0.01 * i
        assert not guard.step(fwd, rays, at0, yaw0)[1].item()


def test_closing_guard_noise_is_seeded_and_leaves_the_global_rng_alone():
    at0, yaw0, fwd = torch.zeros(1, 2), torch.zeros(1), torch.tensor([[0.12, 0.3]])

    def scans():
        guard = ts.ClosingGuard(0.05, 0.26, 60.0, noise_m=0.02)
        out = []
        for _ in range(8):
            guard.step(fwd, _ramp_scan(0.30), at0, yaw0)
            out.append(guard.scans[(guard.head - 1) % (guard.back + 1)].clone())
        return torch.stack(out)

    torch.manual_seed(5)
    before = torch.get_rng_state()
    a = scans()
    assert torch.equal(torch.get_rng_state(), before)
    assert torch.equal(a, scans())  # 같은 씨앗의 같은 잡음
    front = a[:, 0, 172:189]
    assert 0.01 < float(front.std()) < 0.03 and abs(float(front.mean()) - 0.30) < 0.01
    quiet = ts.ClosingGuard(0.05, 0.26, 60.0)
    quiet.step(fwd, _ramp_scan(0.30), at0, yaw0)
    assert torch.equal(quiet.scans[(quiet.head - 1) % (quiet.back + 1)], _ramp_scan(0.30))


def test_closing_guard_with_a_wide_margin_stays_quiet_on_noisy_scans_of_a_static_scene():
    def guard(**kw):
        return ts.ClosingGuard(0.05, 0.50, 90.0, lidar_x=LIDAR_X, noise_m=0.02, **kw)

    x, y, yaw, poses = 0.0, 0.0, -0.5, []
    for _ in range(60):  # 상자 옆으로 가다가 선다
        poses.append((x, y, yaw))
        if len(poses) < 30:
            x, y, yaw = x + 0.006 * math.cos(yaw), y + 0.006 * math.sin(yaw), yaw + 0.04
    wide = _drive(guard(lookback_s=1.0, spread_deg=4.0, closing_mps=0.08), poses,
                  lambda i: ((ROOM, BOX), ()))
    assert not any(s[0] for s in wide)
    # 같은 잡음에서 여유 8 mm 의 기본값은 서 있는 것에도 걸린다 (긴 창과 넓은 여유가 필요한 까닭)
    assert any(s[0] for s in _drive(guard(), poses, lambda i: ((ROOM, BOX), ())))


def test_closing_guard_rejects_a_neighbour_count_or_spread_wider_than_the_scan():
    at0, yaw0, fwd = torch.zeros(1, 2), torch.zeros(1), torch.tensor([[0.12, 0.3]])
    for kw in ({"min_rays": 361}, {"spread_deg": 180.0}):
        guard = ts.ClosingGuard(0.05, 0.26, 60.0, **kw)
        for _ in range(2):  # 다시 불러도 같은 말로 거절한다
            with pytest.raises(ValueError, match="wider than a scan"):
                guard.step(fwd, _ramp_scan(0.30), at0, yaw0)
    # 스캔 크기 안의 가장 큰 값은 돈다
    wide = ts.ClosingGuard(0.05, 0.26, 60.0, min_rays=360, spread_deg=179.0)
    assert not wide.step(fwd, _ramp_scan(0.30), at0, yaw0)[1].item()


def test_parse_closing_tune_reads_named_values_and_rejects_the_rest():
    assert ts.parse_closing_tune("") == {}
    text = "lookback_s=1.0,spread_deg=4,closing_mps=0.08,hold_s=0.5,noise_m=0.02,min_rays=2"
    assert ts.parse_closing_tune(text) == {
        "lookback_s": 1.0, "spread_deg": 4.0, "closing_mps": 0.08, "hold_s": 0.5,
        "noise_m": 0.02, "min_rays": 2,
    }
    assert isinstance(ts.parse_closing_tune("min_rays=3")["min_rays"], int)
    assert ts.parse_closing_tune("spread_deg=0,hold_s=0,noise_m=0") == {
        "spread_deg": 0.0, "hold_s": 0.0, "noise_m": 0.0,
    }
    bad = (
        "lookback=1", "lookback_s", "lookback_s=", "lookback_s=0", "spread_deg=-1",
        "closing_mps=0", "min_rays=0", "min_rays=1.5", "noise_m=-0.1", "hold_s=nan",
        "closing_mps=inf", "lookback_s=1,lookback_s=2", "lookback_s=1,,spread_deg=2",
    )
    for text in bad:
        with pytest.raises(ValueError):
            ts.parse_closing_tune(text)
    # 읽은 값은 그대로 ClosingGuard 의 인자가 된다
    guard = ts.ClosingGuard(0.05, 0.26, 60.0, **ts.parse_closing_tune("lookback_s=1.0,min_rays=2"))
    assert (guard.back, guard.min_rays, guard.noise_m) == (20, 2, 0.0)


def test_block_for_movers_stops_only_for_a_moving_active_mover_in_the_front_cone():
    n = 9
    xy, yaw = torch.zeros(n, 2), torch.full((n,), math.pi / 2)  # 로봇은 +y 를 본다

    def at(deg, d):
        a = math.radians(90.0 + deg)
        return [d * math.cos(a), d * math.sin(a)]

    spots = ((0, 0.25), (0, 0.25), (0, 0.25), (180, 0.25), (0, 0.35), (59, 0.25), (0, 0.25))
    pos = torch.tensor([at(*s) for s in spots + ((61, 0.25), (-59, 0.25))])
    vel = torch.tensor([[0.0, -0.1]] * n)
    vel[1] = torch.tensor([0.0, -0.01])  # 서 있는 것으로 본다 (0.02 m/s 미만)
    active = torch.ones(n, dtype=torch.bool)
    active[2] = False
    target = torch.tensor([[0.12, 0.4]] * n)
    target[6, 0] = -0.05  # 후진 목표
    out, blocked = ts.block_for_movers(target, xy, yaw, [(pos, vel, active)], 0.30, 60.0)
    #                           앞   섬     꺼짐   뒤     멀다   59도  후진   61도   -59도
    assert blocked.tolist() == [True, False, False, False, False, True, False, False, True]
    assert out[:, 0].tolist() == pytest.approx([0.0, 0.12, 0.12, 0.12, 0.12, 0.0, -0.05, 0.12, 0.0])
    assert out[:, 1].tolist() == pytest.approx([0.4] * n)
    assert target[0, 0].item() == pytest.approx(0.12)  # 입력은 그대로
    # mover 가 여럿이면 하나만 걸려도 막는다. 없으면 막지 않는다
    far = (pos + 2.0, vel, active)
    both = ts.block_for_movers(target, xy, yaw, [far, (pos, vel, active)], 0.30, 60.0)[1]
    assert both.tolist() == blocked.tolist()
    assert not ts.block_for_movers(target, xy, yaw, [], 0.30, 60.0)[1].any()


# ---- 몸에 붙은 것의 반환(lidar_posts)과 가림(lidar_mask_deg): 관측 레이에만 ----
POSTS = "lidar_posts=141,146,0.0715,-143,-140,0.0575"
MASK = "lidar_mask_deg=138,157,-152.5,-137"


def test_own_body_rays_is_off_by_default_and_returns_the_same_tensor():
    assert ts.SPEC.lidar_posts == () and ts.SPEC.lidar_mask_deg == ()
    r = torch.rand(3, 360)
    assert ts.own_body_rays(r) is r


def test_lidar_posts_cap_only_the_rays_in_range_and_keep_nearer_readings():
    spec = ts.make_spec(ts.parse_overrides([POSTS]))
    assert spec.lidar_posts == (141.0, 146.0, 0.0715, -143.0, -140.0, 0.0575)
    r = torch.full((2, 360), 1.0)
    r[0, 180 + 143] = float("inf")  # 반환 없음도 기둥 거리로
    r[1, 180 + 144] = float("nan")
    r[1, 180 + 145] = 0.03  # 더 가까운 것은 그대로
    before = r.clone()
    out = ts.own_body_rays(r, spec)
    # 받은 텐서는 바꾸지 않는다 (NaN 자리와 나머지 값을 따로 견준다)
    assert out is not r and torch.equal(r.isnan(), before.isnan())
    assert torch.equal(r[~r.isnan()], before[~before.isnan()])
    deg = torch.arange(360) - 180
    left = (deg >= 141) & (deg <= 146)
    right = (deg >= -143) & (deg <= -140)
    assert left.sum() == 6 and right.sum() == 4
    assert torch.allclose(out[0, left], torch.full((6,), 0.0715))
    assert torch.allclose(out[0, right], torch.full((4,), 0.0575))
    assert torch.equal(out[0, ~(left | right)], torch.ones(350))
    assert out[1, 180 + 144] == torch.tensor(0.0715) and out[1, 180 + 145] == torch.tensor(0.03)


def test_lidar_mask_turns_rays_into_no_return_after_the_posts():
    spec = ts.make_spec(ts.parse_overrides([POSTS, MASK]))
    r = torch.full((1, 360), 0.5)
    out = ts.own_body_rays(r, spec)
    deg = torch.arange(360) - 180
    masked = ((deg >= 138) & (deg <= 157)) | ((deg >= -152.5) & (deg <= -137))
    assert masked.sum() == 20 + 16
    assert torch.equal(out[0, masked], torch.full((36,), ts.SPEC.lidar_obs_max))
    assert torch.equal(out[0, ~masked], torch.full((324,), 0.5))  # 기둥은 모두 가려졌다
    # 섹터로 줄이면: 통째로 가린 섹터는 상한, 일부만 가린 섹터는 남은 레이의 최솟값
    sectors = ts.lidar_sectors(out, spec)
    assert sectors[0, 32] == ts.SPEC.lidar_obs_max and sectors[0, 3] == ts.SPEC.lidar_obs_max
    assert sectors[0, 31] == 0.5 and sectors[0, 33] == 0.5
    # 가림만 주면 기둥 없이 가린다
    only = ts.own_body_rays(r, ts.make_spec(ts.parse_overrides([MASK])))
    assert torch.equal(only, out)


def test_own_body_ranges_meet_at_the_back_seam():
    # 바로 뒤 레이는 -180 이고 180 으로도 가리킬 수 있다
    r = torch.full((1, 360), 0.5)
    a = ts.own_body_rays(r, ts.make_spec(ts.parse_overrides(["lidar_mask_deg=178,180"])))
    assert a[0, 0] == ts.SPEC.lidar_obs_max and a[0, 358] == ts.SPEC.lidar_obs_max
    assert a[0, 359] == ts.SPEC.lidar_obs_max and a[0, 1] == 0.5 and a[0, 357] == 0.5
    b = ts.own_body_rays(r, ts.make_spec(ts.parse_overrides(["lidar_posts=-180,-179,0.07"])))
    assert torch.allclose(b[0, :2], torch.full((2,), 0.07)) and b[0, 2] == 0.5 and b[0, 359] == 0.5


def test_lidar_posts_take_one_ray_ranges_and_keep_a_negative_infinity():
    # 기둥은 레이 하나짜리 범위(lo == hi)도 받는다. -inf 는 min 의 뜻대로 그대로 둔다
    spec = ts.make_spec(ts.parse_overrides(["lidar_posts=-149,-149,0.074"]))
    r = torch.full((1, 360), 0.5)
    r[0, 180 - 149] = float("-inf")
    out = ts.own_body_rays(r, spec)
    assert out[0, 180 - 149] == float("-inf")
    r[0, 180 - 149] = 0.5
    out = ts.own_body_rays(r, spec)
    assert torch.isclose(out[0, 180 - 149], torch.tensor(0.074)) and (out != 0.5).sum() == 1


@pytest.mark.parametrize("item", [
    "lidar_posts=141,146",  # 셋씩 묶이지 않는다
    "lidar_posts=146,141,0.07",  # lo > hi
    "lidar_posts=141,146,0.0",  # 거리 0
    "lidar_posts=141,181,0.07",  # 범위 밖
    "lidar_mask_deg=138",  # 둘씩 묶이지 않는다
    "lidar_mask_deg=157,138",
    "lidar_mask_deg=-181,-170",
    "lidar_posts=",  # 빈 값은 필드를 주지 않는 것으로 한다
    "lidar_posts=141,,146,0.0715",  # 빈 항목을 건너뛰어 묶음을 맞추지 않는다
    "lidar_mask_deg=138, ,157",
    "lidar_mask_deg=140,140",  # 플러그인의 scan_mask_deg 처럼 lo < hi
    "lidar_posts=141,146,inf",  # 거리는 유한해야 한다
    "lidar_posts=141,146,nan",
    "lidar_mask_deg=nan,150",
])
def test_own_body_overrides_reject_bad_groups(item):
    with pytest.raises(ValueError, match=item.split("=")[0]):
        ts.make_spec(ts.parse_overrides([item]))
