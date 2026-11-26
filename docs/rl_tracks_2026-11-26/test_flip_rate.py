import flip_rate as fr
import numpy as np
import pytest


def save(tmp_path, name, env, flips, length_s):
    d = tmp_path / name
    d.mkdir()
    np.savez(d / "episodes.npz", env=np.array(env), flips=np.array(flips, float), length_s=np.array(length_s, float))
    return d


def test_rate_is_total_flips_over_total_time_not_the_mean_of_episode_rates():
    # 에피소드별 비율의 평균이면 (6 + 0.667) / 2 = 3.33이다
    assert fr.rate(np.array([3.0, 1.0]), np.array([30.0, 90.0])) == pytest.approx(2.0)


def test_load_sorts_by_env_so_runs_recorded_in_a_different_order_line_up(tmp_path):
    a = fr.load(save(tmp_path, "a", [2, 0, 1], [5, 1, 3], [10, 20, 30]))
    assert a["env"].tolist() == [0, 1, 2]
    assert a["flips"].tolist() == [1.0, 3.0, 5.0] and a["length_s"].tolist() == [20.0, 30.0, 10.0]


def test_paired_difference_resamples_the_same_episodes_in_both_runs(tmp_path):
    env = np.arange(40)
    flips = np.arange(40) % 7
    length = 10.0 + np.arange(40) % 5
    a = fr.load(save(tmp_path, "a", env, flips, length))
    # 같은 에피소드를 다른 순서로 적은 실행: env로 맞추고 같은 에피소드를 같이 뽑으면 차이와 SE가 모두 0이다
    order = env[::-1]
    b = fr.load(save(tmp_path, "b", order, flips[order], length[order]))
    r = fr.compare(a, b, paired=True, n_boot=200)
    assert (r["kind"], r["diff"], r["se"]) == ("paired", 0.0, 0.0)
    # 따로 뽑으면 같은 자료라도 SE가 0이 아니다
    u = fr.compare(a, b, paired=False, n_boot=200)
    assert u["kind"] == "unpaired" and u["diff"] == 0.0 and u["se"] > 0.1


def test_difference_is_a_minus_b_and_the_error_is_reproducible(tmp_path):
    env = np.arange(30)
    a = fr.load(save(tmp_path, "a", env, np.full(30, 4.0), np.full(30, 60.0)))
    b = fr.load(save(tmp_path, "b", env, np.full(30, 1.0), np.full(30, 60.0)))
    r = fr.compare(a, b, paired=True, n_boot=100)
    assert (r["rate_a"], r["rate_b"], r["diff"]) == (4.0, 1.0, 3.0)
    assert r["se"] == 0.0 and r["n"] == 30  # 모든 에피소드가 같으면 다시 뽑아도 비율이 같다
    c = fr.load(save(tmp_path, "c", env, np.arange(30) % 4, np.full(30, 60.0)))
    assert fr.compare(c, b, True, 100)["se"] == fr.compare(c, b, True, 100)["se"] > 0


def test_paired_comparison_refuses_runs_with_different_envs_or_repeated_envs(tmp_path):
    a = fr.load(save(tmp_path, "a", [0, 1, 2], [1, 1, 1], [10, 10, 10]))
    b = fr.load(save(tmp_path, "b", [0, 1, 3], [1, 1, 1], [10, 10, 10]))
    with pytest.raises(ValueError, match="not the same episodes"):
        fr.compare(a, b, paired=True)
    c = fr.load(save(tmp_path, "c", [0, 1, 1], [1, 1, 1], [10, 10, 10]))
    with pytest.raises(ValueError, match="more than one episode per env"):
        fr.compare(c, c, paired=True)
