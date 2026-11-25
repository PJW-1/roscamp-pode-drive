"""좌우 거울 변환 테스트 (Isaac 없이). 실행: python -m pytest learning/goal_reach -q"""

import dataclasses
import itertools
import math
import os
import sys

import pytest
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "store_arena"))
import symmetry as sym  # noqa: E402
import task_spec as ts  # noqa: E402

LIDAR_X = -0.017  # base_link 기준 라이다 위치 (goal_reach_env.make_lidar_cfg), y 0
# 섹터가 아닌 항목: 삼각함수와 좌표 변환의 float32 반올림만 남음
FLOAT_TOL = 1e-5

# 관측 배치를 바꾸는 선택 필드와 시험 값
BLOCK_VALUES = {
    "prev_action_obs": (0, 1),
    "lidar_frames": (1, 3),
    "speed_cap_obs": (0, 1),
    "intent_robots": (0, 2),
    "intent_priority": (0, 1),
    "intent_points": (3, 2),
}


def all_block_specs():
    keys = list(BLOCK_VALUES)
    for values in itertools.product(*BLOCK_VALUES.values()):
        yield ts.make_spec(dict(zip(keys, values, strict=True)))


def full_spec():
    return ts.make_spec(
        dict(prev_action_obs=1, lidar_frames=3, speed_cap_obs=1, intent_robots=2, intent_priority=1)
    )


# ---- 배치: 모든 선택 블록 조합 ----
@pytest.mark.parametrize("spec", list(all_block_specs()), ids=lambda s: ",".join(ts.obs_blocks(s)))
def test_table_covers_layout_and_is_an_involution(spec):
    perm, sign = sym.mirror_table(spec)
    names = ts.obs_names(spec)
    assert len(perm) == ts.obs_dim(spec) == len(names)
    assert [perm[p] for p in perm] == list(range(len(perm)))
    assert all(sign[i] * sign[perm[i]] == 1.0 for i in range(len(perm)))
    # 실제 관측 생성기의 폭과 같음 (build_observation 에 블록이 늘면 여기서 걸림)
    n = 3
    obs = ts.build_observation(
        torch.rand(n, spec.lidar_rays) * 3, torch.rand(n), torch.rand(n), torch.rand(n, 2), spec
    )
    assert obs.shape[-1] == len(perm)
    x = torch.randn(5, len(perm))
    assert torch.equal(sym.mirror_obs(sym.mirror_obs(x, spec), spec), x)
    # 이름으로 본 규칙: 섹터 k <-> 35-k, 옆 방향 항목만 부호 반전
    S = spec.lidar_sectors
    flipped = {"goal_sin", "w", "prev_w"}
    for i, nm in enumerate(names):
        if "sector_" in nm:
            head, k = nm.rsplit("_", 1)
            assert names[perm[i]] == f"{head}_{S - 1 - int(k)}" and sign[i] == 1.0
        else:
            assert names[perm[i]] == nm
            assert (sign[i] == -1.0) == (nm in flipped or nm.endswith("_y")), nm


def test_every_optional_field_switched_on_is_mirrored():
    """OVERRIDABLE 의 정수 필드를 모두 켠다: 거울 규칙 없는 새 블록은 여기서 실패한다."""
    on = {}
    for f in ts.OVERRIDABLE:
        default = getattr(ts.SPEC, f)
        if isinstance(default, int) and not isinstance(default, bool):
            on[f] = 3 if f == "lidar_frames" else max(default, 2)
    spec = ts.make_spec(on)
    assert set(ts.obs_blocks(spec)) == set(sym.KNOWN_BLOCKS)
    perm, _ = sym.mirror_table(spec)
    n = 2
    obs = ts.build_observation(
        torch.rand(n, spec.lidar_rays), torch.rand(n), torch.rand(n), torch.rand(n, 2), spec
    )
    assert obs.shape[-1] == len(perm)


def test_unknown_block_or_entry_stops():
    with pytest.raises(KeyError):
        sym._partner("new_thing", 36)


def test_privileged_and_actions_mirror_twice_is_identity():
    p = torch.randn(4, 5 * 3)
    m = sym.mirror_privileged(p).view(4, 3, 5)
    assert torch.equal(m[..., [0, 1, 3]], p.view(4, 3, 5)[..., [0, 1, 3]])
    assert torch.equal(m[..., [2, 4]], -p.view(4, 3, 5)[..., [2, 4]])
    assert torch.equal(sym.mirror_privileged(sym.mirror_privileged(p)), p)
    a = torch.randn(6, 2)
    assert torch.equal(sym.mirror_actions(a), a * torch.tensor([1.0, -1.0]))
    assert torch.equal(sym.mirror_actions(sym.mirror_actions(a)), a)
    with pytest.raises(ValueError):
        sym.mirror_privileged(torch.zeros(1, 7))


# ---- 기하 증명: 장면을 뒤집은 관측 = 관측을 뒤집은 것 ----
def mirror_rects(r):
    return torch.stack((r[..., 0], -r[..., 3], r[..., 2], -r[..., 1]), -1)


def flip_y(p):
    return p * torch.tensor([1.0, -1.0])


def arena_rects():
    import arena_spec as arena

    rects = arena.boxes_tensor(arena.arena_boxes())
    boxes = torch.tensor([[0.30, 0.60, 0.42, 0.72], [1.10, 0.15, 1.25, 0.25]])
    return torch.cat((rects, boxes))


def free_poses(rects, n, seed):
    """아레나 안, 벽/선반/박스에서 0.08 m 넘게 떨어진 라이다 위치를 갖는 자세 (xy, yaw)."""
    import arena_spec as arena

    g = torch.Generator().manual_seed(seed)
    out_xy, out_yaw = [], []
    while sum(len(x) for x in out_xy) < n:
        xy = torch.rand(4 * n, 2, generator=g) * torch.tensor([1.98, 0.98])
        yaw = (2 * torch.rand(4 * n, generator=g) - 1) * math.pi
        lid = lidar_origin(xy, yaw)
        ok = ~arena.near_wall_starts(lid, rects, 0.08) & ~arena.near_wall_starts(xy, rects, 0.08)
        out_xy.append(xy[ok])
        out_yaw.append(yaw[ok])
    return torch.cat(out_xy)[:n], torch.cat(out_yaw)[:n]


def lidar_origin(xy, yaw):
    return xy + LIDAR_X * torch.stack((torch.cos(yaw), torch.sin(yaw)), -1)


def scan(xy, yaw, rects):
    import arena_spec as arena

    return arena.raycast_rects(lidar_origin(xy, yaw), yaw, rects)


def scene_obs(spec, rects, xy, yaw, hist_pose, goal, vel, prev, cap, intent_w, mask, prio, objs):
    """장면(월드 좌표)에서 관측과 크리틱 특권 항목을 만든다."""
    import arena_spec as arena

    ranges = scan(xy, yaw, rects)
    hist = torch.stack(
        [ts.lidar_sectors(scan(hx, hy, rects), spec) / spec.lidar_obs_max for hx, hy in hist_pose],
        1,
    )
    dist, ang = ts.goal_in_robot_frame(xy, yaw, goal)
    n, R, P = intent_w.shape[:3]
    intent = arena._to_frame(intent_w.reshape(n, R * P, 2), xy, yaw).reshape(n, R, P, 2)
    obs = ts.build_observation(
        ranges, dist, ang, vel, spec, intent=intent, intent_mask=mask, intent_priority=prio,
        prev_target=prev, lidar_history=hist, speed_cap=cap,
    )
    pos, ovel, present = objs
    priv = ts.privileged_block(pos, ovel, present, xy, yaw)
    return ranges, obs, priv


def make_scene(spec, n, seed):
    rects = arena_rects()
    xy, yaw = free_poses(rects, n, seed)
    g = torch.Generator().manual_seed(seed + 1)
    hist_pose = []
    for f in range(1, spec.lidar_frames):  # 조금 뒤, 조금 다른 방향에서 본 이전 프레임
        back = 0.03 * f * torch.stack((torch.cos(yaw), torch.sin(yaw)), -1)
        hist_pose.append((xy - back, yaw - 0.05 * f))
    goal = torch.rand(n, 2, generator=g) * torch.tensor([1.98, 0.98])
    vel = (torch.rand(n, 2, generator=g) * 2 - 1) * torch.tensor([0.3, 1.0])
    prev = (torch.rand(n, 2, generator=g) * 2 - 1) * torch.tensor([0.3, 1.0])
    cap = 0.1 + 0.2 * torch.rand(n, generator=g)
    intent_w = torch.rand(n, spec.intent_robots, spec.intent_points, 2, generator=g) * 2
    mask = torch.rand(n, spec.intent_robots, generator=g) < 0.7
    prio = torch.rand(n, spec.intent_robots, generator=g) < 0.5
    pos = torch.rand(n, 4, 2, generator=g) * 2
    ovel = torch.randn(n, 4, 2, generator=g) * 0.3
    present = torch.rand(n, 4, generator=g) < 0.7
    scene = (rects, xy, yaw, hist_pose, goal, vel, prev, cap, intent_w, mask, prio,
             (pos, ovel, present))
    mirrored = (
        mirror_rects(rects), flip_y(xy), -yaw, [(flip_y(hx), -hy) for hx, hy in hist_pose],
        flip_y(goal), flip_y(vel), flip_y(prev), cap, flip_y(intent_w), mask, prio,
        (flip_y(pos), flip_y(ovel), present),
    )
    return scene, mirrored


def sector_bound(ranges, spec):
    """섹터 k 의 근사 오차 상한 (관측 단위): 거울 창과 섹터 35-k 창이 다른 두 레이의 차.
    정확한 거울 = min(공통 9 레이, 레이 360-10k), 근사 = min(공통, 레이 350-10k),
    |min(A, a) - min(A, b)| <= |a - b|."""
    r = torch.nan_to_num(ranges, nan=spec.lidar_obs_max, posinf=spec.lidar_obs_max)
    r = r.clamp(max=spec.lidar_obs_max) / spec.lidar_obs_max
    width = spec.lidar_rays // spec.lidar_sectors
    k = torch.arange(spec.lidar_sectors)
    a = (spec.lidar_rays - width * k) % spec.lidar_rays
    b = spec.lidar_rays - width - width * k
    return (r[:, a] - r[:, b]).abs()


def sector_flip_error(n=2000, seed=0):
    """실제 아레나 기하에서 섹터 35-k 근사 오차 |obs(M 장면) - M obs(장면)| [m] (현재 섹터 블록)."""
    spec = ts.SPEC
    rects = arena_rects()
    xy, yaw = free_poses(rects, n, seed)
    a = ts.lidar_sectors(scan(xy, yaw, rects), spec)
    b = ts.lidar_sectors(scan(flip_y(xy), -yaw, mirror_rects(rects)), spec)
    return (b - a.flip(-1)).abs(), torch.minimum(b, a.flip(-1))


def test_mirrored_scan_is_the_ray_mirror():
    """거울 장면의 레이 i = 원래 장면의 레이 (360 - i) mod 360 (레이 단위에서는 정확)."""
    spec = ts.SPEC
    rects = arena_rects()
    xy, yaw = free_poses(rects, 256, 3)
    r = scan(xy, yaw, rects)
    rm = scan(flip_y(xy), -yaw, mirror_rects(rects))
    idx = (spec.lidar_rays - torch.arange(spec.lidar_rays)) % spec.lidar_rays
    both_inf = torch.isinf(r[:, idx]) & torch.isinf(rm)
    assert both_inf.equal(torch.isinf(r[:, idx]) | torch.isinf(rm))
    assert torch.allclose(rm[~both_inf], r[:, idx][~both_inf], atol=1e-5, rtol=0)


def test_observation_of_mirrored_scene_is_mirrored_observation():
    spec = full_spec()
    scene, mirrored = make_scene(spec, 512, 0)
    ranges, obs, priv = scene_obs(spec, *scene)
    _, obs_m, priv_m = scene_obs(spec, *mirrored)
    want = sym.mirror_obs(obs, spec)
    names = ts.obs_names(spec)
    cur = [i for i, nm in enumerate(names) if nm.startswith("lidar_sector_")]
    hist = [i for i, nm in enumerate(names) if nm.startswith("lidar_hist")]
    other = [i for i in range(len(names)) if i not in cur and i not in hist]
    # 섹터가 아닌 항목 (목표, 속도, 직전 행동, 상한, intent 존재·우선권·경로점): float 정밀도로 같음
    assert torch.allclose(obs_m[:, other], want[:, other], atol=FLOAT_TOL, rtol=0)
    assert torch.allclose(priv_m, sym.mirror_privileged(priv), atol=FLOAT_TOL, rtol=0)
    # 현재 섹터: 두 가장자리 레이 차이 이내 (레이 하나 어긋남이 강제하는 허용 오차)
    err = (obs_m[:, cur] - want[:, cur]).abs()
    assert (err <= sector_bound(ranges, spec) + FLOAT_TOL).all()
    # 근사가 실제로 틀리는 경우가 있음 (상한이 빈 말이 아님), 중앙값은 mm 수준
    assert err.max() > 0.01
    assert err.median() * spec.lidar_obs_max < 0.005
    # 이력 프레임도 같은 상한 (각 프레임의 자기 레이 기준)
    S = spec.lidar_sectors
    for f, (hx, hy) in enumerate(scene[3]):
        cols = hist[f * S : (f + 1) * S]
        e = (obs_m[:, cols] - want[:, cols]).abs()
        assert (e <= sector_bound(scan(hx, hy, scene[0]), spec) + FLOAT_TOL).all()


def test_exactly_equivariant_policy_survives_the_layout():
    """정책 쪽 검사: 거울 관측에서 거울 행동을 내는 함수는 이 표로 오차 0."""
    spec = full_spec()
    x = torch.randn(64, ts.obs_dim(spec))
    perm, sign = sym.mirror_table(spec)
    names = ts.obs_names(spec)
    w_i, sin_i = names.index("w"), names.index("goal_sin")

    def pi(o):
        v = o[:, names.index("v")] + o[:, 0] + o[:, 35]
        return torch.stack((v, o[:, w_i] - o[:, sin_i]), -1)

    assert torch.allclose(pi(sym.mirror_obs(x, spec)), sym.mirror_actions(pi(x)), atol=1e-6)
    assert dataclasses.is_dataclass(spec)


if __name__ == "__main__":
    e, near = sector_flip_error()
    print(f"poses {e.shape[0]}, sector entries {e.numel()}")
    for label, flat in (("all", e.flatten()), ("sector < 0.25 m", e[near < 0.25])):
        qs = " ".join(f"p{int(q * 100)} {flat.quantile(q):.4f}" for q in (0.5, 0.9, 0.95, 0.99))
        print(
            f"{label}: n {flat.numel()}, exact(<1e-5) {(flat < 1e-5).float().mean():.3f}, "
            f"mean {flat.mean():.4f}, {qs}, max {flat.max():.4f} m"
        )
    print(f"per-pose max over sectors, mean {e.amax(-1).mean():.4f} m")
