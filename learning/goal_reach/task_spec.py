"""시뮬레이터에 의존하지 않는 goal-reach 과제 정의: 관측, 행동, 보상, 성공, 충돌.

배치 텐서(첫 차원 = env)로만 쓰여 Isaac Lab env 와 2D 시뮬이 같은 정의를 공유한다.
좌표: 바닥 x/y, yaw 는 +x 에서 반시계. 라이다 레이 i 는 base 프레임 (-180 + i) deg (180 = 정면).
"""

from __future__ import annotations

import dataclasses
import math
import os
from dataclasses import dataclass

import torch
import yaml

# 모든 Pode 한도의 최고속도: Nav2 패키지 파일
NAV_SPEED_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "pode", "pode_nav_config",
    "params", "nav_speed.yaml"
)
# 에피소드 길이와 이동 시간 기반 대기는 이 속도 기준으로 맞춤
TIMES_TUNED_AT_MPS = 0.25


def nav_speed_mps(path: str = NAV_SPEED_FILE) -> float:
    with open(path, encoding="utf-8") as f:
        value = float(yaml.safe_load(f)["nav_speed_mps"])
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"{path}: nav_speed_mps must be above 0, got {value!r}")
    return value


NAV_SPEED_MPS = nav_speed_mps()


def travel_time(seconds_at_tuned_speed: float, speed: float = NAV_SPEED_MPS) -> float:
    """맞춘 속도 기준 시간을 현재 속도에서 같은 거리를 가도록 늘린다."""
    return seconds_at_tuned_speed * TIMES_TUNED_AT_MPS / speed


@dataclass(frozen=True)
class ActionLimits:
    """명령 한도 한 곳. env, 행동 매핑, 2D 시뮬이 함께 읽는다."""

    v_min: float = (
        -0.05
    )  # m/s; 소폭 후진 (실로봇 Nav2 BackUp 속도)
    v_max: float = NAV_SPEED_MPS  # m/s; pode_nav_config/params/nav_speed.yaml
    w_max: float = 1.0  # rad/s
    acc_v: float = 2.0  # m/s^2
    acc_w: float = 2.0  # rad/s^2


@dataclass(frozen=True)
class RobotGeometry:
    wheel_radius: float = 0.028  # m
    wheel_separation: float = 0.0961  # m, 실로봇 bringup 과 Gazebo 값


@dataclass(frozen=True)
class TaskSpec:
    control_dt: float = 0.05  # s, 20 Hz 정책
    episode_s: float = travel_time(20.0)  # 0.25 m/s 로 20 s 거리
    goal_dist_range: tuple[float, float] = (1.0, 2.0)  # 시작점에서 m
    success_radius: float = 0.05  # m; 실로봇 Nav2 xy_goal_tolerance 와 같음
    out_of_bounds: float = (
        3.0  # 시작점에서 m: 넘으면 실패 (로봇을 자기 칸에 가둠)
    )
    # ---- 라이다 ----
    lidar_rays: int = 360
    lidar_sectors: int = 36  # 10 deg 섹터별 최솟값
    lidar_min_range: float = 0.05  # m; 레이 시작 거리 (RPLIDAR C1 최소와 같음)
    lidar_obs_max: float = 2.0  # m; [0, 1] 로 스케일하기 전 클리핑
    # 관측 거리 = s × 실제 거리, s 는 에피소드마다 이 범위에서 균등 (로봇별 라이다 배율 오차)
    lidar_scale_range: tuple[float, float] = (1.0, 1.0)
    # 몸에 붙은 것(트레이 기둥)의 반환: (lo deg, hi deg, 거리 m) 묶음. base 기준 각이 그 범위인
    # 관측 레이는 min(레이, 거리) 를 읽는다. 빈 값이면 없음 (평가용: 실물 로봇의 scan 을 흉내 냄)
    lidar_posts: tuple[float, ...] = ()
    # 가릴 각도: (lo deg, hi deg) 묶음. 그 범위의 관측 레이는 반환 없음 (lidar_obs_max).
    # 플러그인의 scan_mask_deg 와 같은 뜻이고 lidar_posts 뒤에 적용한다
    lidar_mask_deg: tuple[float, ...] = ()
    # 1 이면 가림에 걸렸고 가리지 않은 레이의 반환이 없는 섹터를 이웃 섹터 값으로 채운다
    lidar_mask_fill: int = 0
    # ---- 명령 응답 모델: 목표 -> 이득 -> 지연 -> 1차 지연 -> 변화율 제한 -> 바퀴 ----
    # 에피소드마다 env 별로 균등하게 뽑음 (한 값이면 뽑지 않음). 기본값은 응답 모델 없음
    cmd_delay_steps_range: tuple[int, int] = (0, 0)  # 제어 스텝 수, 양끝 포함 정수
    cmd_lag_tau_range: tuple[float, float] = (0.0, 0.0)  # s; 1차 지연 시간 상수 (0 = 지연 없음)
    cmd_gain_v_range: tuple[float, float] = (1.0, 1.0)  # 정상 상태 v 비 (실제 / 명령)
    cmd_gain_w_range: tuple[float, float] = (1.0, 1.0)  # 정상 상태 w 비 (실제 / 명령)
    goal_obs_max: float = 3.0  # m; 목표 거리 클리핑
    # ---- 플릿 intent (4c): 가까운 intent_robots 대마다 존재 마스크와 경로점 intent_points 개 ----
    # 0 대 = goal-reach, 4a, 4b 정책의 41차원 배치
    intent_robots: int = 0
    intent_points: int = 3
    intent_spacing_m: float = 0.2  # 경로점 간격 [m]
    intent_obs_max: float = 2.0  # m, 라이다와 같음
    # 1: 존재 플래그 뒤에 우선권 플래그 추가 (그 로봇이 우리보다 우선)
    intent_priority: int = 0
    # ---- 선택 관측 블록: 기본 41개와 intent 블록 사이에 이 순서로 (모두 끄면 ppo_k 배치) ----
    # prev_action_obs 1: 직전 제어 스텝의 자른 목표 (v, w) / (v_max, w_max), 에피소드 시작에는 0
    prev_action_obs: int = 0
    # lidar_frames > 1: 이전 섹터 벡터 frames - 1 개 (같은 배율, 최신 먼저, lidar_stride_steps 간격)
    lidar_frames: int = 1
    lidar_stride_steps: int = 4
    # speed_cap_obs 1: 에피소드 최고 선속도 / v_max. 상한은 speed_cap_range 에서 에피소드마다 뽑음
    # v 행동 상한, 에피소드 시간, 스크립트 로봇 속도가 상한을 따름. v 관측은 그대로 / v_max
    speed_cap_obs: int = 0
    speed_cap_range: tuple[float, float] = (NAV_SPEED_MPS, NAV_SPEED_MPS)  # m/s
    # critic_privileged 1: 크리틱(액터 아님)이 움직이는 물체와 스크립트 로봇의 실제 위치, 속도도
    # 받음
    critic_privileged: int = 0
    # ---- 충돌: COLLISION_BODIES 의 최대 접촉력이 이를 넘으면 종료 ----
    collision_force: float = 1.0  # N; 로봇 무게 15 N, 벽 밀기 5-11 N
    # ---- 보상 가중치 (20 Hz 스텝당) ----
    w_progress: float = 1.0  # (d_prev - d_now) / (v_max * dt) 배: 최고속도에서 스텝당 +1
    w_time: float = -0.05  # 매 스텝
    w_arrival: float = 10.0  # 성공 시 한 번
    w_collision: float = -10.0  # 충돌 또는 범위 이탈 시 한 번
    # 부드러움: 정규화한 (v, w) 목표의 스텝당 변화 제곱, w 목표 부호 반전마다 곱함
    w_action_rate: float = 0.0
    w_w_flip: float = 0.0
    flip_deadband: float = 0.02  # rad/s; 부호 반전을 셀 때 |w| 가 이보다 작으면 건너뜀


# 접촉하면 충돌로 치는 몸체 (바퀴와 캐스터는 바닥에 닿으므로 제외)
COLLISION_BODIES = ("base_link", "screen_mount", "front_camera_mount", "rplidar_mount")

ACTION_LIMITS = ActionLimits()
GEOMETRY = RobotGeometry()
SPEC = TaskSpec()

# 실험에서 바꿀 수 있는 TaskSpec 필드 (train.py --set), 나머지는 import 때 한 번 읽음
OVERRIDABLE = (
    "goal_dist_range",
    "success_radius",
    "out_of_bounds",
    "lidar_scale_range",
    "lidar_posts",
    "lidar_mask_deg",
    "lidar_mask_fill",
    "cmd_delay_steps_range",
    "cmd_lag_tau_range",
    "cmd_gain_v_range",
    "cmd_gain_w_range",
    "goal_obs_max",
    "collision_force",
    "w_progress",
    "w_time",
    "w_arrival",
    "w_collision",
    # 관측 크기가 바뀜: 정책은 학습한 배치에서만 동작
    "intent_robots",
    "intent_points",
    "intent_spacing_m",
    "intent_obs_max",
    "intent_priority",
    "prev_action_obs",
    "lidar_frames",
    "lidar_stride_steps",
    "speed_cap_obs",
    "speed_cap_range",
    "critic_privileged",
    "w_action_rate",
    "w_w_flip",
    "flip_deadband",
)


# 길이가 정해지지 않은 튜플 필드와 한 묶음의 크기 (값은 묶음 수만큼 이어 적음)
GROUPED = {"lidar_posts": 3, "lidar_mask_deg": 2}
# 채울 값을 찾아 한쪽으로 보는 섹터 수 (lidar_mask_fill)
FILL_REACH = 4


def parse_overrides(items, spec=SPEC) -> dict:
    """--set 의 FIELD=VALUE 문자열을 spec 필드 기본값 타입으로 변환 (튜플은 a,b)."""
    out = {}
    for item in items:
        key, text = item.split("=", 1)
        default = getattr(spec, key, None)
        if key in GROUPED:
            # 빈 항목을 건너뛰지 않는다: 141,,146 은 틀린 입력이다
            parts = text.split(",")
            try:
                values = tuple(float(x) for x in parts)
            except ValueError:
                values = ()
            if not values or len(values) % GROUPED[key]:
                raise ValueError(
                    f"{key} takes groups of {GROUPED[key]} comma-separated numbers, got {text!r}"
                )
            out[key] = values
        elif isinstance(default, str):
            out[key] = text
        elif isinstance(default, int) and not isinstance(default, bool):
            out[key] = int(float(text))
        elif isinstance(default, tuple):
            parts = text.split(",")
            if len(parts) != len(default):
                raise ValueError(
                    f"{key} takes {len(default)} comma-separated numbers, got {text!r}"
                )
            values = tuple(float(x) for x in parts)
            if any(
                isinstance(d, int) and not v.is_integer()
                for d, v in zip(default, values, strict=True)
            ):
                raise ValueError(f"{key} takes integers, got {text!r}")
            out[key] = tuple(
                int(v) if isinstance(d, int) else v for d, v in zip(default, values, strict=True)
            )
        else:
            out[key] = float(text)
    return out


def split_dones(outcome_done: torch.Tensor, length: torch.Tensor, max_length: int):
    """(terminated, time_out). 마지막 스텝의 성공/충돌/이탈은 terminated 만 (PPO 가 time_out 에서 가치를 부트스트랩)."""
    time_out = (length >= max_length) & ~outcome_done
    return outcome_done, time_out


def make_spec(overrides: dict) -> TaskSpec:
    bad = sorted(set(overrides) - set(OVERRIDABLE))
    if bad:
        raise ValueError(f"TaskSpec fields {bad} cannot be overridden; allowed: {OVERRIDABLE}")
    spec = dataclasses.replace(SPEC, **overrides)
    own_body_groups(spec)  # 틀린 묶음은 여기서 거절
    return spec


OBS_LIDAR = SPEC.lidar_sectors


BASE_DIM = OBS_LIDAR + 3 + 2  # 라이다 섹터, 목표 (거리, sin, cos), (v, w): ppo_k 배치


def obs_blocks(spec: TaskSpec = SPEC) -> list[str]:
    """기본 항목 뒤에 있는 선택 블록 이름 (정해진 순서)."""
    out = []
    if spec.prev_action_obs:
        out.append("prev_action")
    if spec.lidar_frames > 1:
        out.append("lidar_history")
    if spec.speed_cap_obs:
        out.append("speed_cap")
    if spec.intent_robots:
        out.append("intent")
    return out


def obs_dim(spec: TaskSpec = SPEC) -> int:
    """기본 항목, 이어서 prev_action (2), lidar_history (섹터 x (frames - 1)), speed_cap (1),
    intent 로봇마다 마스크, 우선순위 플래그(선택), 경로점."""
    per = 1 + int(spec.intent_priority > 0) + 2 * spec.intent_points
    return (
        BASE_DIM
        + 2 * int(spec.prev_action_obs > 0)
        + spec.lidar_sectors * max(spec.lidar_frames - 1, 0)
        + int(spec.speed_cap_obs > 0)
        + spec.intent_robots * per
    )


def obs_names(spec: TaskSpec = SPEC) -> list[str]:
    """관측 항목 이름 순서 (정책 내보내기 policy_spec.json 용)."""
    names = [f"lidar_sector_{i}" for i in range(spec.lidar_sectors)]
    names += ["goal_dist", "goal_sin", "goal_cos", "v", "w"]
    if spec.prev_action_obs:
        names += ["prev_v", "prev_w"]
    for f in range(1, spec.lidar_frames):
        names += [f"lidar_hist{f}_sector_{i}" for i in range(spec.lidar_sectors)]
    if spec.speed_cap_obs:
        names.append("speed_cap")
    for r in range(spec.intent_robots):
        names.append(f"robot{r}_present")
        if spec.intent_priority:
            names.append(f"robot{r}_priority")
        for k in range(spec.intent_points):
            names += [f"robot{r}_p{k + 1}_x", f"robot{r}_p{k + 1}_y"]
    return names


OBS_DIM = obs_dim(SPEC)  # 41
ACT_DIM = 2


def action_to_target(
    actions: torch.Tensor, lim: ActionLimits = ACTION_LIMITS, v_cap: torch.Tensor | None = None
) -> torch.Tensor:
    """정책 행동 [-1, 1]^2 를 목표 (v, w) 로. v 는 [v_min, v_max] 로 선형 변환 (env 별 v_cap (N,) 이
    있으면 [v_min, v_cap])."""
    a = actions.clamp(-1.0, 1.0)
    top = lim.v_max if v_cap is None else v_cap
    v = lim.v_min + (a[:, 0] + 1.0) * 0.5 * (top - lim.v_min)
    w = a[:, 1] * lim.w_max
    return torch.stack((v, w), dim=-1)


def still_action(n: int, device=None, lim: ActionLimits = ACTION_LIMITS, v_cap=None):
    """목표가 (0, 0) 인 (n, 2) 행동: 로봇이 서 있음 (item 7 상한 실행용)."""
    top = lim.v_max if v_cap is None else v_cap
    a0 = 2.0 * (0.0 - lim.v_min) / (top - lim.v_min) - 1.0
    out = torch.zeros(n, 2, device=device)
    out[:, 0] = a0
    return out


def capped_action(actions: torch.Tensor, v_cap: torch.Tensor, lim: ActionLimits = ACTION_LIMITS):
    """speed_cap_obs 없이 학습한 정책의 행동을 v 를 [v_min, v_cap] 로 맞추는 env 용으로 바꿈.
    목표 v 는 lim.v_max 기준 값을 v_cap 으로 자른 것 (낮은 최고속도에서 플러그인이 형식 2 정책에
    하는 일)."""
    a = actions.clamp(-1.0, 1.0)
    v = lim.v_min + (a[:, 0] + 1.0) * 0.5 * (lim.v_max - lim.v_min)
    v = torch.minimum(v, v_cap)
    out = a.clone()
    out[:, 0] = 2.0 * (v - lim.v_min) / (v_cap - lim.v_min) - 1.0
    return out


def parse_front_stop(text: str) -> tuple[float, float]:
    """play.py --front_stop 의 "거리 m,반각 deg": 거리는 유한한 양수, 반각은 0 초과 180 이하.
    아니면 ValueError (nan 은 앞 막기를 조용히 끄고 inf 는 늘 막으므로 받지 않는다)."""
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError(f"front stop wants RANGE_M,HALF_DEG, got {text!r}")
    stop_range, half_width_deg = float(parts[0]), float(parts[1])
    if not (math.isfinite(stop_range) and stop_range > 0.0 and 0.0 < half_width_deg <= 180.0):
        raise ValueError(f"front stop wants a finite range > 0 and 0 < half width <= 180: {text!r}")
    return stop_range, half_width_deg


def block_forward(
    target: torch.Tensor, rays: torch.Tensor, stop_range: float, half_width_deg: float
) -> tuple[torch.Tensor, torch.Tensor]:
    """앞 막기: 정면 +-half_width_deg 안의 레이 가운데 stop_range 보다 가까운 것이 있으면
    전진 목표 v 를 0 으로 (w 는 그대로). 플러그인 block_reverse 의 앞쪽 짝이고 평가에서만 쓴다
    (play.py --front_stop). rays (N, R) 는 라이다 중심에서의 거리, -180 deg 부터 고른 간격.
    반환: (목표 (N, 2), 막았는지 (N,))."""
    n_rays = rays.shape[1]
    deg = -180.0 + torch.arange(n_rays, device=rays.device) * (360.0 / n_rays)
    front = deg.abs() <= half_width_deg + 1e-9
    near = (rays[:, front] < stop_range).any(dim=-1)
    blocked = near & (target[:, 0] > 0.0) & (stop_range > 0.0)
    out = target.clone()
    out[:, 0] = torch.where(blocked, torch.zeros_like(out[:, 0]), out[:, 0])
    return out, blocked


# ---- 움직이는 것 앞에서만 서는 규칙 (평가 전용) ----
MOVING_MIN_MPS = 0.02  # m/s; 이보다 느린 mover 는 서 있는 것으로 본다
CLOSING_LOOKBACK_S = 0.2  # s; 지금 스캔과 견주는 옛 스캔까지의 시간
CLOSING_SPREAD_DEG = 8.0  # deg; 옛 스캔에서 최솟값을 찾는 이웃의 반각
CLOSING_MIN_MPS = 0.04  # m/s; 로봇 자신의 이동을 뺀 다가옴이 이보다 빠르면 건다
CLOSING_HOLD_S = 0.3  # s; 마지막으로 걸린 뒤 유지하는 시간
RAY_CAP_M = 100.0  # m; 닿지 않은 레이를 유한한 거리로
CLOSING_TUNE = ("lookback_s", "spread_deg", "closing_mps", "hold_s", "noise_m", "min_rays")


def ray_angles(n_rays: int, device=None) -> torch.Tensor:
    """레이 i 의 base 프레임 방위 (rad): -180 deg 부터 고른 간격."""
    return torch.deg2rad(-180.0 + torch.arange(n_rays, device=device) * (360.0 / n_rays))


def block_for_movers(
    target: torch.Tensor,
    xy: torch.Tensor,
    yaw: torch.Tensor,
    movers: list,
    stop_range: float,
    half_width_deg: float,
    min_speed: float = MOVING_MIN_MPS,
) -> tuple[torch.Tensor, torch.Tensor]:
    """사람 위치를 아는 막기 (play.py --person_stop): min_speed 이상으로 움직이는 활성 mover 의
    중심이 로봇 중심에서 stop_range 안, 정면 +-half_width_deg 안에 있으면 전진 목표 v 를 0 으로
    (w 는 그대로). 시뮬만 아는 정보라 실물에는 없다: 움직이는 것을 완벽히 가려낼 때의 위쪽
    한계를 잰다.
    movers 는 (위치 (N, 2), 속도 (N, 2), 활성 (N,)) 의 목록. 반환: (목표 (N, 2), 막았는지 (N,))."""
    near = torch.zeros(len(target), dtype=torch.bool, device=target.device)
    half = math.radians(half_width_deg)
    for pos, vel, active in movers:
        rel = pos - xy
        bearing = torch.atan2(rel[:, 1], rel[:, 0]) - yaw
        bearing = torch.atan2(torch.sin(bearing), torch.cos(bearing))  # -pi..pi
        moving = active & (vel.norm(dim=-1) >= min_speed)
        near |= moving & (rel.norm(dim=-1) < stop_range) & (bearing.abs() <= half)
    blocked = near & (target[:, 0] > 0.0)
    out = target.clone()
    out[:, 0] = torch.where(blocked, torch.zeros_like(out[:, 0]), out[:, 0])
    return out, blocked


def parse_closing_tune(text: str) -> dict:
    """play.py --closing_tune 의 "이름=값,..." 을 ClosingGuard 의 인자로 (빈 글이면 {}).
    이름은 CLOSING_TUNE 의 것만, 값은 유한한 수: lookback_s 와 closing_mps 는 양수, spread_deg,
    hold_s, noise_m 은 0 이상, min_rays 는 1 이상의 정수. 아니면 ValueError."""
    out = {}
    for part in text.split(",") if text else ():
        name, sep, value = part.partition("=")
        if not sep or name not in CLOSING_TUNE or name in out:
            raise ValueError(f"closing tune wants NAME=VALUE with NAME in {CLOSING_TUNE}: {part!r}")
        num = float(value)
        positive = name in ("lookback_s", "closing_mps")
        if not math.isfinite(num) or num < 0.0 or (positive and num == 0.0):
            raise ValueError(f"closing tune: {name} out of range: {part!r}")
        if name == "min_rays":
            if num < 1.0 or num != int(num):
                raise ValueError(f"closing tune: min_rays wants an integer >= 1: {part!r}")
            num = int(num)
        out[name] = num
    return out


def static_ranges(
    rays: torch.Tensor,
    origin: torch.Tensor,
    yaw: torch.Tensor,
    origin_now: torch.Tensor,
    yaw_now: torch.Tensor,
    spread_bins: int,
) -> torch.Tensor:
    """옛 스캔의 점들이 서 있었다면 지금 레이마다 보일 가장 가까운 거리 (N, R).
    옛 스캔 rays (N, R) 을 그때의 라이다 원점 origin (N, 2) 과 yaw (N,) 로 점으로 펴고, 지금의
    원점과 yaw 에서 본 방위 칸에 넣어 칸마다 이웃 +-spread_bins 칸의 최솟값을 낸다 (이웃은 한 바퀴를
    넘어 이어진다). 이웃에 옛 점이 없는 칸은 inf."""
    n_rays = rays.shape[1]
    world = yaw[:, None] + ray_angles(n_rays, rays.device)[None, :]
    px = origin[:, 0:1] + rays * torch.cos(world) - origin_now[:, 0:1]
    py = origin[:, 1:2] + rays * torch.sin(world) - origin_now[:, 1:2]
    bearing = torch.atan2(py, px) - yaw_now[:, None]
    bins = torch.round((bearing + math.pi) * (n_rays / (2.0 * math.pi))).long() % n_rays
    nearest = torch.full_like(rays, float("inf")).scatter_reduce(
        1, bins, torch.hypot(px, py), reduce="amin", include_self=True
    )
    if spread_bins > 0:
        ring = torch.cat((nearest[:, -spread_bins:], nearest, nearest[:, :spread_bins]), dim=1)
        pooled = torch.nn.functional.max_pool1d(-ring[:, None, :], 2 * spread_bins + 1, stride=1)
        nearest = -pooled[:, 0, :]
    return nearest


class ClosingGuard:
    """다가옴 막기 (play.py --closing_stop): 라이다와 자기 자세만으로 "다가오는 것" 앞에서 선다.
    lookback_s 전의 스캔을 지금 자세로 옮겨 (static_ranges) 레이마다 "로봇 자신의 이동으로 설명되는
    것보다 얼마나 빨리 가까워졌는가" 를 내고, 정면 +-half_width_deg 안에서 stop_range 보다 가깝고
    그 값이 closing_mps 를 넘는 레이가 있으면 건다. 마지막으로 걸린 뒤 hold_s 동안 유지한다. 걸린
    동안 전진 목표 v 를 0 으로 (w 는 그대로). 서 있는 것은 로봇이 다가가도 걸리지 않는다.
    걸리는 여유는 closing_mps * lookback_s (m) 다: 창이 길면 옛 스캔에서 비어 있던 자리에 들어온
    것 (시선을 가로지르는 것) 도 잡는다. min_rays 는 걸리려면 나란히 있어야 하는 레이 수,
    noise_m 은 이 막기가 보는 스캔에만 더하는 거리 잡음의 표준편차 (실물 라이다 흉내) 다.
    env 별 상태 (옛 스캔 고리, 유지 스텝) 를 갖고, 리셋한 env 는 reset() 으로 비운다."""

    def __init__(
        self,
        step_dt: float,
        stop_range: float,
        half_width_deg: float,
        *,
        lidar_x: float = 0.0,
        lookback_s: float = CLOSING_LOOKBACK_S,
        spread_deg: float = CLOSING_SPREAD_DEG,
        closing_mps: float = CLOSING_MIN_MPS,
        hold_s: float = CLOSING_HOLD_S,
        noise_m: float = 0.0,
        min_rays: int = 1,
    ):
        self.step_dt, self.stop_range, self.half_width_deg = step_dt, stop_range, half_width_deg
        self.lidar_x, self.spread_deg, self.closing_mps = lidar_x, spread_deg, closing_mps
        self.noise_m, self.min_rays = noise_m, min_rays
        self.back = max(1, round(lookback_s / step_dt))  # 옛 스캔까지의 스텝 수
        self.hold_steps = max(0, round(hold_s / step_dt))
        self.scans = None  # 첫 스텝에 스캔의 크기로 잡는다

    def _allocate(self, rays: torch.Tensor) -> None:
        n, n_rays = rays.shape
        # 이웃 창이나 나란한 레이 수가 스캔보다 넓으면 판정할 수 없다
        if 2 * round(self.spread_deg * n_rays / 360.0) + 1 > n_rays or self.min_rays > n_rays:
            raise ValueError(
                f"closing stop: spread_deg {self.spread_deg} or min_rays {self.min_rays} is wider "
                f"than a scan of {n_rays} rays"
            )
        dev, slots = rays.device, self.back + 1
        self.scans = torch.zeros(slots, n, n_rays, device=dev)
        self.origins = torch.zeros(slots, n, 2, device=dev)
        self.yaws = torch.zeros(slots, n, device=dev)
        self.head = 0  # 이번 스캔을 적을 자리
        # 리셋 뒤에 적은 스캔 수 - 1: 리셋 직후의 스캔은 기준으로 쓰지 않는다
        self.filled = torch.full((n,), -1, dtype=torch.long, device=dev)
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)  # 남은 유지 스텝
        self.hit = torch.zeros(n, dtype=torch.bool, device=dev)  # 이번 스텝에 걸렸는지 (유지 제외)
        deg = torch.rad2deg(ray_angles(n_rays, dev))
        self.front = deg.abs() <= self.half_width_deg + 1e-4
        self.spread_bins = round(self.spread_deg * n_rays / 360.0)
        # 잡음은 자기 난수로 뽑는다: 전역 난수 (env 의 리셋과 mover) 의 순서를 바꾸지 않게
        self.noise_rng = torch.Generator(device=dev)
        self.noise_rng.manual_seed(0)

    def reset(self, env_ids) -> None:
        if self.scans is not None:
            self.filled[env_ids] = -1
            self.hold[env_ids] = 0

    def step(
        self, target: torch.Tensor, rays: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """rays (N, R) 는 라이다 중심에서의 거리, xy 와 yaw 는 base 의 자세.
        반환: (목표 (N, 2), 막았는지 (N,))."""
        if self.scans is None:
            self._allocate(rays)
        # 돌아오지 않은 레이 (nan, +-inf) 는 먼 것으로: 옛 스캔의 이웃 칸을 흐리지 않게
        rays = torch.nan_to_num(rays, nan=RAY_CAP_M, posinf=RAY_CAP_M, neginf=RAY_CAP_M)
        rays = rays.clamp(max=RAY_CAP_M)
        if self.noise_m > 0.0:
            noise = torch.randn(rays.shape, generator=self.noise_rng, device=rays.device)
            rays = rays + self.noise_m * noise
        origin = xy + self.lidar_x * torch.stack((torch.cos(yaw), torch.sin(yaw)), dim=-1)
        slot = self.head
        self.scans[slot], self.origins[slot], self.yaws[slot] = rays, origin, yaw
        self.filled += 1
        old = (slot + 1) % (self.back + 1)  # back 스텝 전에 적은 자리 (다음 스텝에 덮어쓴다)
        self.head = old
        static = static_ranges(
            self.scans[old], self.origins[old], self.yaws[old], origin, yaw, self.spread_bins
        )
        closing = (static - rays) / (self.back * self.step_dt)
        closing = torch.where(torch.isfinite(closing), closing, torch.zeros_like(closing))
        near = self.front[None, :] & (rays < self.stop_range) & (closing > self.closing_mps)
        if self.min_rays > 1:  # 나란한 min_rays 개가 모두 걸린 곳만 남긴다
            lone = torch.nn.functional.max_pool1d(1.0 - near[:, None, :].float(), self.min_rays, 1)
            near = lone[:, 0, :] < 0.5
        self.hit = near.any(dim=-1) & (self.filled > self.back)
        fresh_hold = torch.full_like(self.hold, self.hold_steps + 1)
        self.hold = torch.where(self.hit, fresh_hold, self.hold)
        blocked = (self.hold > 0) & (target[:, 0] > 0.0)
        self.hold = (self.hold - 1).clamp(min=0)
        out = target.clone()
        out[:, 0] = torch.where(blocked, torch.zeros_like(out[:, 0]), out[:, 0])
        return out, blocked


def sample_speed_cap(n: int, device, spec: TaskSpec = SPEC) -> torch.Tensor:
    """에피소드별 최고속도 (n,), speed_cap_range 균등 (한 값이면 뽑지 않음)."""
    lo, hi = spec.speed_cap_range
    if lo == hi:
        return torch.full((n,), float(lo), device=device)
    return lo + (hi - lo) * torch.rand(n, device=device)


def sample_lidar_scale(n: int, device, spec: TaskSpec = SPEC) -> torch.Tensor:
    """에피소드별 라이다 거리 배율 (n,), lidar_scale_range 균등 (한 값이면 뽑지 않음)."""
    lo, hi = spec.lidar_scale_range
    if lo == hi:
        return torch.full((n,), float(lo), device=device)
    return lo + (hi - lo) * torch.rand(n, device=device)


def scale_ranges(ranges: torch.Tensor, scale: torch.Tensor) -> torch.Tensor:
    """(N, rays) 거리에 env 별 배율 (N,) 을 곱한다. 섹터 최솟값·자르기 전에 부름 (관측 전용)."""
    return ranges * scale[:, None]


def own_body_groups(spec: TaskSpec = SPEC) -> tuple[list, list]:
    """(lidar_posts 의 (lo, hi, 거리) 목록, lidar_mask_deg 의 (lo, hi) 목록).
    각도는 -180 <= lo <= hi <= 180 (가림은 플러그인의 scan_mask_deg 처럼 lo < hi),
    거리는 0 보다 크고 유한해야 한다. 아니면 ValueError."""
    out = []
    for name, size in GROUPED.items():
        flat = tuple(getattr(spec, name))
        if len(flat) % size:
            raise ValueError(f"{name} takes groups of {size} numbers, got {len(flat)} values")
        groups = [flat[i : i + size] for i in range(0, len(flat), size)]
        for g in groups:
            if not -180.0 <= g[0] <= g[1] <= 180.0:
                raise ValueError(f"{name}: a range must be -180 <= lo <= hi <= 180, got {g[:2]}")
            if size == 2 and not g[0] < g[1]:
                raise ValueError(f"{name}: a range must have lo < hi, got {g}")
            if size == 3 and not (g[2] > 0.0 and math.isfinite(g[2])):
                raise ValueError(f"{name}: the distance must be finite and above 0, got {g[2]}")
        out.append(groups)
    return out[0], out[1]


def rays_within(n_rays: int, lo: float, hi: float, device=None) -> torch.Tensor:
    """-180 deg 부터 같은 간격인 레이 가운데 [lo, hi] deg 안의 것 (n_rays,) bool.
    바로 뒤 레이 (-180) 는 180 으로도 본다."""
    deg = -180.0 + torch.arange(n_rays, device=device) * (360.0 / n_rays)
    back = deg + 360.0
    tol = 1e-6
    front = (deg >= lo - tol) & (deg <= hi + tol)
    return front | ((back >= lo - tol) & (back <= hi + tol))


def own_body_rays(ranges: torch.Tensor, spec: TaskSpec = SPEC) -> torch.Tensor:
    """관측용 (N, rays) 거리에 몸에 붙은 것의 반환 (lidar_posts) 을 넣고, 가릴 각도
    (lidar_mask_deg) 를 반환 없음으로 바꾼다. 레이는 -180 deg 부터 같은 간격이다. 둘 다 비면 받은
    텐서를 그대로 돌려준다 (배율을 곱한 뒤, 섹터 최솟값·자르기 전에 부름. 관측 전용).
    lidar_mask_fill 이면 가린 레이를 NaN 으로 표시한다 (lidar_sectors 가 채울 섹터를 가린다)."""
    posts, masks = own_body_groups(spec)
    if not posts and not masks:
        return ranges
    n_rays = ranges.shape[1]
    out = ranges.clone()
    for lo, hi, dist in posts:
        hit = rays_within(n_rays, lo, hi, ranges.device)
        inf = float("inf")
        seen = torch.nan_to_num(out[:, hit], nan=inf, posinf=inf, neginf=-inf)
        out[:, hit] = seen.clamp(max=dist)
    gone = float("nan") if spec.lidar_mask_fill else spec.lidar_obs_max
    for lo, hi in masks:
        out[:, rays_within(n_rays, lo, hi, ranges.device)] = gone
    return out


def masked_sectors(spec: TaskSpec, n_rays: int, device=None) -> torch.Tensor:
    """가림 (lidar_mask_deg) 에 걸린 레이가 하나라도 있는 섹터 (sectors,) bool."""
    _, masks = own_body_groups(spec)
    hit = torch.zeros(n_rays, dtype=torch.bool, device=device)
    for lo, hi in masks:
        hit |= rays_within(n_rays, lo, hi, device)
    return hit.view(spec.lidar_sectors, -1).any(dim=-1)


def fill_masked_sectors(
    sec: torch.Tensor,
    usable: torch.Tensor,
    touched: torch.Tensor,
    obs_max: float,
    reach: int = FILL_REACH,
) -> torch.Tensor:
    """가림에 걸렸고 쓸 수 있는 레이가 없는 섹터를, 양쪽으로 가장 가까운 '그렇지 않은 섹터' 값의
    작은 쪽으로 채운다. sec (N, S) 섹터 값, usable (N, S) 가리지 않았고 반환이 있는 레이가 있는지,
    touched (S,) 가림에 걸린 섹터. 한쪽에서 reach 섹터 안에 없으면 그쪽은 빼고, 양쪽 다 없으면
    obs_max 다. 나머지 섹터는 그대로다. 채울 섹터가 없으면 받은 텐서를 그대로 돌려준다."""
    empty = touched[None, :] & ~usable
    if not bool(empty.any()):
        return sec
    out = sec.clone()
    n_sec = sec.shape[1]
    for s in torch.nonzero(touched).flatten().tolist():
        best = sec.new_full((sec.shape[0],), obs_max)
        for step in (-1, 1):
            found = torch.zeros_like(empty[:, 0])
            for d in range(1, reach + 1):
                j = (s + step * d) % n_sec
                take = ~found & ~empty[:, j]
                best = torch.where(take, torch.minimum(best, sec[:, j]), best)
                found = found | take
        out[:, s] = torch.where(empty[:, s], best, sec[:, s])
    return out


def flip_step(w: torch.Tensor, last_sign: torch.Tensor, deadband: float):
    """모든 env 에 tools/nav_bench sign_changes 한 스텝: (반전 여부 (N,), 새 last_sign).
    |w| < deadband 는 건너뜀, last_sign 0 은 에피소드에서 아직 부호를 못 봄."""
    s = torch.sign(w) * (w.abs() >= deadband)
    flipped = (s != 0) & (last_sign != 0) & (s != last_sign)
    return flipped, torch.where(s != 0, s, last_sign)


def action_rate_terms(
    target: torch.Tensor,
    prev_target: torch.Tensor,
    flipped: torch.Tensor,
    spec: TaskSpec = SPEC,
    lim: ActionLimits = ACTION_LIMITS,
) -> dict[str, torch.Tensor]:
    """부드러움 보상 항: w_action_rate x (v / v_max, w / w_max) 변화 제곱, w 목표 반전마다
    w_w_flip."""
    scale = torch.tensor((lim.v_max, lim.w_max), device=target.device)
    d = ((target - prev_target) / scale).square().sum(-1)
    return {"action_rate": spec.w_action_rate * d, "w_flip": spec.w_w_flip * flipped.float()}


PRIV_POS_M = 2.0  # m; 특권 위치를 이 값으로 나눔 (라이다 범위와 같음)
PRIV_VEL_MPS = 0.5  # m/s; 특권 속도를 이 값으로 나눔
PRIV_PER_OBJECT = 5  # 존재, x, y, vx, vy


def privileged_block(
    pos: torch.Tensor, vel: torch.Tensor, present: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor
) -> torch.Tensor:
    """크리틱 전용 항목 (critic_privileged): 물체마다 (움직이는 물체, 이어서 스크립트 로봇)
    우리 로봇 프레임의 [존재, x, y, vx, vy] / PRIV_POS_M·PRIV_VEL_MPS, [-1, 1] 로 자름. 없는 물체는
    0."""
    c, s = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
    d = pos - xy[:, None]
    p = torch.stack((c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]), -1)
    v = torch.stack((c * vel[..., 0] + s * vel[..., 1], -s * vel[..., 0] + c * vel[..., 1]), -1)
    m = present.float()[..., None]
    p = (p / PRIV_POS_M).clamp(-1, 1) * m
    v = (v / PRIV_VEL_MPS).clamp(-1, 1) * m
    return torch.cat((m, p, v), -1).flatten(1)


def rate_limit(
    cmd: torch.Tensor, target: torch.Tensor, dt: float, lim: ActionLimits = ACTION_LIMITS
) -> torch.Tensor:
    """명령 (v, w) 를 목표로 (acc_v, acc_w) * dt 이내로만 옮긴다."""
    step = torch.tensor((lim.acc_v * dt, lim.acc_w * dt), device=cmd.device, dtype=cmd.dtype)
    return cmd + (target - cmd).clamp(-step, step)


def cmd_response_active(spec: TaskSpec = SPEC) -> bool:
    """명령 응답 모델이 무언가를 바꾸는지 (기본값이면 env 가 모델을 건너뛰어 이전과 같음)."""
    return (
        tuple(spec.cmd_delay_steps_range) != (0, 0)
        or tuple(spec.cmd_lag_tau_range) != (0.0, 0.0)
        or tuple(spec.cmd_gain_v_range) != (1.0, 1.0)
        or tuple(spec.cmd_gain_w_range) != (1.0, 1.0)
    )


def cmd_delay_slots(spec: TaskSpec = SPEC) -> int:
    """지연 버퍼 칸 수: 가장 긴 지연 + 1 (지금 목표 포함)."""
    return int(spec.cmd_delay_steps_range[1]) + 1


def _uniform(n: int, device, rng: tuple[float, float]) -> torch.Tensor:
    lo, hi = rng
    if lo == hi:
        return torch.full((n,), float(lo), device=device)
    return lo + (hi - lo) * torch.rand(n, device=device)


def sample_cmd_response(n: int, device, spec: TaskSpec = SPEC):
    """에피소드별 응답 모델 값: (지연 스텝 (n,) long, 시간 상수 (n,) s, 이득 (n, 2) v·w).
    범위가 한 값이면 난수를 쓰지 않는다 (고정 값 평가와 기본 실행의 난수열 그대로)."""
    lo, hi = (int(x) for x in spec.cmd_delay_steps_range)
    if not 0 <= lo <= hi:
        raise ValueError(f"cmd_delay_steps_range must be 0 <= lo <= hi, got {(lo, hi)}")
    if lo == hi:
        delay = torch.full((n,), lo, dtype=torch.long, device=device)
    else:
        delay = torch.randint(lo, hi + 1, (n,), device=device)
    if min(spec.cmd_lag_tau_range) < 0:
        raise ValueError(f"cmd_lag_tau_range must be >= 0, got {spec.cmd_lag_tau_range}")
    tau = _uniform(n, device, spec.cmd_lag_tau_range)
    gain = torch.stack(
        (_uniform(n, device, spec.cmd_gain_v_range), _uniform(n, device, spec.cmd_gain_w_range)),
        -1,
    )
    return delay, tau, gain


def cmd_delay_step(
    buf: torch.Tensor, x: torch.Tensor, delay: torch.Tensor
) -> tuple[torch.Tensor, torch.Tensor]:
    """지연 버퍼 (N, slots, 2) 에 x (N, 2) 를 맨 앞에 넣고 env 별 delay 스텝 전 값을 낸다.
    반환 (지연된 값 (N, 2), 새 버퍼). 칸 0 이 지금 값."""
    buf = torch.cat((x[:, None], buf[:, :-1]), 1)
    rows = torch.arange(x.shape[0], device=x.device)
    return buf[rows, delay], buf


def cmd_lag_step(y: torch.Tensor, x: torch.Tensor, tau: torch.Tensor, dt: float) -> torch.Tensor:
    """1차 지연 한 스텝 (스텝 동안 입력 x 일정, 정확한 이산화): x + (y - x) exp(-dt / tau).
    tau 0 인 env 는 x 그대로."""
    beta = torch.exp(-dt / tau.clamp(min=1e-9))[:, None]
    return torch.where(tau[:, None] > 0, x + (y - x) * beta, x)


def cmd_response_step(
    buf: torch.Tensor,
    lag: torch.Tensor,
    target: torch.Tensor,
    delay: torch.Tensor,
    tau: torch.Tensor,
    gain: torch.Tensor,
    dt: float,
) -> tuple[torch.Tensor, torch.Tensor]:
    """정책 목표 (N, 2) -> 이득 -> 지연 -> 1차 지연. 반환 (응답 = 새 1차 지연 상태 (N, 2), 새 버퍼).
    변화율 제한은 이 뒤에 건다 (바퀴 명령은 계속 acc 한도 안)."""
    delayed, buf = cmd_delay_step(buf, target * gain, delay)
    return cmd_lag_step(lag, delayed, tau, dt), buf


def twist_to_wheels(cmd: torch.Tensor, geo: RobotGeometry = GEOMETRY) -> torch.Tensor:
    """(v, w) -> (왼, 오른) 바퀴 속도 rad/s."""
    v, w = cmd[:, 0], cmd[:, 1]
    half = 0.5 * geo.wheel_separation
    return torch.stack(((v - w * half) / geo.wheel_radius, (v + w * half) / geo.wheel_radius), -1)


def goal_in_robot_frame(
    robot_xy: torch.Tensor, robot_yaw: torch.Tensor, goal_xy: torch.Tensor
) -> tuple[torch.Tensor, torch.Tensor]:
    """로봇에서 본 목표의 거리 [m] 와 방위 [rad]."""
    d = goal_xy - robot_xy
    dist = d.norm(dim=-1)
    ang = torch.atan2(d[:, 1], d[:, 0]) - robot_yaw
    ang = torch.atan2(torch.sin(ang), torch.cos(ang))
    return dist, ang


def lidar_sectors(ranges: torch.Tensor, spec: TaskSpec = SPEC) -> torch.Tensor:
    """(N, 360) 거리를 섹터별 최솟값으로 줄이고 lidar_obs_max 로 자른다 (비유한값은 lidar_obs_max).
    lidar_mask_fill 이고 가림이 있으면, 가림에 걸린 섹터 가운데 유한한 레이가 없는 것을 이웃 섹터
    값으로 채운다 (가린 레이는 own_body_rays 가 NaN 으로 표시한 것)."""
    r = torch.nan_to_num(ranges, nan=spec.lidar_obs_max, posinf=spec.lidar_obs_max)
    r = r.clamp(max=spec.lidar_obs_max)
    sec = r.view(r.shape[0], spec.lidar_sectors, -1).min(dim=-1).values
    if spec.lidar_mask_fill and spec.lidar_mask_deg:
        usable = torch.isfinite(ranges).view(ranges.shape[0], spec.lidar_sectors, -1).any(dim=-1)
        touched = masked_sectors(spec, ranges.shape[1], ranges.device)
        sec = fill_masked_sectors(sec, usable, touched, spec.lidar_obs_max)
    return sec


def build_observation(
    lidar_ranges: torch.Tensor,
    goal_dist: torch.Tensor,
    goal_ang: torch.Tensor,
    vel: torch.Tensor,
    spec: TaskSpec = SPEC,
    lim: ActionLimits = ACTION_LIMITS,
    intent: torch.Tensor | None = None,
    intent_mask: torch.Tensor | None = None,
    intent_priority: torch.Tensor | None = None,
    prev_target: torch.Tensor | None = None,
    lidar_history: torch.Tensor | None = None,
    speed_cap: torch.Tensor | None = None,
) -> torch.Tensor:
    """관측 벡터, 각 항목 대략 [-1, 1]:
    [라이다 섹터 / lidar_obs_max (36), 목표 거리 / goal_obs_max, sin, cos, v / v_max, w / w_max],
    이어서 intent 로봇마다 [존재 (0/1), 경로점 x, y ...] (우리 프레임 m / intent_obs_max).
    없는 로봇과 intent 없는 env 는 모두 0.
    선택 블록(obs_blocks)은 (v, w) 와 intent 사이: prev_target (N, 2) m/s·rad/s, lidar_history
    (N, frames - 1, sectors) 현재 섹터와 같은 배율·최신 먼저, speed_cap (N,) m/s.
    입력이 없으면 각각 0, 현재 프레임, v_max 로 채움."""
    lid = lidar_sectors(lidar_ranges, spec) / spec.lidar_obs_max
    g = torch.stack(
        (
            goal_dist.clamp(max=spec.goal_obs_max) / spec.goal_obs_max,
            torch.sin(goal_ang),
            torch.cos(goal_ang),
        ),
        dim=-1,
    )
    v = torch.stack((vel[:, 0] / lim.v_max, vel[:, 1] / lim.w_max), dim=-1)
    parts = [lid, g, v]
    n = lid.shape[0]
    if spec.prev_action_obs:
        p = lid.new_zeros(n, 2) if prev_target is None else prev_target
        parts.append(torch.stack((p[:, 0] / lim.v_max, p[:, 1] / lim.w_max), -1))
    if spec.lidar_frames > 1:
        h = lidar_history
        if h is None:
            h = lid[:, None].expand(n, spec.lidar_frames - 1, -1)
        parts.append(h.reshape(n, -1))
    if spec.speed_cap_obs:
        cap = lid.new_full((n,), lim.v_max) if speed_cap is None else speed_cap
        parts.append((cap / lim.v_max)[:, None])
    per = 1 + int(spec.intent_priority > 0) + 2 * spec.intent_points
    if spec.intent_robots and intent is None:
        parts.append(lid.new_zeros(n, spec.intent_robots * per))
    elif spec.intent_robots:
        m = intent_mask[:, : spec.intent_robots].float()
        p = (intent[:, : spec.intent_robots] / spec.intent_obs_max).clamp(-1.0, 1.0)
        p = p.flatten(2) * m[..., None]
        head = [m[..., None]]
        if spec.intent_priority:  # (N, R) bool; 없는 로봇은 0
            prio = torch.zeros_like(m) if intent_priority is None else intent_priority.float()
            head.append((prio[:, : spec.intent_robots] * m)[..., None])
        parts.append(torch.cat((*head, p), -1).flatten(1))
    return torch.cat(parts, dim=-1)


def is_success(goal_dist: torch.Tensor, spec: TaskSpec = SPEC) -> torch.Tensor:
    return goal_dist < spec.success_radius


def is_collision(max_contact_force: torch.Tensor, spec: TaskSpec = SPEC) -> torch.Tensor:
    """max_contact_force: COLLISION_BODIES 의 최대 접촉력 노름 (N,)."""
    return max_contact_force > spec.collision_force


def is_out_of_bounds(robot_xy: torch.Tensor, start_xy: torch.Tensor, spec: TaskSpec = SPEC):
    return (robot_xy - start_xy).norm(dim=-1) > spec.out_of_bounds


def episode_outcome(
    goal_dist: torch.Tensor,
    max_contact_force: torch.Tensor,
    robot_xy: torch.Tensor,
    start_xy: torch.Tensor,
    spec: TaskSpec = SPEC,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """(성공, 충돌, 범위 이탈), env 마다 최대 하나. 실패가 우선하고 같은 스텝의 시간초과는 env 가 성공으로 센다."""
    collision = is_collision(max_contact_force, spec)
    oob = is_out_of_bounds(robot_xy, start_xy, spec) & ~collision
    success = is_success(goal_dist, spec) & ~collision & ~oob
    return success, collision, oob


def compute_reward(
    d_prev: torch.Tensor,
    d_now: torch.Tensor,
    success: torch.Tensor,
    failed: torch.Tensor,
    spec: TaskSpec = SPEC,
    lim: ActionLimits = ACTION_LIMITS,
    v_cap: torch.Tensor | None = None,
) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    """스텝 보상과 항별 값. failed = 충돌 또는 이탈 (시간초과 제외).
    진행 보상은 최고속도(v_max 또는 env 별 v_cap)에서 스텝당 +1."""
    top = lim.v_max if v_cap is None else v_cap
    terms = {
        "progress": spec.w_progress * (d_prev - d_now) / (top * spec.control_dt),
        "time": torch.full_like(d_now, spec.w_time),
        "arrival": spec.w_arrival * success.float(),
        "collision": spec.w_collision * failed.float(),
    }
    return sum(terms.values()), terms


def sample_goal(n: int, device, spec: TaskSpec = SPEC) -> torch.Tensor:
    """시작점 기준 목표 오프셋: 거리 균등(goal_dist_range), 방위 균등."""
    lo, hi = spec.goal_dist_range
    r = lo + (hi - lo) * torch.rand(n, device=device)
    th = (2 * torch.rand(n, device=device) - 1) * math.pi
    return torch.stack((r * torch.cos(th), r * torch.sin(th)), dim=-1)
