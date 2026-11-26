"""Isaac Lab(Newton / MuJoCo-Warp) direct RL env: Pinky 가 빈 바닥에서 무작위 목표로 간다.

과제(관측, 행동, 보상, 성공, 충돌)는 task_spec.py 에 있고, 여기서는 시뮬 상태를 읽어 넘기고
바퀴 속도 목표를 쓴다.
"""

from __future__ import annotations

import csv
import os
import sys
import time

import isaaclab.sim as sim_utils
import torch
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sensors.ray_caster import patterns
from isaaclab.sensors.ray_caster.patterns import LidarPatternCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab_newton.sensors import NewtonRaycastSensorCfg

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "sim_check"))

import task_spec as ts  # noqa: E402
from pinky_cfg import BODY, make_newton_cfg, make_pinky_cfg  # noqa: E402

PHYSICS_DT = 1.0 / 200.0
DECIMATION = round(ts.SPEC.control_dt / PHYSICS_DT)  # 10 -> 20 Hz 제어
LIDAR_POS = (-0.017, 0.0, 0.097)  # m; base_link 에서 본 rplidar_link


def lidar_min_range_pattern(cfg, device):
    """레이는 min_range 에서 시작해 로봇 자신의 rplidar 콜라이더를 안 맞힌다 (sim_check/README.md).
    반환 거리 + min_range = 중심에서의 거리."""
    starts, dirs = patterns.lidar_pattern(cfg, device)
    return starts + cfg.min_range * dirs, dirs


@configclass
class MinRangeLidarPatternCfg(LidarPatternCfg):
    func: object = lidar_min_range_pattern
    min_range: float = 0.0


def make_lidar_cfg(max_range: float) -> NewtonRaycastSensorCfg:
    """rplidar_link 의 라이다, base_link (-0.017, 0, 0.097); 레이 i 는 base 프레임 (-180 + i) deg."""
    return NewtonRaycastSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/" + BODY,
        offset=NewtonRaycastSensorCfg.OffsetCfg(pos=LIDAR_POS),
        pattern_cfg=MinRangeLidarPatternCfg(
            channels=1,
            vertical_fov_range=(0.0, 0.0),
            horizontal_fov_range=(-180.0, 179.0),
            horizontal_res=360.0 / ts.SPEC.lidar_rays,
            min_range=ts.SPEC.lidar_min_range,
        ),
        ray_alignment="base",
        max_distance=max_range - ts.SPEC.lidar_min_range,
    )


@configclass
class GoalReachSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    robot = make_pinky_cfg()
    # 레이는 자기 env(Newton world) 와 공용 바닥만 맞힘: arena_check.py --mode crossenv 로 확인
    lidar = make_lidar_cfg(ts.SPEC.lidar_obs_max)
    contact_base = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + BODY)
    contact_parts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/"
        + BODY
        + "/(screen_mount|front_camera_mount|rplidar_mount)"
    )
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))


@configclass
class GoalReachEnvCfg(DirectRLEnvCfg):
    decimation = DECIMATION
    episode_length_s = ts.SPEC.episode_s
    action_space = ts.ACT_DIM
    observation_space = ts.OBS_DIM
    state_space = 0
    sim: SimulationCfg = SimulationCfg(
        dt=PHYSICS_DT, render_interval=DECIMATION, physics=make_newton_cfg()
    )
    # 8 m 칸이면 로봇이 눈에 띄게 떨어짐 (env 는 별개 Newton world 라 라이다/접촉에는 불필요)
    scene: GoalReachSceneCfg = GoalReachSceneCfg(
        num_envs=4096, env_spacing=8.0, replicate_physics=True
    )
    # 지정하면 metrics_every 스텝마다 에피소드 결과 한 행을 추가
    metrics_csv: str = ""
    metrics_every: int = 24
    # 덮어쓸 TaskSpec 필드: task_spec.OVERRIDABLE 만
    spec_overrides: dict = {}


class GoalReachEnv(DirectRLEnv):
    cfg: GoalReachEnvCfg

    def __init__(self, cfg: GoalReachEnvCfg, render_mode: str | None = None, **kwargs):
        self.spec = ts.make_spec(cfg.spec_overrides)
        s = self.spec
        recipe = set(ts.obs_blocks(s)) - {"intent"} or s.critic_privileged or s.w_action_rate
        recipe = recipe or s.w_w_flip
        recipe = recipe or tuple(s.speed_cap_range) != (ts.NAV_SPEED_MPS, ts.NAV_SPEED_MPS)
        if type(self) is GoalReachEnv and recipe:  # 이 필드들은 매장 아레나만 채움
            raise ValueError(
                "the recipe fields (task_spec obs_blocks, speed cap, critic_privileged, "
                "action-rate weights) need the store arena env"
            )
        cfg.observation_space = ts.obs_dim(self.spec)  # intent_robots 면 넓어짐
        super().__init__(cfg, render_mode, **kwargs)
        self.robot = self.scene["robot"]
        self.lidar = self.scene["lidar"]
        self.wheel_ids, _ = self.robot.find_joints(
            ["l_wheel_joint", "r_wheel_joint"], preserve_order=True
        )
        n, dev = self.num_envs, self.device
        self.cmd = torch.zeros(n, 2, device=dev)  # 변화율 제한된 (v, w) 명령
        # 앞 막기 (평가 전용, play.py --front_stop): (거리 m, 반각 deg) 이면 켬. 학습에서는 None
        self.front_stop = None
        self.front_blocked = torch.zeros(n, dtype=torch.bool, device=dev)  # 지난 스텝에 막았는지
        # 움직이는 것 앞에서만 서는 규칙 둘 (평가 전용): play.py --closing_stop 은 ClosingGuard,
        # --person_stop 은 (거리 m, 반각 deg). 학습에서는 None. 막았는지는 front_blocked 에 적는다
        self.closing_guard = None
        self.person_stop = None
        # 관측 라이다 거리 배율 (lidar_scale_range): 리셋 때 env 별로 뽑음
        self.lidar_scale = ts.sample_lidar_scale(n, dev, self.spec)
        # 명령 응답 모델 (cmd_*_range): 끄면 (기본) _pre_physics_step 이 건너뜀
        self.cmd_response = ts.cmd_response_active(self.spec)
        self.resp_delay, self.resp_tau, self.resp_gain = ts.sample_cmd_response(n, dev, self.spec)
        self.resp_buf = torch.zeros(n, ts.cmd_delay_slots(self.spec), 2, device=dev)
        self.resp_lag = torch.zeros(n, 2, device=dev)
        self.start_xy = torch.zeros(n, 2, device=dev)
        self.goal_xy = torch.zeros(n, 2, device=dev)
        self.d_prev = torch.zeros(n, device=dev)
        self.d_now = torch.zeros(n, device=dev)
        self.ang_now = torch.zeros(n, device=dev)
        self.success = torch.zeros(n, dtype=torch.bool, device=dev)
        self.collided = torch.zeros(n, dtype=torch.bool, device=dev)
        self.oob = torch.zeros(n, dtype=torch.bool, device=dev)
        self.last_ep_len_s = torch.zeros(n, device=dev)  # env 별 마지막으로 끝난 에피소드의 길이와
        self.last_ep_dist = torch.zeros(n, device=dev)  # 최종 거리 (play.py 용)
        self.ep_reward_terms: dict[str, torch.Tensor] = {}
        # 지난 metrics 행 이후 결과 수: done, success, collision, oob, timeout
        self._win = torch.zeros(5, device=dev)
        self._win_succ_time = torch.zeros((), device=dev)
        self._win_final_dist = torch.zeros((), device=dev)
        self._steps = 0
        self._t0 = time.time()
        print(f"[GOAL] contact_parts bodies: {self.scene['contact_parts'].body_names}")
        print(
            f"[GOAL] obs dim {ts.obs_dim(self.spec)}, decimation {self.cfg.decimation}, "
            f"limits {ts.ACTION_LIMITS}"
        )
        print(f"[GOAL] spec {self.spec}")

    # ---- 시뮬 상태 -> 과제 입력 ----
    def robot_pose(self) -> tuple[torch.Tensor, torch.Tensor]:
        pos = self.robot.data.root_link_pos_w.torch[:, :2] - self.scene.env_origins[:, :2]
        q = self.robot.data.root_link_quat_w.torch  # x, y, z, w
        x, y, z, w = q.unbind(-1)
        yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return pos, yaw

    def lidar_ranges(self) -> torch.Tensor:
        return self.lidar.data.ray_distances.torch + ts.SPEC.lidar_min_range

    def max_contact_force(self) -> torch.Tensor:
        fb = self.scene["contact_base"].data.net_forces_w.torch.norm(dim=-1).amax(dim=-1)
        fp = self.scene["contact_parts"].data.net_forces_w.torch.norm(dim=-1).amax(dim=-1)
        return torch.maximum(fb, fp)

    def body_velocity(self) -> torch.Tensor:
        return torch.stack(
            (
                self.robot.data.root_link_lin_vel_b.torch[:, 0],
                self.robot.data.root_link_ang_vel_b.torch[:, 2],
            ),
            dim=-1,
        )

    # ---- DirectRLEnv 훅 ----
    def action_target(self, actions: torch.Tensor) -> torch.Tensor:
        """정책 행동의 목표 (v, w) (매장 아레나는 v 를 에피소드별 상한에 맞춤)."""
        return ts.action_to_target(actions)

    def moving_things(self) -> list:
        """사람 위치를 아는 막기가 보는 mover 별 (위치, 속도, 활성). 이 과제에는 없다."""
        return []

    def make_closing_guard(self, stop_range: float, half_width_deg: float) -> ts.ClosingGuard:
        """다가옴 막기 (play.py --closing_stop) 를 이 env 의 제어 주기와 라이다 자리로 만든다."""
        return ts.ClosingGuard(self.step_dt, stop_range, half_width_deg, lidar_x=LIDAR_POS[0])

    def reward_v_cap(self) -> torch.Tensor | None:
        """진행 보상의 env 별 최고속도 (None: ActionLimits.v_max)."""
        return None

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.target = target = self.action_target(actions)
        # 앞 막기는 명령만 바꾼다. 관측의 직전 행동 (self.target) 은 정책 목표 그대로다
        # (플러그인의 뒤쪽 막기와 같은 자리)
        if self.front_stop is not None:
            target, self.front_blocked = ts.block_forward(
                target, self.lidar_ranges(), *self.front_stop
            )
        if self.closing_guard is not None:
            target, self.front_blocked = self.closing_guard.step(
                target, self.lidar_ranges(), *self.robot_pose()
            )
        if self.person_stop is not None:
            target, self.front_blocked = ts.block_for_movers(
                target, *self.robot_pose(), self.moving_things(), *self.person_stop
            )
        if self.cmd_response:  # 정책 목표 -> 실물 응답 모델 -> 변화율 제한
            target, self.resp_buf = ts.cmd_response_step(
                self.resp_buf,
                self.resp_lag,
                target,
                self.resp_delay,
                self.resp_tau,
                self.resp_gain,
                self.step_dt,
            )
            self.resp_lag = target
        self.cmd = ts.rate_limit(self.cmd, target, self.step_dt)
        self._wheel_target = ts.twist_to_wheels(self.cmd)

    def _apply_action(self) -> None:
        self.robot.set_joint_velocity_target_index(
            target=self._wheel_target, joint_ids=self.wheel_ids
        )

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        xy, yaw = self.robot_pose()
        self.d_now, self.ang_now = ts.goal_in_robot_frame(xy, yaw, self.goal_xy)
        self.success, self.collided, self.oob = ts.episode_outcome(
            self.d_now, self.max_contact_force(), xy, self.start_xy, self.spec
        )
        return ts.split_dones(
            self.success | self.collided | self.oob,
            self.episode_length_buf,
            self.max_episode_length,
        )

    def _get_rewards(self) -> torch.Tensor:
        rew, terms = ts.compute_reward(
            self.d_prev,
            self.d_now,
            self.success,
            self.collided | self.oob,
            self.spec,
            v_cap=self.reward_v_cap(),
        )
        self.d_prev = self.d_now.clone()
        for k, v in terms.items():
            self.ep_reward_terms.setdefault(k, torch.zeros_like(v))
            self.ep_reward_terms[k] += v
        self._count_outcomes()
        return rew

    def _count_outcomes(self) -> None:
        done = self.reset_terminated | self.reset_time_outs
        succ = self.success & done
        self._win += torch.stack(
            [
                done.sum(),
                succ.sum(),
                (self.collided & done).sum(),
                (self.oob & done).sum(),
                (self.reset_time_outs & ~self.reset_terminated).sum(),
            ]
        ).float()
        self._win_succ_time += (self.episode_length_buf.float() * self.step_dt * succ.float()).sum()
        self._win_final_dist += (self.d_now * done.float()).sum()
        self._steps += 1
        if self.cfg.metrics_csv and self._steps % self.cfg.metrics_every == 0:
            self._write_metrics()

    def _write_metrics(self) -> None:
        w = self._win.tolist()
        st, fd = float(self._win_succ_time), float(self._win_final_dist)
        n_done = max(w[0], 1.0)
        row = {
            "wall_s": round(time.time() - self._t0, 2),
            "iteration": self._steps // self.cfg.metrics_every,
            "env_steps": self._steps * self.num_envs,
            "episodes": int(w[0]),
            "success_rate": round(w[1] / n_done, 4),
            "collision_rate": round(w[2] / n_done, 4),
            "oob_rate": round(w[3] / n_done, 4),
            "timeout_rate": round(w[4] / n_done, 4),
            "success_time_s": round(st / max(w[1], 1.0), 3),
            "final_dist_m": round(fd / n_done, 4),
        }
        new = not os.path.exists(self.cfg.metrics_csv)
        with open(self.cfg.metrics_csv, "a", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=list(row))
            if new:
                wr.writeheader()
            wr.writerow(row)
        self._win.zero_()
        self._win_succ_time.zero_()
        self._win_final_dist.zero_()

    def sample_start(self, n: int) -> tuple[torch.Tensor, torch.Tensor]:
        """리셋하는 n 개 env 의 시작 (x, y) (env 프레임)와 yaw: 칸 중심, 임의 yaw."""
        yaw = (2 * torch.rand(n, device=self.device) - 1) * torch.pi
        return torch.zeros(n, 2, device=self.device), yaw

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        self.last_ep_len_s[env_ids] = self.episode_length_buf[env_ids].float() * self.step_dt
        self.last_ep_dist[env_ids] = self.d_now[env_ids]
        # 보상 항별 에피소드 합 평균 (텐서보드용)
        log = self.extras.setdefault("log", {})
        for k, v in self.ep_reward_terms.items():
            log[f"Episode_Reward/{k}"] = v[env_ids].mean()
            v[env_ids] = 0.0
        super()._reset_idx(env_ids)
        n = len(env_ids)
        origins = self.scene.env_origins[env_ids]
        start, yaw = self.sample_start(n)
        pose = self.robot.data.default_root_pose.torch[env_ids].clone()
        pose[:, :3] += origins
        pose[:, :2] += start
        pose[:, 3:7] = torch.stack(
            (torch.zeros_like(yaw), torch.zeros_like(yaw), torch.sin(yaw / 2), torch.cos(yaw / 2)),
            -1,
        )
        self.robot.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros(n, 6, device=self.device), env_ids=env_ids
        )
        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        self.robot.write_joint_position_to_sim_index(position=jp, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.cmd[env_ids] = 0.0
        if self.closing_guard is not None:
            self.closing_guard.reset(env_ids)
        self.start_xy[env_ids] = start
        offset = ts.sample_goal(n, self.device, self.spec)
        self.goal_xy[env_ids] = start + offset
        self.d_prev[env_ids] = offset.norm(dim=-1)
        self.d_now[env_ids] = self.d_prev[env_ids]
        self.ang_now[env_ids] = torch.atan2(offset[:, 1], offset[:, 0]) - yaw
        self.ang_now[env_ids] = torch.atan2(
            torch.sin(self.ang_now[env_ids]), torch.cos(self.ang_now[env_ids])
        )
        # 리셋하는 env 만 배율을 새로 뽑음 (한 값이면 난수를 안 써 이전 실행과 같음)
        self.lidar_scale[env_ids] = ts.sample_lidar_scale(n, self.device, self.spec)
        # 응답 모델 값도 리셋하는 env 만 새로 뽑음 (한 값이면 난수를 안 씀)
        if self.cmd_response:
            delay, tau, gain = ts.sample_cmd_response(n, self.device, self.spec)
            self.resp_delay[env_ids], self.resp_tau[env_ids] = delay, tau
            self.resp_gain[env_ids] = gain
        self.reset_cmd_response(env_ids)

    def reset_cmd_response(self, env_ids) -> None:
        """응답 모델의 지연 버퍼와 1차 지연 상태를 지금 명령 self.cmd 로 채운다 (정지면 0)."""
        self.resp_buf[env_ids] = self.cmd[env_ids][:, None]
        self.resp_lag[env_ids] = self.cmd[env_ids]

    def obs_lidar_ranges(self) -> torch.Tensor:
        """관측용 거리: lidar_ranges 에 에피소드 배율을 곱함 (시뮬 판정은 lidar_ranges 그대로)."""
        return ts.scale_ranges(self.lidar_ranges(), self.lidar_scale)

    def _get_observations(self) -> dict:
        obs = ts.build_observation(
            self.obs_lidar_ranges(), self.d_now, self.ang_now, self.body_velocity(), self.spec
        )
        return {"policy": obs}
