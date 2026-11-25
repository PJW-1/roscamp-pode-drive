# 2026-11-26: RL 정책의 좌우 거울 대칭 (변환, 진단, 학습 옵션)

- 브랜치 `feat/rl-symmetry-2026-11-26`. Isaac 시뮬레이터와 GPU는 쓰지 않았다. 결과 절은 비어 있고 오케스트레이터가 채운다.
- 모두 기본값이 꺼짐이다. `play.py --dump_obs`는 0, `train.py --symmetry`는 `off`다. 이때 symmetry_cfg를 두지 않으므로 기존 실행과 체크포인트는 바뀌지 않는다.

## 1. 가설과 판정 규칙 (결과 전에 씀)

- 가설: 주행은 좌우 거울 대칭이다. 로봇이 보는 세상을 좌우로 뒤집으면 맞는 행동은 같은 속도와 반대 방향 회전이다. 지금 정책은 이것을 따로 배운 적이 없다.
- 규칙: acd3의 실제 관측에서 잰 등변 오차가 크다고 하자. 그러면 같은 출발점에서 거울 증강(`--symmetry aug`)으로 이어 학습한 정책은 성공률이 오르거나, 성공률이 같으면서 오차가 줄어야 한다.
  - 비교는 같은 에피소드에서 한다(`play.py --episodes 1 --seed 7`, env 번호로 짝짓기).
  - 짝 차이가 2 표준오차(2 SE)를 넘을 때만 차이로 센다.
  - 대조 실행은 같은 init과 같은 반복 수로 `--symmetry off`를 돌린 것이다.
- "크다"의 기준은 미리 정하지 않았다. 아래 3절의 근사 오차보다 큰 오차만 정책 탓으로 볼 수 있다는 점만 정해 둔다.

## 2. 사실 (1단계)

### 로봇 몸체와 라이다

- Isaac 로봇은 `make_pinky_cfg()`다(`learning/goal_reach/goal_reach_env.py:70`). 충돌 메쉬는 `C:/isaac_assets/pinky/pinky.urdf`이고, USD에서는 convex hull로 근사한다.
- **base_link 충돌 메쉬는 좌우 대칭이 아니다.**
  - `meshes/collision/base_link.stl`의 꼭짓점 범위는 x −0.0745..0.0387, y −0.0406..+0.0473 m다(직접 재 봄).
  - 왼쪽(+y)으로만 0.0473 m까지 나온 꼭짓점이 36개 있다. 이 탭은 x −0.048..−0.045, base_link z 0.014..0.016 근처에 있다.
  - 그래서 convex hull은 왼쪽이 6.7 mm 더 넓다.
- 나머지 충돌 몸체는 꼭짓점 범위로 보면 y 대칭이다. 이 항목은 하위 에이전트가 쟀고 나는 다시 재지 않았다.
  - screen_mount: ±0.038
  - front_camera_mount: ±0.0225
  - rplidar_mount: ±0.0278
  - 두 바퀴: y ±0.04055
  - 캐스터
  - screen 메쉬는 꼭짓점 범위만 대칭이고, hull 모양 자체가 정확히 대칭인지는 모른다.
- 스침 검사용 발자국 상자는 대칭이다. `footprint_y = (-0.047, 0.047)`(`learning/store_arena/arena_spec.py:86-87`)이고, 탭까지 덮는다. 따라서 이 상자로 보면 오른쪽에 약 6 mm 여유가 더 있다.
- 라이다는 base_link (−0.017, 0, 0.097)에 있다(`goal_reach_env.py:51-54`, URDF `rplidar_mount_fixed_joint` xyz −0.017 0 0.067에 +0.030). 즉 정면 축 위에 있고 y 오프셋이 없다.
  - 레이는 `ray_alignment="base"`이고 `horizontal_fov_range=(-180, 179)`다(`goal_reach_env.py:55-62`).
  - URDF의 rplidar_link yaw π는 Isaac 레이에 적용되지 않는다. 실물 쪽은 `export_policy.scan_to_rays`(`learning/goal_reach/export_policy.py:94-112`)가 `lidar_yaw`를 빼서 같은 360 레이로 바꾼다.

### 레이와 섹터 배치

- 레이 i의 각도는 base 프레임 −180 + i deg다(`arena_spec.py:306-308`, `task_spec.py:4`).
- `lidar_sectors`는 360 레이를 (36, 10)으로 접어 최솟값을 취한다(`task_spec.py:363-367`). 따라서 섹터 k는 레이 10k..10k+9, 즉 −180+10k..−171+10k deg다. 오케스트레이터가 찾은 내용이 맞다.
  - 정면 0 deg 레이(레이 180)는 섹터 18(0..+9 deg, 왼쪽)에 들어간다. 섹터 17은 −10..−1 deg다.
- 거울(각 θ → −θ)에서 레이 i의 짝은 레이 (360 − i) mod 360이다. 레이 0(−180)과 레이 180(0)은 자기 자신이 짝이다.
  - 섹터 k의 거울 창은 레이 351−10k..360−10k이고, 섹터 35−k는 레이 350−10k..359−10k다.
  - 두 창은 9개 레이를 공유하고 한 레이씩 다르다. 정확한 거울 섹터는 섹터 값만으로는 만들 수 없다.
- **대칭 섹터 배치를 쓰려면 새 관측 형식이 필요하다.**
  - 정수 deg 레이에서 거울이 고정하는 레이는 0과 180, 두 개뿐이다.
  - 레이 0을 담은 섹터가 자기 거울이 되려면 레이 수가 홀수여야 하므로, 10 레이 섹터 36개로는 대칭 분할이 안 된다.
  - 대칭 분할에는 다음 중 하나가 필요하다.
    - 크기가 다른 섹터(예: 앞뒤 섹터 11개와 9개)
    - 반 deg 어긋난 레이 격자
  - 어느 쪽이든 `task_spec.lidar_sectors`, 플러그인의 섹터 계산, policy_spec 형식이 모두 바뀐다. 배포된 정책이 지금 배치에 묶여 있으므로 이번에는 바꾸지 않았다.

### 세상 자체의 좌우 치우침

하위 에이전트가 읽고, 내가 위치를 확인한 것이다.

- 스크립트 플릿 로봇(`learning/store_arena/robot_spec.py`):
  - 양보는 거리만 본다(`held = (g1 < yield_m) & (g1 < g0 - 1e-9)`, 252행).
  - 잠금(259-261행), 노드 예약(263-281행), 물러남(287-296행)도 자기 경로를 따라 거리만 본다.
  - 우선권은 에피소드별 확률로 yield_m = 0을 둔다(`store_arena_env.py:466-467`).
  - 왼쪽/오른쪽, 외적 부호, 통행 방향을 쓰는 규칙은 없다.
- mover(`mover_spec.py`):
  - 구간 방향은 균등 2π다(99행).
  - 양보는 거리만 본다(194행). 돌아서기는 자기 구간을 따라간다.
- 시작 자세:
  - yaw는 균등 [−π, π)다(`store_arena_env.py:303`).
  - 코너 시작 jitter는 대칭이다(`arena_spec.py:263-266`).
  - 인계 시작의 side는 0.5 확률로 정한다(`arena_spec.py:278`).
  - 박스 옆 이동은 world x, y 각각 균등 ±이다(`route_spec.py:157`). 다만 `route_seed=0`으로 한 번만 뽑으므로, 유한한 배치가 우연히 한쪽으로 치우칠 수는 있다(확인 안 함).
- 지도 v7(`pc/fleet_core/fleet_core/data/fleet_map_v7.yaml`)은 거울 대칭이 아니다.
  - 남쪽 벽에만 충전 칸막이가 있다.
  - 진열대 배치와 노드가 한쪽으로 치우쳐 있다.
  - 통행은 한 방향이다.
  - 따라서 전역 경로를 따라가는 실제 분포에는 회전 방향 치우침이 있을 수 있다. 크기는 재지 않았다.
- 전역 경로(`route_spec.py`)는 8방향 Dijkstra다. 박스를 양쪽으로 돌아가는 비용이 같을 때 어느 쪽을 고르는지는 격자 순서로 정해진다. 한쪽을 선호하는지는 재지 않았다.

## 3. 거울 변환이 각 블록에 하는 일

`learning/goal_reach/symmetry.py`가 변환을 담당한다. `task_spec.obs_names`의 이름마다 (짝 이름, 부호)를 정해 순열과 부호 표를 만든다.

| 블록 | 항목 | 거울 |
|---|---|---|
| 라이다 섹터 (36) | lidar_sector_k | lidar_sector_{35−k} (근사, 아래) |
| 목표 | goal_dist, goal_sin, goal_cos | 그대로, −, 그대로 |
| 속도 | v, w | 그대로, − |
| prev_action | prev_v, prev_w | 그대로, − |
| lidar_history | lidar_hist{f}_sector_k | 프레임마다 섹터 35−k |
| speed_cap | speed_cap | 그대로 |
| intent | robot{r}_present, _priority, p{k}_x, p{k}_y | 그대로, 그대로, 그대로, − |
| 크리틱 특권 (물체마다) | present, x, y, vx, vy | 그대로, 그대로, −, 그대로, − |
| 행동 | a_v, a_w | 그대로, − |

- 모든 변환은 자기 역변환이다(두 번 하면 원래 값).
- 모르는 이름이나 블록이 나오면 표를 만들 때 멈춘다. `obs_blocks`에 새 블록이 생기거나 `obs_names`에 새 항목이 생기는 경우가 여기에 해당한다.
- 섹터 35−k 근사의 크기는 실제 v7 아레나에서 쟀다.
  - 조건: 벽, 진열대, 박스 2개, 라이다가 장애물에서 8 cm 넘게 떨어진 자세 2000개.
  - 측정량: 거울 장면의 섹터와 섹터 35−k의 차이, m 단위(`python learning/goal_reach/test_symmetry.py`).

  | 범위 | 항목 수 | 정확(1e-5 m 미만) | 평균 | p50 | p90 | p95 | p99 | 최대 |
  |---|---|---|---|---|---|---|---|---|
  | 전체 | 72000 | 0.166 | 0.0123 | 0.0019 | 0.0148 | 0.0288 | 0.3412 | 1.6010 |
  | 섹터 0.25 m 미만 | 27370 | 0.139 | 0.0055 | 0.0010 | 0.0044 | 0.0064 | 0.0381 | 1.6010 |

  - 섹터 하나의 차이는 대부분 mm 수준이다. 섹터 경계의 한 레이가 진열대 모서리나 틈을 지나는 드문 경우에는 수십 cm에서 1.6 m까지 커진다.
  - 자세마다 36 섹터 중 최대 오차의 평균은 0.234 m다.
  - 관측 단위(÷ lidar_obs_max 2.0)로는 위 값의 절반이다.
- 그래서 정책의 등변 오차에는 이 근사 탓 몫이 들어 있다. 거울 증강으로 학습하면 정책은 근사된 거울에 맞춰진다.

## 4. 시험 (실제 출력)

- 기하 증명: `learning/goal_reach/test_symmetry.py`의 `test_observation_of_mirrored_scene_is_mirrored_observation`
  - 장면 자체를 뒤집는다. 대상은 rect, 로봇 자세, 이력 프레임 자세, 목표, 속도, 직전 목표, intent 경로점(월드), 특권 물체 위치·속도(월드)다.
  - 그다음 `arena_spec.raycast_rects`와 `task_spec.build_observation`, `privileged_block`으로 관측을 다시 만든다. 블록을 모두 켠 512 자세에서 아래가 성립한다.
    - 섹터가 아닌 항목 전부와 특권 항목은 `M obs`와 1e-5 안에서 같다(float32 삼각함수 반올림).
    - 현재 섹터와 이력 섹터의 허용 오차는 항목마다 두 가장자리 레이(350−10k와 (360−10k) mod 360)의 거리 차이다. 이 상한은 |min(A, a) − min(A, b)| ≤ |a − b|에서 나온다. 이 상한이 실제로 쓰이는지(최대 오차 > 0.01)와 중앙값이 5 mm 미만인지도 확인한다.
  - `test_mirrored_scan_is_the_ray_mirror`: 레이 단위로는 거울 장면의 레이 i가 원래 레이 (360 − i) mod 360과 1e-5 m 안에서 같다.
- 배치: 선택 블록 64개 조합마다 다음을 확인한다.
  - 표 길이 = `obs_dim` = `build_observation` 폭
  - 자기 역변환
  - 이름별 규칙
  - `test_every_optional_field_switched_on_is_mirrored`는 `OVERRIDABLE`의 정수 필드를 모두 켠다. 거울 규칙 없는 새 블록이 생기면 여기서 실패한다.
- 진단 `learning/store_arena/test_symmetry_check.py`(손으로 만든 MLP):
  - 은닉 두 반쪽이 x와 Mx를 보는 정확히 등변인 망은 v, w 오차와 치우침 모두 1e-5 미만이다.
  - 같은 망에 w 출력 bias 0.2를 더하면 치우침 0.2, w 오차 0.4가 나온다.
  - 덤프 행동과 1e-3 다르면 멈춘다.
  - Isaac 인터프리터에서는 rsl_rl `MLPModel`(정규화 켬, Gaussian)의 결정적 출력과 1e-5 안에서 같다.
- 학습 옵션 `learning/store_arena/test_symmetry_train.py`:
  - TensorDict("policy" + "critic")로 순서(원본 먼저), 모양, 값을 확인한다.
  - rsl_rl `Symmetry`에 Isaac Lab `to_dict`가 만드는 문자열 `"symmetry:augment"`를 넘겨 함수를 찾는지 확인한다.
  - `augment_batch`가 다른 텐서를 두 배로 늘리는지, `compute_loss`가 손으로 계산한 MSE와 같은지 확인한다.
- Windows python(이 브랜치 전 113 passed):

  ```
  $ python -m pytest learning -q -p no:cacheprovider
  190 passed, 5 skipped in 10.97s
  ```

  건너뛴 5개는 tensordict 또는 rsl_rl이 필요한 시험이다.
- Isaac 가상환경 인터프리터:
  - 이 venv에는 pytest가 없어서 Windows python으로 `pip install --target`한 임시 폴더를 PYTHONPATH로 붙였다. Isaac venv 자체는 건드리지 않았다.
  - 가져온 것은 tensordict와 rsl_rl뿐이다(isaaclab.app 없음).

  ```
  $ PYTHONPATH=<temp>/pyt /d/isaac/IsaacLab/.venv/Scripts/python.exe -m pytest learning/store_arena/test_symmetry_train.py learning/store_arena/test_symmetry_check.py learning/goal_reach/test_symmetry.py learning/store_arena/test_run_config.py -q -p no:cacheprovider
  86 passed, 2 warnings in 6.15s
  ```

  경고 2개는 venv의 anyio와 dash pytest 플러그인이 내는 지원 중단 경고다.

## 5. 학습 옵션

- `learning/store_arena/train.py --symmetry {off,aug,loss,both}`가 `agent_cfg.algorithm.symmetry_cfg = RslRlSymmetryCfg(data_augmentation_func=symmetry.augment, ...)`를 둔다.
  - `aug`는 use_data_augmentation, `loss`는 use_mirror_loss, `both`는 둘 다다.
  - `run_config.json`의 `"symmetry"`와 `run.json`에 모드를 적는다.
- `symmetry.augment(env, obs, actions)`는 TaskSpec을 `env.unwrapped.spec`에서 읽고 원본 뒤에 거울 사본을 붙인다. TaskSpec이 아니면 멈춘다.
  - 처리하는 그룹은 "policy"(액터 관측)와 "critic"(critic_privileged의 특권 항목)이다. 다른 그룹이 있으면 멈춘다.
- 거울 손실 계수 기본값은 1.0이다(`--mirror_loss_coeff`, `run_config.MIRROR_LOSS_COEFF`). 이유는 다음과 같다.
  - rsl_rl의 손실은 행동 평균의 MSE다. 행동은 [−1, 1] 단위이고, w 차이가 0.1이면 손실은 0.005 정도다.
  - 이 값을 같은 배치에서 value_loss_coef 1.0과 같은 크기로 더하는 것이 가장 단순한 출발점이다.
  - 조정해 본 값이 아니다.
- rsl_rl docstring(`rsl_rl/extensions/symmetry.py`, Mittal et al. 2024, arXiv 2403.04359 인용)이 말하는 것은 두 가지뿐이다.
  - 증강은 미니배치마다 거울 관측·행동 쌍을 붙여 정책·가치 손실을 원본과 거울 모두에서 평가한다.
  - 거울 손실은 정책이 거울 관측에서 자기와 어긋나는 것에 보조 MSE 벌점을 준다.
- 코드에서 확인한 사실(`augment_batch`): 거울 사본은 원본의 old log prob, value, advantage, return을 그대로 복사해 쓴다.
- 추측: 아래 두 가지는 확인하지 않았다.
  - acd3처럼 등변이 아닌 정책에서 시작하면, 처음에는 거울 샘플의 중요도 비가 1에서 멀어 많이 잘릴 수 있다.
  - 지도가 비대칭이므로, 거울 상태에 원래 return을 붙이는 것이 정확하지 않을 수 있다.

## 6. 오케스트레이터 명령

acd3(t_wall `model_1745.pt`)의 설정은 `docs/rl_acd3_2026-11-25/jobs_main.txt`에서 가져왔다.

1. 관측 덤프. 0.12 random 칸, 기존 평가와 같은 에피소드:

   ```
   /d/isaac/IsaacLab/.venv/Scripts/python.exe learning/store_arena/play.py --checkpoint D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt --out D:/isaac/runs/store_arena/sym/e_acd3_dump --num_envs 4096 --episodes 1 --seed 7 --viz none --dump_obs 200 --set prev_action_obs=1 --set critic_privileged=1 --set speed_cap_obs=1 --set speed_cap_range=0.12,0.12 --set robot_max=2 --set robot_priority_prob=0.5
   ```

   - 200 스텝 × 4096 env × 44 float는 약 144 MB(압축 전)다.
   - `--still`, `--capped`와는 같이 쓸 수 없다.
2. 등변 오차(CPU, Isaac 불필요):

   ```
   python learning/store_arena/symmetry_check.py --checkpoint D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt --dump D:/isaac/runs/store_arena/sym/e_acd3_dump/obs_dump.npz --json D:/isaac/runs/store_arena/sym/e_acd3_dump/symmetry.json
   ```

   - forward와 덤프 행동의 차이가 1e-4를 넘으면 멈춘다.
   - 출력: all / near(최소 섹터 < 0.25 m) / clear마다 |a_v(o) − a_v(Mo)|와 |a_w(o) + a_w(Mo)|의 평균·p50·p90·p95·p99·최대, 그리고 회전 치우침 평균((a_w(o) + a_w(Mo)) / 2, +는 왼쪽)이다.
3. 같은 출발점에서 증강 학습과 대조 학습. 반복 수는 오케스트레이터가 정한다. t_wall과 같은 250을 예로 들었다.

   ```
   /d/isaac/IsaacLab/.venv/Scripts/python.exe learning/store_arena/train.py --out D:/isaac/runs/store_arena/sym/t_aug --num_envs 4096 --max_iterations 250 --seed 3 --viz none --init D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt --symmetry aug --set prev_action_obs=1 --set critic_privileged=1 --set speed_cap_obs=1 --set speed_cap_range=0.08,0.16 --set w_action_rate=-0.05 --set w_w_flip=-0.5 --set robot_max=2 --set robot_priority_prob=0.5 --set handover_start_prob=0.15 --set near_wall_m=0.08 --set w_near_wall=-0.2
   ```

   - 대조 실행은 같은 줄에서 `--symmetry off`, `--out .../t_off`로 바꾼다.
   - 그다음 두 실행을 같은 `play.py --episodes 1 --seed 7` 칸에서 평가하고, 위 1-2로 오차를 잰다.
   - `--viz none`은 run_train.sh(31행)가 넘기는 값과 같게 적었다. 이 명령들은 Isaac에서 돌려 보지 않았다.

## 7. 남은 것과 정할 것

- Isaac에서 돌려 보지 않은 것:
  - `play.py --dump_obs`의 실제 덤프
  - `train.py --symmetry`가 실제 runner에서 symmetry_cfg를 받아 학습하는지
  - `RslRlSymmetryCfg`를 `isaaclab_rl.rsl_rl`에서 가져오는 것. Isaac Lab 예제 `anymal_b/agents/rsl_rl_ppo_cfg.py:8`도 같은 방식으로 가져온다.
  - 문자열 해석 경로는 rsl_rl 단위 시험으로 확인했다.
- 정할 것:
  - "acd3 오차가 크다"의 기준
  - 학습 반복 수
  - aug와 loss 중 무엇을 돌릴지. 기본 제안은 aug다. Isaac Lab 예제(anymal, cartpole)도 `use_data_augmentation=True`만 쓴다.
- 몸체 탭(왼쪽 6.7 mm)과 지도 비대칭 때문에 세상은 정확히 대칭이 아니다. 거울 증강은 이 차이를 지운다.

## 결과

(오케스트레이터가 채움)
