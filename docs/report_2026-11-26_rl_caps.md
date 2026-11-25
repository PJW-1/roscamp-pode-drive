# CAPS: 매끄러움 항을 보상에서 손실로 옮기기 (2026-11-26)

## 1. 가설과 판정 규칙 (학습 결과를 보기 전에 씀)

배경: 평가에서 정책의 w 명령 부호가 분당 약 43 번 바뀐다 (pure pursuit 는 14~25). 11-22 에 보상 벌점
`w_action_rate` (-0.05, 스텝 사이 행동 변화) 와 `w_w_flip` (-0.5, w 부호 바뀜) 을 넣었는데, 이 벌점은
샘플된 행동에 걸리므로 탐색 잡음 자체가 벌을 받는다. 그 뒤 w 의 탐색 std 가 0.139 에서 0.029 로 줄었다.
시작 체크포인트 `D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt` 의 `distribution.std_param` 을
읽은 값은 (v 0.0566, w 0.0294) 다 (Windows python, torch.load, CPU).

가설: 매끄러움 항을 보상이 아니라 손실에, 샘플이 아니라 정책 평균에 걸면 잡음은 벌을 받지 않으므로
보상 벌점 두 개를 끄고도 흔들림을 줄이면서 탐색 std 를 지킬 수 있다.

판정 규칙: 같은 체크포인트에서 같은 반복 수로 학습했을 때, 손실 쪽 매끄러움 항을 쓰고 두 보상 벌점을 끈
정책이

1. w 의 탐색 std 가 0.10 위에 머물고 (저장된 모든 체크포인트의 `distribution.std_param[1]` > 0.10),
2. 분당 w 부호 바뀜이 보상 벌점을 쓴 대조군보다 높지 않으며,
3. 같은 에피소드에서 성공률이 대조군보다 2 표준오차 넘게 낮지 않으면

가설을 받아들인다. 셋 중 하나라도 어기면 받아들이지 않는다.

주의: CAPS-A (std 재설정 없음) 는 w std 0.029 에서 시작하므로 엔트로피 보너스만으로 0.10 을 넘어야
1 번을 통과한다. 이 조건의 실질적인 시험은 `--init_std 0.3` 을 쓰는 CAPS-B 다.

## 2. 구현

| 파일 | 내용 |
| --- | --- |
| `learning/goal_reach/caps_ppo.py:21` | `action_mean`: `actor.get_latent` (관측 선택 + 정규화) → 잡음(선택) → `actor.mlp` → `distribution.deterministic_output` |
| `learning/goal_reach/caps_ppo.py:29` | `temporal_pair_index`: `[T, N, 1]` dones 에서 t < T-1 이고 t 에서 끝나지 않은 (t, n) 의 평탄 인덱스 t·N+n |
| `learning/goal_reach/caps_ppo.py:37` | `temporal_loss`: 인덱스 i 와 i+N (같은 env 의 다음 스텝) 행동 평균 차 제곱 평균 |
| `learning/goal_reach/caps_ppo.py:46` | `spatial_loss`: 관측과 잡음 더한 관측의 행동 평균 차 제곱 평균 |
| `learning/goal_reach/caps_ppo.py:51` | `CapsPPO(PPO)`: 생성자 인자 `caps_temporal`, `caps_spatial`, `caps_sigma`, 순환 정책이면 거부 |
| `learning/goal_reach/caps_ppo.py:90` | `update`: 미니배치마다 `optimizer.zero_grad()` 직후 CAPS 손실을 backward |
| `learning/store_arena/run_config.py:27,130,149` | `CAPS_CLASS_NAME`, `caps_settings` (모두 0 이면 None), `caps_record` |
| `learning/store_arena/run_config.py:95` | run_config.json 의 `caps` 항목 |
| `learning/store_arena/train.py:64-82` | `--caps_temporal`, `--caps_spatial`, `--caps_sigma` (기본 0) |
| `learning/store_arena/train.py:141-144` | 계수가 하나라도 0 보다 클 때만 `agent_cfg.algorithm` 에 `class_name="caps_ppo:CapsPPO"` 와 `caps_*` 를 넣음 |
| `learning/store_arena/train.py:204` | run.json 의 `caps` 항목 |

### 클래스 선택

설치된 rsl_rl 5.5.1 의 `OnPolicyRunner` 는 `resolve_callable(cfg["algorithm"]["class_name"])` 로
알고리즘을 찾고 (`runners/on_policy_runner.py:39`), `"모듈:클래스"` 문자열을 `importlib` 로 읽는다
(`utils/utils.py:130`). train.py 는 이미 `learning/goal_reach` 를 sys.path 에 넣으므로
`"caps_ppo:CapsPPO"` 로 저장소 안의 클래스를 고를 수 있다. Isaac Lab `to_dict` 는 설정 객체의
`__dict__` 를 그대로 옮기므로 `setattr` 로 넣은 `caps_*` 가 `PPO.construct_algorithm` 을 거쳐
`CapsPPO.__init__` 인자로 간다. 이 경로 (Isaac Lab 설정 → 러너) 는 Isaac 을 띄우지 않아 실제로는 돌려 보지 않았다.

### 손실을 끼우는 자리

rsl_rl `PPO.update` 는 미니배치마다 손실 계산 → `optimizer.zero_grad()` → `loss.backward()` →
`clip_grad_norm_` (액터, 크리틱 따로) → `optimizer.step()` 순서다 (`algorithms/ppo.py`). PPO 미니배치는
섞인 평탄 인덱스만 담고 다음 스텝 관측을 담지 않으므로, update 를 복사하지 않고 `update()` 동안
`optimizer.zero_grad` 를 감싸 zero_grad 바로 뒤에 CAPS 손실을 backward 한다. 기울기는 더해지므로
(PPO 손실 + CAPS 손실) 의 기울기와 같고, 기울기 자르기와 step 을 함께 거친다. update 가 끝나면
(예외여도) 원래 zero_grad 로 돌려 놓는다. CAPS 표본은 롤아웃 전체 (`storage.observations`, T=24 × N) 에서
미니배치마다 PPO 미니배치 크기만큼 따로 뽑는다 (시간 항은 유효한 쌍 중에서). 손실 기록에
`caps_temporal`, `caps_spatial` 평균이 더해져 텐서보드에 나온다.

### 기울기가 가는 곳

두 항은 `get_latent` → `mlp` → 평균만 지난다. `GaussianDistribution` 의 평균은 MLP 출력이고 std 는
따로 있는 `std_param` 이라 std 에는 기울기가 없다. 크리틱은 부르지 않는다. 관측 정규화기는 기울기로
배우지 않는다 (통계는 update 끝의 `update_normalization`). 시험 `test_terms_reach_only_the_actor_mean`
이 이를 확인한다.

### 시간 항의 쌍

storage 의 dones[t] 는 스텝 t 의 행동 뒤에 에피소드가 끝났는지이고 (Isaac Lab
`RslRlVecEnvWrapper` 가 `terminated | truncated` 로 만들므로 타임아웃 포함, `vecenv_wrapper.py:162`),
observations[t+1] 은 그때 이미 새 에피소드의 첫 관측이다. 그래서 dones[t] 인 (t, t+1) 은 뺀다. 롤아웃의
마지막 스텝 (t = T-1) 은 다음 관측이 storage 에 없어 뺀다. 논문 코드 (baselines PPO) 는 에피소드 끝 쌍을
빼지 않지만, 이 과제는 빼도록 정했다.

### 공간 항의 잡음: 정규화 뒤

잡음은 액터의 관측 정규화 뒤 (정규화된 단위) 에 더한다. 관측 44 개는 라이다 섹터, 목표 거리·방향, 속도,
이전 행동, 속도 상한처럼 단위와 크기가 다르다. 정규화 전에 sigma 하나를 쓰면 항목마다 뜻이 달라지고,
정규화 뒤에 쓰면 모든 항목에서 "그 항목 퍼짐의 sigma 배" 로 같다. 시작 체크포인트 정규화기의 `_std` 는
0.032~0.82 이고 0 에 가까운 항목은 없어 (가장 작은 값 0.0318) 정규화 단위가 터지는 항목은 없다.
논문 코드는 원 관측에 잡음을 더한 뒤 정규화하고 ±5 로 자르지만, 이 구현은 자르지 않는다 (rsl_rl
`EmpiricalNormalization` 이 자르지 않으므로 학습·추론 경로와 같게 둠).

### 기존 옵션과 함께

- 비대칭 크리틱 (`critic` 그룹): 액터는 `obs_groups["actor"]` 만 읽으므로 그대로 동작 (시험의 관측에 `critic` 그룹이 있음).
- symmetry 확장: `super().update()` 안에서 그대로 돈다. CAPS 표본은 storage 의 원본 관측만 쓴다 (거울 사본에는 걸지 않음).
- `--init_std`: 러너를 만든 뒤 `std_param` 을 바꾸는 것이라 CAPS 와 무관. 시험에서 std 0.3 으로 바꾼 뒤 symmetry 와 함께 update 가 돈다.
- 계수가 모두 0 이면 train.py 는 `class_name` 을 건드리지 않아 rsl_rl `PPO` 그대로이고 추가 난수도 없다.
  `CapsPPO` 를 계수 0 으로 만들어도 `super().update()` 만 부른다.

## 3. 첫 값과 이유

논문 보충자료 (CAPS_SmoothControl_Supplementary.pdf, B.2) 의 PPO 값: Pendulum λT = 1e-2, λS = 5e-2,
Lunar Lander λT = 1e-3, λS = 5e-3. 논문 코드 (`baseline_ppo/ppo/pposgd_simple.py`) 의 항은
`tf.nn.l2_loss(mu_a - mu_b) / batch` = 0.5·Σ(차²) 의 배치 평균이고, 잡음 표준편차 기본값은
`trainer.py` 의 `eps_s = 0.2` (원 관측 단위) 다.

- 같은 식으로 맞추기: 이 구현은 행동 두 개 (v, w) 에 대한 차 제곱의 평균, 즉 0.5·(Δv² + Δw²) 이라 행동
  차원당 무게가 논문 코드 (0.5·Δ²) 와 같다.
- 행동 단위: Pendulum 토크는 [-2, 2], 여기 행동은 [-1, 1] 이다. 같은 상대 매끄러움에서 차² 은 1/4 이 되고,
  PPO 대리 손실의 평균에 대한 기울기는 1/std 에 비례해 단위가 반이면 두 배가 된다. 손실 기울기 2λΔ 를
  대리 손실 기울기와 같은 비율로 두려면 λ 를 4 배로 해야 한다: λT = 0.04, λS = 0.2.
- 제어 주기: Pendulum dt = 0.05 s 로 여기와 같은 20 Hz 라 시간 항의 스텝 간격은 그대로 옮긴다.
- sigma: 논문 Pendulum 의 0.2 (원 단위) 는 cos·sin (퍼짐 약 0.7) 에서 약 0.3 배, 각속도에서 그보다
  작다. 여기서는 정규화 단위 0.1 로 둔다: 라이다 섹터 정규화 `_std` 가 약 0.1 이라 섹터 값으로 약 0.01
  (같은 단위) 의 흔들림으로, 측정 잡음 수준이다.

첫 값: `--caps_temporal 0.04 --caps_spatial 0.2 --caps_sigma 0.1`.

이 값이 현재 보상 벌점 (-0.05, -0.5) 과 같은 세기인지는 알 수 없다 (보상과 손실은 단위가 다르다).
CAPS 로그 (`Loss/caps_temporal`) 가 몇 십 반복 안에 줄지 않고 w 부호 바뀜도 대조군보다 높으면 다음 단계는
두 계수를 10 배 (0.4, 2.0) 로 올리는 것이다. 이 판단은 학습 결과가 나온 뒤 오케스트레이터가 한다.

## 4. 실행 줄

세 실행 모두 같은 체크포인트, seed, 반복 수, 44 입력 레시피다. `run_train.sh` 는 무거운 작업 락을 잡고
청크 1 개를 `<run_dir>/c1` 에 쓴다 (`INIT` 환경 변수는 쓰지 않음: `--init` 은 옵티마이저 없이 가중치만 읽음).

대조군 (보상 벌점 켬, 오늘과 같은 코드 경로):

```bash
bash learning/store_arena/run_train.sh D:/isaac/runs/store_arena/caps/ctrl 1 300 4096 \
  --seed 3 --init D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt \
  --set prev_action_obs=1 --set critic_privileged=1 --set speed_cap_obs=1 --set speed_cap_range=0.08,0.16 \
  --set w_action_rate=-0.05 --set w_w_flip=-0.5 \
  --set robot_max=2 --set robot_priority_prob=0.5 --set handover_start_prob=0.15 \
  --set near_wall_m=0.08 --set w_near_wall=-0.2
```

CAPS-A (보상 벌점 끔, std 그대로):

```bash
bash learning/store_arena/run_train.sh D:/isaac/runs/store_arena/caps/caps 1 300 4096 \
  --seed 3 --init D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt \
  --set prev_action_obs=1 --set critic_privileged=1 --set speed_cap_obs=1 --set speed_cap_range=0.08,0.16 \
  --set w_action_rate=0 --set w_w_flip=0 \
  --set robot_max=2 --set robot_priority_prob=0.5 --set handover_start_prob=0.15 \
  --set near_wall_m=0.08 --set w_near_wall=-0.2 \
  --caps_temporal 0.04 --caps_spatial 0.2 --caps_sigma 0.1
```

CAPS-B (보상 벌점 끔, 행동 std 0.3 으로 재설정):

```bash
bash learning/store_arena/run_train.sh D:/isaac/runs/store_arena/caps/caps_std 1 300 4096 \
  --seed 3 --init D:/isaac/runs/store_arena/acd3/t_wall/model_1745.pt --init_std 0.3 \
  --set prev_action_obs=1 --set critic_privileged=1 --set speed_cap_obs=1 --set speed_cap_range=0.08,0.16 \
  --set w_action_rate=0 --set w_w_flip=0 \
  --set robot_max=2 --set robot_priority_prob=0.5 --set handover_start_prob=0.15 \
  --set near_wall_m=0.08 --set w_near_wall=-0.2 \
  --caps_temporal 0.04 --caps_spatial 0.2 --caps_sigma 0.1
```

비교의 한계: CAPS-B 는 std 재설정과 CAPS 두 가지가 대조군과 다르다. "std 재설정 + 보상 벌점" 대조군
(대조군 줄에 `--init_std 0.3` 만 더한 실행) 이 없으면 CAPS-B 의 std 가 0.10 위에 머무는 것이 CAPS 덕인지
가를 수 없다. 이 네 번째 실행을 돌릴지는 오케스트레이터가 정한다.

`run_config.json` 의 `caps` 항목 (`enabled`, `temporal`, `spatial`, `sigma`) 과 `agent.algorithm.class_name`,
`run.json` 의 `caps` 로 실행마다 무엇이 켜졌는지 확인할 수 있다.

## 5. 시험

Isaac 가상환경 인터프리터 (CPU, `CUDA_VISIBLE_DEVICES=` 빈 값, pytest 는 임시 폴더에서 PYTHONPATH 로):

```
CUDA_VISIBLE_DEVICES= PYTHONPATH=<temp>/pytest_pkg /d/isaac/IsaacLab/.venv/Scripts/python.exe -m pytest \
  learning/goal_reach/test_caps_ppo.py learning/store_arena/test_symmetry_train.py learning/store_arena/test_run_config.py -q -p no:cacheprovider
23 passed, 2 warnings in 5.55s
```

(경고 두 개는 가상환경의 anyio, dash pytest 플러그인의 폐기 예고.)

Windows python:

```
python -m pytest learning -q -p no:cacheprovider
253 passed, 6 skipped in 29.10s
```

변경 전 같은 커밋 (b512ee0e) 에서는 `252 passed, 5 skipped` 였다 (작업을 잠시 stash 하고 잰 값). 요청서의
"259 passed, 5 skipped" 와 다르다: 이 브랜치의 시작 커밋 기준 수는 252 다. 더해진 것은 run_config 시험 1 개
통과와 `test_caps_ppo.py` 모듈 1 개 건너뜀 (tensordict 없음) 이다.

`learning/goal_reach/test_caps_ppo.py` 가 확인하는 것:

- `test_zero_coefficients_equal_stock_ppo`: 같은 seed 에서 계수 0 의 `CapsPPO` 와 rsl_rl `PPO` 의 update 뒤 액터·크리틱 파라미터가 모두 같고 손실 기록도 같다.
- `test_caps_update_changes_the_actor_and_restores_zero_grad`: 계수를 켜면 두 항이 0 보다 크게 기록되고 액터가 PPO 와 달라지며, update 뒤 감싼 zero_grad 가 남지 않는다.
- `test_caps_gradient_is_added_to_the_ppo_gradient`: 미니배치 1 개·epoch 1 번 update 에서 `optimizer.step` 직전 (기울기 자르기 뒤) 의 기울기가 rsl_rl PPO 의 기울기 + 고정된 CAPS 대용 손실의 기울기와 같다. `caps_ppo.py` 의 `loss.backward()` 를 지우면 이 시험이 실패하는 것을 확인했다.
- `test_zero_grad_is_restored_after_an_error`: CAPS 손실에서 예외가 나도 감싼 zero_grad 가 남지 않는다.
- `test_terms_equal_hand_values`: 평균 = 관측 이 되게 만든 작은 정책에서 시간 항이 손으로 계산한 9/4, 공간 항이 같은 난수로 계산한 (0.3·ε)² 평균과 같다.
- `test_pairs_across_an_episode_end_are_excluded`: 끝난 env 의 쌍이 인덱스에서 빠지고, 새 에피소드 첫 관측이 100 이어도 그 쌍은 손실에 들어가지 않는다.
- `test_terms_reach_only_the_actor_mean`: CAPS 손실만 backward 했을 때 `std_param` 과 크리틱 파라미터의 기울기가 없거나 0 이고 액터 MLP 에는 있다.
- `test_bad_settings_stop`: sigma 없이 공간 항, 음수 계수는 거부.
- `test_runs_with_symmetry_and_a_reset_std`: 44 입력 레시피의 관측 크기, 비대칭 크리틱, symmetry (거울 사본 + 거울 손실), std 0.3 과 함께 update 가 돌고 손실이 유한하다.

`learning/store_arena/test_run_config.py::test_caps_settings_and_record`: 모두 0 이면 None 과 `enabled: False`,
켜면 `class_name` 과 값, 잘못된 조합 (sigma 없는 공간 항, 공간 항 없는 sigma, 음수, NaN) 거부.
`--caps_sigma` 만 0 보다 크고 두 계수가 0 이면 아무것도 하지 않으므로 조용히 PPO 로 돌리지 않고 거부한다.

검수: Codex (gpt-6.1-terra, 읽기 전용) 가 커밋 8503b976 을 봤다. 지적 세 가지 중 "CAPS 기울기 합산을 시험이
확인하지 않는다" 는 맞아서 위 두 시험을 더했다 (커밋 d5276a44). "sigma 만 양수인 입력을 거부한다" 는 의도한
동작이다 (모두 0 이면 PPO 라는 요구와 맞고, 효과 없는 설정을 막음). "설정 전달과 타임아웃의 통합 검증이 없다" 는
맞고 아래 "시험하지 않은 것" 에 남긴다 (타임아웃이 dones 에 들어가는 것은 Isaac Lab 소스로 확인).

시험하지 않은 것: Isaac Lab 설정 → `create_rsl_rl_runner` → `CapsPPO` 생성 경로, GPU 에서의 동작, 학습 속도
영향 (미니배치마다 액터 전방 계산이 시간 항 2 번, 공간 항 2 번 더해진다).

## 6. 학습 결과

(비어 있음: 학습을 돌리지 않았다. 위 세 실행의 결과와 판정 규칙의 1~3 번 판정을 여기에 적는다.)
