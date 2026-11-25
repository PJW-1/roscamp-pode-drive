# RL 주행 정책 출력 평활(배치 쪽) 보고서 (2026-11-26)

RL 계획 4번 작업 "덜 흔들리기"의 배치 쪽이다. Gazebo에서 RL+shim은 분당 w 부호를 40~80번 바꾸고 RPP는 14~25번 바꾼다.
플러그인 `pode_rl_controller::FollowPathRL`에 끔이 기본인 선택 둘을 더했고, Gazebo 비교는 스크립트만 준비했다(돌리지 않았다:
오늘 밤 무거운 작업 잠금과 GPU는 학습이 쓴다).

## 1. 가설과 판정 규칙(결과를 보기 전에 적음)

- 가설: 정책 목표 w에 지수 이동 평균을 걸면 부호 바뀜이 준다.
- 판정 규칙: **어떤 alpha에서 분당 부호 바뀜이 RPP 범위로 내려가면서, 성공은 100 %로 남고, 시간과 몸 여유 최솟값이 평활 없음보다
  2 SE 넘게 나빠지지 않으면** 그 alpha를 후보로 삼는다. 둘 다 맞으면 지연이 작은 쪽(alpha가 큰 쪽)을 고른다.
- "RPP 범위"는 같은 판에서 잰 RPP 칸의 판별 값 범위로 본다. 시간은 성공 판 평균의 Welch 차이, 몸 여유는 판별 최솟값의 Welch 차이다.
- 각가속도·각저크 RMS는 판정에 쓰지 않고 함께 적는다.
- 이 규칙에는 충돌 판 수 조건이 없다(성공한 판에도 몸 여유 5 mm 아래 접촉이 있을 수 있다). 충돌 판은 표에 따로 적는다.
  "2 SE 넘게 나빠지지 않음"은 칸마다 10판이라 SE가 크면 쉽게 통과하므로, 나빠지지 않았다는 증명으로 읽지 않는다.
  충돌 판 조건과 허용 폭을 더할지는 결과를 보기 전에 정해야 한다(정할 것).

## 2. 평활이 정책에게 주는 지연

평활은 `out = alpha * target + (1 - alpha) * 직전 out`이다. 20 Hz(dt 0.05 s)에서 일정한 입력에 대한 평균 지연은
`dt * (1 - alpha) / alpha`, 시간 상수는 `-dt / ln(1 - alpha)`다(계산값).

| alpha | 평균 지연 | 시간 상수 | 계단 입력의 몇 %가 한 주기 뒤에 나오나 |
|---|---|---|---|
| 1.0(끔) | 0 s | 0 s | 100 % |
| 0.5 | 0.05 s(1주기) | 0.072 s | 50 % |
| 0.25 | 0.15 s(3주기) | 0.174 s | 25 % |

이 지연은 기존 가속 제한(rate limit) 위에 더해진다. 정책은 이 지연을 학습에서 본 적이 없다. 관측의 측정 속도(v, w)는 늦어진 명령을
따라가므로 정책이 보는 세계가 그만큼 늦게 반응한다. prev_action 관측은 아래처럼 평활 전 값이므로 이 지연을 보지 않는다.

## 3. 더한 선택(기본은 모두 끔)

| 파라미터 | 기본 | 받는 값 | 하는 일 |
|---|---|---|---|
| `ema_alpha_v`, `ema_alpha_w` | 1.0(끔) | (0, 1]. 밖이면 configure에서 멈춘다 | 정책의 목표 (v, w)를 후진 막기, 속도 상한, 가속 제한 앞에서 평활한다 |
| `scan_mask_deg` | 빈 목록(끔) | base 기준 도 단위 쌍 `[lo, hi, ...]`, `-180 <= lo < hi <= 180`, 소수점으로 적는다. 홀수 개나 틀린 쌍이면 configure에서 멈춘다 | 범위 안 빔을 "돌아온 것 없음"(inf → `obs_max`)으로 바꾼 뒤 학습 광선 360개를 만든다. 후진 막기도 그 빔을 보지 않는다 |

정한 것과 까닭:
- **prev_action 기억은 평활 전 정책 목표를 담는다.** format 3 정의(학습의 `task_spec.py`)가 "정책 자신의 직전 목표"이고, 지금도
  가속 제한 전 값을 담는다(`test_format3.cpp` `PrevActionIsTheScaledTargetBeforeTheRateLimit`). 평활 값을 넣으면 정책이 학습에서
  본 적 없는 관계(자기 출력과 다른 값)를 보게 되므로 바꾸지 않았다.
- **평활 상태는 ObsMemory가 리셋되는 곳에서 같이 리셋한다**: activate, `reset()`, 마지막 점이 1 mm 넘게 바뀐 계획, 직전 주기에
  정책이 돌지 않았을 때(2주기 넘게 정책 호출이 없음: shim 회전, 목표 근처 제자리 회전). 마지막 경우는 이전에 format 3에서만
  검사했는데 이제 format 2 정책에서도 검사한다(format 2에서 ObsMemory 리셋은 관측에 쓰이지 않으므로 명령은 바뀌지 않는다).
- **리셋 뒤 첫 주기의 평활은 지금 내는 명령(`cmd_`)에서 출발한다.** 공백 뒤에는 그것이 측정 속도다. 0에서 출발하면 shim이 1 rad/s로
  돌다 넘겨준 순간 w를 0 쪽으로 당기므로 연속인 쪽을 골랐다.
- 평활은 정책 경로에만 건다. 목표 근처 제자리 회전(`rotate_gain`)에는 걸지 않는다.
- 가리기는 실물 로봇의 라이다가 몸 뒤 왼쪽(base 약 150~153°)을 0.075 m로 보는 문제(`docs/report_2026-11-24_rl_real_prep.md` S1)를
  위한 것이다. 바로 뒤(±180°)에 걸친 범위는 두 쌍으로 적고, 정확히 ±180°인 빔은 어느 쪽 끝으로도 맞는다.

코드: `pode/pode_rl_controller/src/observation.cpp`(`TargetSmoother`, `check_ema_alpha`, `parse_scan_mask`, `mask_scan`),
`src/follow_path_rl.cpp`(파라미터, 리셋, 적용 순서), README의 "Deployment options".

## 4. 시험

- WSL `colcon build --packages-up-to pode_rl_controller` 후 ctest: 7개 모두 통과(`100% tests passed, 0 tests failed out of 7`).
  다섯 parity 시험은 각각 8개 시험 통과, 추론을 돌렸다(건너뜀 0, `[parity] inference 6.9~7.4 us per call`).
- 더한 gtest:
  - `test_parity`(정책 다섯 각각) `Action.SmoothingOffAndEmptyMaskAreBitIdentical`: `action_cases.json`의 모든 단계에서 alpha 1의
    평활 출력이 목표와 비트까지 같고 가속 제한 뒤 명령 계열도 같다. `obs_cases.json`에서 빈 가리기는 광선을 비트까지 바꾸지 않는다.
  - `test_observation` `Smoothing.*`: 계단 입력이 닫힌 식 `u(1 - (1 - alpha)^n)`과 1e-12 안에서 같다, 리셋 뒤 주어진 명령에서 출발한다,
    (0, 1] 밖 alpha(0, −0.1, 1.01, NaN)는 거부한다.
  - `ScanMask.*`: base 기준 149.5~153.5°를 가리면 yaw π로 단 라이다에서 해당 빔만 inf가 되고 학습 광선 330~333만 `obs_max`, 나머지는
    그대로다. 뒤쪽 ±180° 처리, 틀린 목록 거부.
- 한계: 비트 동일은 플러그인이 쓰는 함수 단위로 확인했다. Nav2 노드를 띄워 플러그인 전체의 명령을 바꾸기 전후로 비교하지는 않았다
  (이 패키지에 플러그인 노드 시험이 없다). 끔일 때 바뀐 경로는 평활의 통과 분기, 빈 가리기의 즉시 반환, format 2에서의 쓰이지 않는
  ObsMemory 리셋뿐이다.

## 5. Gazebo 비교 준비(돌리지 않음)

`docs/rl_smooth_2026-11-25/`:
- `gz_overrides.yaml`: shim으로 감싼 `store_arena_acd3`(설정은 `follow_path_rl_acd3.yaml`의 FollowPathRL과 같다) 셋.
  `FollowPathRLShimACD3`(평활 없음), `FollowPathRLShimACD3W50`(ema_alpha_w 0.5), `FollowPathRLShimACD3W25`(ema_alpha_w 0.25). v는 평활하지 않는다.
- `gz_run.sh [OUT]`: RPP(`FollowPathGoal`), MPPI, 위 셋을 Q2 → E-3(`results/2026-11-22/compare_012/q2e3.yaml`, a_hall), c_moving, d_robot에서
  칸마다 `GZ_REPEATS`판(기본 10). `SRC`, `B` 기본은 `~/nf_src/pd-rl-base`, `~/gg_build/pd-rl-base`, `BENCH_DOMAIN` 기본 72.
  설치된 플러그인에 `ema_alpha_w`가 없으면 멈춘다(세 RL 칸이 같아지므로). 지난 acd3 비교(120구간)가 82분이었으므로 이번
  150구간은 약 2시간으로 본다[추측].
- `gz_results.py <OUT> [dest]`: 칸마다 성공, 충돌 판, 시간 초과, 성공 판 평균 시간(SD), 분당 w 부호 바뀜(SD), 명령 w의 RMS 각가속도
  (rad/s²)와 RMS 각저크(rad/s³), 몸 여유 최솟값과 판별 최솟값의 중앙값, 명령 간격 p95. 쌍(평활 − 평활 없음, RL − RPP)마다 성공은
  Fisher p, 나머지는 Welch (차이, SE).
  - 각가속도·각저크는 `/cmd_vel` 명령을 20 Hz 계열로 보고 이웃 차분을 0.05 s로 나눈다. 이웃 간격이 0.075 s를 넘으면 계열을 끊어
    그 사이 차분은 세지 않는다. 받은 시각(시뮬 시각)의 흔들림을 나눗셈에 넣지 않으려고 명목 dt를 썼다. 지난 acd3 판에서 명령 간격
    p95는 60 ms였다.
  - `test_gz_results.py`(Windows pytest 5개 통과): 기울기, 부호가 번갈아 바뀌는 계열의 닫힌 식, 사인의 이산 미분 진폭, 간격에서
    끊기, 짧은 계열.
  - 지난 acd3 판 기록(`~/rl_gz_check/acd3_1125`, 칸마다 20판)에 돌려 보았다: acd3+shim의 분당 부호 바뀜이 지난 보고서 값과 같고
    (Q2 → E-3 41.462, c_moving 33.75), RMS 각가속도 1.49 / 1.356 rad/s², RMS 각저크 27.4 / 22.3 rad/s³였다. 기준선 참고용이다.
- `bash -n`: `gz_run.sh`, `tools/nav_bench/run_bench.sh` 통과. `run_bench.sh`에는 dry-run이나 목록 모드가 없어서 실행하지 않았다.

## 6. 학습 쪽은 따로

행동 변화율 벌점 가중치나 CAPS식 정규화(시간·공간 평활 손실)처럼 정책 자체를 덜 흔들리게 학습시키는 방법은 별도 실험이다.
이 보고서의 평활은 학습된 정책을 그대로 두고 배치에서만 거는 것이다.

## Gazebo 결과

(비어 있음. 오케스트레이터가 `gz_run.sh`를 돌린 뒤 채운다.)
