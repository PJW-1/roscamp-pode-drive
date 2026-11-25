# 2026-11-26 RL 의도 관측 다시 보기: 평가 시나리오, 보상 항, 자기 대국

브랜치 `feat/rl-intent2-2026-11-25`. Isaac은 돌리지 않았다. 아래 숫자는 모두 시뮬레이터 없는 계산(맵 기하, 단위 시험)에서 나왔고,
학습·평가 결과는 아직 없다("학습 결과" 절은 비워 두었다).

## 1. 가설과 판정 규칙 (학습 결과를 보기 전에 적음)

**가설.** 11-23의 의도 관측이 성공을 못 올린 까닭은 평가 세트에 의도가 라이다보다 더 알려 주는 장면이 드물었기 때문이다.
라이다에 늦게 보이는 만남(선반 뒤, 옆에서 건넘, 좁은 통로 마주침)에서는 의도가 도움이 될 것이다.

**판정 규칙.** 새 세 시나리오(corner, tee, entry) 중 적어도 하나에서, 의도 또는 의도 + 우선순위 변형이 의도 없음 변형보다
교착 수 또는 충돌 수를 2 표준오차 넘게 줄이면 가설을 받아들인다.

- 비교는 같은 에피소드끼리 한다: 세 변형 모두 같은 평가 seed(7), 같은 `--num_envs`(4096), env마다 첫 에피소드. env 번호로 짝짓는다.
- 변형마다 학습 seed 셋(1, 2, 3)을 쓴다. 차이는 seed별 짝지은 차이(의도 변형 − 의도 없음, 같은 seed끼리)의 평균이고, 표준오차는
  seed 세 차이의 표준편차 ÷ √3이다(11-23 6번과 같은 셈). 에피소드 수준의 짝지은 표준오차도 같이 적는다.
- 지표: 충돌률(`rates.collision`)과 교착 수(`robot_encounter.deadlocks`), 시나리오별로 `by_scenario.<이름>`.
- 세 시나리오 × 두 변형 × 두 지표로 비교가 12개다. 여러 번 비교하는 것을 보정하지 않았으므로, 한 칸만 넘으면 "약한 근거"로 적고
  다른 seed로 다시 확인한다.
- 무작위 세트와 정면 세트(`robot_headon=1`)에서 성공이 1 %p 넘게 떨어지면 같이 적는다(대가).

## 2. 평가 시나리오 (1단계, 커밋 d878f9d6)

`--set robot_scenario=corner` (또는 `tee`, `entry`, 쉼표로 여럿). 기본값 `""`이면 동작과 난수 순서가 이전과 같다.
켜면 모든 env가 그 시나리오의 후보 하나로 시작한다(시나리오는 고르게, 후보도 고르게). `robot_headon`과 같이 쓸 수 없고
`robot_max >= 1`이 필요하다.

**후보 고르기(`learning/store_arena/scenario_spec.py`).** 좌표를 손으로 적지 않는다. 우리 전역 경로(박스 없는 배치 0, 44쌍)와
로봇 HALL 그래프 경로(7노드, a → b 42개)의 모든 조합에서:

1. 두 경로가 만나는 점을 찾는다: 우리 경로 점이 로봇 경로에서 `TOUCH_M` 0.03 m 안. tee·corner는 첫 만남점, entry는 마주 오는 구간 중
   좌우 빈 폭이 가장 좁은 점.
2. 로봇 0의 출발 호장 s0 = (로봇 경로의 만남점 호장) − (우리 경로의 만남점 호장). 둘 다 같은 속도로 달리면 만남점에 같이 닿는다.
   s0 < 0이면(로봇 경로가 더 짧음) 뺀다.
3. 공통 조건: 우리 만남점까지 0.35 m 이상(`MIN_MEET_M`), 두 시작점 중심 0.30 m 이상(`START_SEP_M`), 로봇 시작점이 벽·선반에서 0.09 m 이상.
4. 시나리오 조건:
   - **corner:** 우리 중심에서 로봇 중심과 로봇 박스 네 모서리로 가는 선분이 출발 때 선반에 막히고, 같이 달리는 동안(1 cm 걸음)
     중심 거리가 `CLOSE_M` 0.60 m 아래가 될 때까지 계속 막힌다.
   - **tee:** 만남점에서 두 진행 방향의 사잇각이 60~120도.
   - **entry:** 만남점에서 진행 방향 cos < −0.7(마주 옴)이고, 만남점의 좌우 빈 폭(진행 방향 ±90도 레이 합)이 0.40 m 미만.

**완화한 기준.** corner의 "가까워질 때까지 막힘"은 0.60 m로 잡았다. 처음 0.30 m로 했더니 후보가 하나도 없었다. 이 아레나의
선반은 4 cm 두께라 로봇이 중심 거리 0.44~0.58 m에서 보이기 시작한다(후보마다 잰 값). 즉 corner에서 의도가 라이다보다 앞서는
것은 출발부터 그 거리까지, 같이 최고속도로 달리면 약 0.2~2.3 s다. 이 맵에서 더 길게 가릴 수는 없다.

**entry의 해석.** 남쪽 두 선반 사이 0.25 m 틈은 로봇 그래프 경로가 지나가지 않는다(GAP_S는 via 노드라 그래프에서 빠짐). 그래서 entry는
"통로 입구"를 택했다: 가운데 통로(0.35 m)와 남쪽 통로(약 0.36 m) 안에서 마주 오며 같이 닿는 배치다. 정면(`robot_headon`)과의 차이는
로봇이 우리 목표가 아니라 통로 반대편에서, 같은 시각에 통로 안 한 점에 닿도록 출발한다는 점이다.

**후보 수(v7 맵).** corner 24개(시작/목표 쌍 9), tee 32개(쌍 21), entry 37개(쌍 9).

**시나리오 에피소드에서 바뀌는 것.** 우리 경로는 배치 0(박스 없음), 우리 yaw는 경로 방향(코너·인계 시작 없음), 로봇 0은 시나리오
그래프 경로 a → b를 최고속도(× 에피소드 속도 상한 / 0.12)로 호장 s0부터 달리고 b 뒤는 임의 경로다. 로봇 수는 1 이상(로봇 1은 평소처럼
임의, 로봇 0 시작점 0.30 m 안 노드에서는 출발하지 않음). mover와 로봇 우선권은 설정 그대로다.
로봇 0은 스크립트 규칙(양보 정지, 노드 예약)을 그대로 따르므로, 정책이 느리거나 노드를 잡으면 만남 시각이 어긋날 수 있다.
"같이 닿음"은 둘 다 최고속도로 막힘 없이 달릴 때의 성질이다.

**시험(`test_scenario_spec.py`, 15개).** 시나리오마다 시작/목표 쌍 수 하한(corner 6, tee 12, entry 6), 만남점까지 두 경로 길이 차
< 0.05 m(`project`로 다시 잼), 로봇 시작점이 빈 곳, corner 시작 선분이 선반에 막힘(`arena_spec.raycast_rects`로 직접 잼), 같이 달리는
동안 가려짐, tee 사잇각, entry 마주옴과 폭, env와 같은 방식(`chain_route`, t0 = s0 / 속도)으로 깐 로봇이 우리 만남 시각에 만남점에서
0.05 m 안에 있음. corner의 진행 중 가려짐과 entry 폭 시험은 후보를 고를 때 쓴 함수(`robot_hidden`, `free_width`)를 다시 부르므로
그 함수 자체의 오류는 못 잡는다. 가려짐은 1 cm 걸음 표본에서만 확인했다.

**평가 지표(`learning/store_arena/encounter.py`, `play.py`).** 기준은 `encounter.py` 한 곳에 있다.

- `ENCOUNTER_M` 0.15 m: 우리 몸체 박스와 로봇 박스 사이 간격(`box_clearance`, 겹치면 0).
- `STILL_MPS` 0.02 m/s, `DEADLOCK_S` 5.0 s.
- 에피소드별 최소 간격(로봇이 없던 에피소드는 뺌), 첫 만남 시간(간격이 처음 0.15 m 아래로 떨어진 때부터 다시 넘거나 에피소드가 끝날 때까지),
  교착(우리와 같은 로봇이 둘 다 0.02 m/s보다 느리고 간격 0.15 m 아래인 상태가 5 s 넘게 이어짐). 속력은 둘 다 스텝당 위치 변화로 잰다.
- `eval.json`의 `robot_encounter`(전체)와 `by_scenario.<이름>`(결과 비율 + 같은 지표), `episodes.npz`의 `enc_min_gap`, `enc_enc_s`,
  `enc_enc_seen`, `enc_deadlock`, `scenario`(env 짝짓기용).
- 리셋 직후 시작 자세의 간격은 최소 간격에 넣지 않는다(첫 스텝 뒤부터 잼). 시나리오 시작 간격은 0.30 m 이상이라 결과에 영향이 없다.

## 3. 보상 항 (2단계, 커밋 f46825b7)

모두 `ArenaSpec` 필드이고 `--set`으로 바꾼다. 기본 0이면 꺼지고 보상과 로그가 이전과 같다. 켠 항은 `Episode_Reward/near_dyn`,
`Episode_Reward/yield`, `Episode_Reward/block`으로 에피소드 합이 기록된다. 셋 다 제어 스텝마다 더하며 성공·충돌 보상은 건드리지 않는다.
함수는 `learning/store_arena/social_reward.py`, 시험은 `test_social_reward.py`(8개).

| 항 | 정의(스텝당) | 첫 제안값 | 까닭 |
|---|---|---|---|
| (a) 불편 거리 | mover나 로봇과 몸체 간격 g < `near_dyn_m`이면 `w_near_dyn` × (near_dyn_m − g) / near_dyn_m | `near_dyn_m=0.10`, `w_near_dyn=-0.2` | 벽 벌점(0.08 m, −0.2)과 같은 크기. 0.35 m 통로에서 두 로봇이 비껴 가면 몸체 간격이 최대 약 0.14 m라 0.10이면 비껴 가기는 벌받지 않는다 |
| (b) 양보 | 우선 로봇이 몸체 간격 `yield_m` 안에서 다가오고, 그런 로봇 모두에 대해 우리가 간격을 지키면 `w_yield` | `yield_m=0.15`, `w_yield=0.05` | 기다리는 동안 시간 벌점(−0.05)만 지운다. 최고속도 진행 보상(+1)보다 훨씬 작아 일부러 다가오는 로봇 곁에 머물 이득이 없다 |
| (c) 서로 막힘 | 어떤 로봇과 몸체 간격 < `block_m`이고 둘 다 `block_v_mps`보다 느리면 `w_block` | `block_m=0.10`, `w_block=-0.05`, `block_v_mps=0.02` | 시간 벌점을 두 배로 만든다. 5 s 막히면 −5로 충돌(−10)의 절반이라 충돌보다 막힘을 고르게 되지 않는다 |

(b)의 정확한 정의(중심 거리, 이번 스텝 전후 위치, ε = `STEP_EPS_M` 0.001 m):

- 로봇 k가 **다가옴**: 로봇 자신의 이동이 우리 직전 위치와의 거리를 ε 넘게 줄임. |r_k(t) − x(t−1)| < |r_k(t−1) − x(t−1)| − ε.
- 우리가 **간격을 지킴**: 우리 이동이 로봇 현재 위치와의 거리를 ε 넘게 줄이지 않음. |r_k(t) − x(t)| ≥ |r_k(t) − x(t−1)| − ε.
  서 있기, 물러나기, 옆으로 비키기가 모두 들어간다.
- 우선 로봇의 플래그는 intent 관측의 우선권과 같은 `robot_priority`다.

## 4. 자기 대국 (3단계)

(작성 중)

## 5. 실행 줄

공통 레시피(44입력 a+c+d), 아래에서 `R`:

```
--set prev_action_obs=1 --set critic_privileged=1 --set speed_cap_obs=1 --set speed_cap_range=0.08,0.16 --set w_action_rate=-0.05 --set w_w_flip=-0.5 --set robot_max=2 --set robot_priority_prob=0.5 --set handover_start_prob=0.15 --set near_wall_m=0.08 --set w_near_wall=-0.2
```

변형별 관측 추가, 아래에서 `V`:

- 의도 없음: (없음)
- 의도: `--set intent_robots=2`
- 의도 + 우선순위: `--set intent_robots=2 --set intent_priority=1`

**학습(변형 × seed 1, 2, 3).** 11-23처럼 같은 시작점에서 넓혀 이어 학습한다. 의도 변형은 넓힌 체크포인트가 필요하다
(`widen_checkpoint.py <a+c+d 체크포인트> <out.pt> R V`). 11-23은 500번(250 × 2)으로 짧았다는 의심이 있으므로 같은 길이와
그 두 배(1000번) 중 하나를 오케스트레이터가 정한다.

```
INIT=<넓힌 체크포인트> bash learning/store_arena/run_train.sh /d/isaac/runs/store_arena/intent2_<변형>_s<seed> 2 250 4096 --seed <seed> R V
```

보상 항을 같이 시험하려면 `R V` 뒤에 `--set near_dyn_m=0.10 --set w_near_dyn=-0.2 --set yield_m=0.15 --set w_yield=0.05 --set block_m=0.10 --set w_block=-0.05`
를 붙인다(이것은 가설 판정과 따로 센다: 보상을 바꾸면 의도의 효과와 섞인다).

**평가(시나리오마다, 정책마다).** 평가는 0.12 m/s 고정이다. 시나리오 평가에서는 `mover_prob=0`을 권한다(사람·카트는 시나리오와
상관없이 놓여 로봇 효과를 묽힘). 같은 seed면 어느 쪽이든 모든 정책이 같은 에피소드를 본다.

```
/d/isaac/IsaacLab/.venv/Scripts/python.exe learning/store_arena/play.py --checkpoint <model.pt> --out <dir>/corner --num_envs 4096 --episodes 1 --seed 7 --headless R V --set speed_cap_range=0.12,0.12 --set robot_scenario=corner --set mover_prob=0
/d/isaac/IsaacLab/.venv/Scripts/python.exe learning/store_arena/play.py --checkpoint <model.pt> --out <dir>/tee    --num_envs 4096 --episodes 1 --seed 7 --headless R V --set speed_cap_range=0.12,0.12 --set robot_scenario=tee    --set mover_prob=0
/d/isaac/IsaacLab/.venv/Scripts/python.exe learning/store_arena/play.py --checkpoint <model.pt> --out <dir>/entry  --num_envs 4096 --episodes 1 --seed 7 --headless R V --set speed_cap_range=0.12,0.12 --set robot_scenario=entry  --set mover_prob=0
```

`R`에서 `speed_cap_range`는 뒤의 `--set`이 덮는다. 대가를 보려면 같은 줄에서 `robot_scenario`를 빼고(무작위), 또
`--set robot_headon=1`로(정면) 돌린다.

## 6. 학습 결과

(오케스트레이터가 채운다)

## 7. 열린 점

- corner의 가려짐은 이 맵에서 0.44~0.58 m까지가 한계다(2절). 더 강한 시험이 필요하면 높은 선반이 아니라 맵 자체가 달라야 한다.
- 로봇 1(`robot_max=2`)은 시나리오 에피소드에서도 임의로 나온다(로봇 수 1~2). critic 입력 크기 때문에 `robot_max`를 1로 줄일 수 없다.
- 시나리오 판정 기준(TOUCH_M, TEE_DEG, PASSAGE_M 등)은 `scenario_spec.py` 머리에 모여 있고 단위 시험으로만 확인했다. Isaac에서 로봇 0이
  실제로 만남점에 같이 닿는지는 `--still`이 아닌 정책 실행의 `rollouts.npz`로 확인해야 한다.
