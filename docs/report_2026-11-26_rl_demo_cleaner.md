# RL 시연 준비: 청소 로봇 칸과 로봇 수 고정 (2026-11-26)

브랜치 `feat/rl-demo-cleaner-2026-11-26`, 출발점 `9a09378a`. Isaac과 Gazebo는 돌리지 않았다. 확인은 시뮬 없는 단위 시험뿐이다.

## 1. 바꾼 것

- `learning/store_arena/cleaner_spec.py`(새 파일): 청소 브랜치(`origin/feat/cleaner-yield-2026-11-25`)의 `pc/fleet_core/fleet_core/coverage.py` 계산(청소 격자, 셀 나누기, 줄 고르기, A* 이동, 모서리 둥글게)을 그대로 옮기고, 실제 다리 순서로 한 바퀴를 잇는 `clean_lap`, 바퀴를 이어 경로 dict로 까는 `lap_table`/`lap_route`, env 연결용 `pad_route`, `mix_route`, `start_clock`, `cleaner_active`, `watch_xy`를 더했다.
- `learning/store_arena/arena_spec.py`: `robot_scenario` 바로 아래 한 덩어리로 `robot_cleaner_prob`(0.0), `robot_min`(0), `robot_cleaner_yield_m`(0.30), `robot_cleaner_wait_s`(10.0).
- `learning/store_arena/store_arena_env.py`: 기존 함수 안은 몇 줄만 고쳤다(`__init__`에서 `_cleaner_init` 호출, `_sample_robots`의 로봇 수 `randint` 하한, `_cleaner_pick`/`_cleaner_route`/`_cleaner_reset` 호출, `_pre_physics_step`의 `robot_step` 인자 `watch`/`wait_s`). 나머지는 새 메서드 `_cleaner_*` 다섯 개다.
- `learning/store_arena/test_cleaner_spec.py`: 시험 14개.

## 2. 동작

- 켜는 법: `robot_cleaner_prob > 0`. 에피소드마다 이 확률로 **마지막 로봇 칸(`robot_max - 1`)**이 청소 로봇이 된다. 그 env의 로봇 수는 최소 1(robot_headon, robot_scenario, ghost_pairs 중 하나가 켜져 있으면 최소 2)로 올리고, 그래프 로봇은 앞 칸 `count - 1`대다.
- `robot_min`: 로봇 수를 `robot_min..robot_max` 균등으로 뽑는다. `robot_min=2 robot_max=2 robot_cleaner_prob=1`이면 칸 0은 HALL 그래프 운반 로봇, 칸 1은 청소 로봇이다.
- 바퀴: P-1 → E-1 → 출발점(0.1125, 0.2625) → 줄마다 (지금 자리에서 줄 시작까지 이동 + 줄) → 출발점 → E-1 → P-1(후진, 진행 방향의 반대를 봄). 각 다리는 회전 반경 0.10 m로 둥글게 하고, 꼭짓점은 0.05 m 간격 이하로 촘촘히 했다. 바퀴는 에피소드 동안 끊기지 않게 여러 번 이어 붙인다.
- 지금 지도(`fleet_map_v7.yaml`, 11-25 줄자 판)에서: 줄 20개, 셀 11개, coverage.py 요약 lap_time 136.2 s, 시뮬 한 바퀴 16.04 m = 0.12 m/s로 133.7 s(나가기·들어가기를 넣고, 이동과 줄의 이음 모서리를 둥글게 해서 계획 값과 다르다).
- 청소 브랜치의 옛 v7 지도로 돌리면 줄 17개, 116.8 s로 그 브랜치의 `coverage_v7.yaml` 요약과 같다(시험 `test_the_branch_map_gives_17_rows_and_116_8_s`). 앞선 메모의 "17줄, 116.8 s"는 옛 지도 값이다.
- 속도: `coverage.py`의 `SPEED_MPS = 0.12`. 속도 상한이 바뀌는 학습(`speed_cap_range`)에서는 다른 플릿 로봇처럼 `v_cap / 0.12` 배율을 곱한다.
- 출발: 바퀴 위 무작위 시각. 후보 8개를 뽑아 우리 시작점과 켜진 운반 로봇(ghost_pairs면 칸 0은 같이 리셋되는 짝의 시작점, 아니면 짝의 지금 자세)에서 `scenario_spec.START_SEP_M`(0.30 m) 이상 떨어진 첫 후보, 없으면 가장 먼 후보.
- 비키기: 기존 `robot_step`을 그대로 쓴다. 다음 스텝에 상대와 `robot_cleaner_yield_m` 안으로 가까워지면 서고, `robot_cleaner_wait_s` 동안 서 있으면 `robot_retreat`대로 경로를 따라 물러난다(꼭짓점이 0.05 m 간격이라 5~10 cm). 상대는 우리 로봇과 켜진 운반 로봇 중 가장 가까운 것이다. 청소 로봇은 우선권을 갖지 않는다.
- intent 관측: 청소 로봇의 intent 점은 지금 다리(이동 + 줄)의 다음 점이고 다리 끝에서 멈춘다. 즉 지금 줄의 다음 점들이다. 다리 이음 꼭짓점은 두 번 두어(길이 0 구간) 새 다리에 들어서자마자 intent가 새 다리 끝까지 간다.
- 노드 예약: 청소 경로의 꼭짓점 노드는 -1이다. 청소 로봇은 노드를 쥐지 않고, 남이 쥔 노드에도 막히지 않는다.

## 3. 실제 청소 로봇과 비교

| 항목 | 실제 (`coverage.py`, `traffic/zone/cleaning.py`, `fleet_params.yaml`) | 시뮬 청소 칸 |
|---|---|---|
| 줄 만들기 | 격자 0.025 m, 몸 반지름 0.083 + 여유 0.01, HALL만 바닥, 셀마다 0.10 m 이하 간격 | 같음(코드를 옮김, 실제 `make_plan`과 줄·이동 꺾은선이 같음을 시험으로 확인) |
| 청소 폭 | 0.112 m | 쓰지 않음(덮은 비율을 재지 않음) |
| 바퀴 순서 | bay_out_1 → 출발점 → 줄(이동 + 줄) → 출발점 → E-1 → bay_in_1, 끝나면 바로 다음 바퀴 | 같은 순서, 바퀴를 이어 붙임 |
| 모서리 | 다리마다 회전 반경 0.10 m로 둥글게 | 같은 `rounded`로 다리마다 |
| 속도 | 0.12 m/s | 0.12 m/s(상한 배율 곱함). 다리 끝 도착 대기와 goal 보내는 틈은 없음 |
| 줄 미루기·건너뛰기 | 막히면 미루고(`clean_retry_s`), 3번 넘으면 그 바퀴에서 포기, 사람·카트 우회 | 없음. 늘 계획 순서대로 감 |
| 만남 판단 | 운반 로봇 띠(`clean_yield_band_m` 0.30) 안, 다리가 띠에서 0.18 안, 다가오는 운반 로봇 0.30 안, 쥔 노드를 기다림 | 상대(우리 또는 가장 가까운 운반 로봇)와 다음 스텝 거리가 0.30 안에서 줄어들 때만 |
| 비키기 | 띠 밖 빈칸까지 A* 경로로 옮겨 섬, 진행점을 남기고 이어 함 | 그 자리에 섬(시계 멈춤), 진행점은 그대로 |
| 오래 막힘 | 처음 만난 뒤 `clean_yield_wait_s`(10 s) 지나도 만나면 집 자리 P-1로 피난, 10 s 쉼, `clean_stuck_s`(5 s) 넘게 못 가면 CLEAN_STUCK | 10 s 서 있으면 경로를 따라 5~10 cm 물러나고 다시 감. 집 자리 피난과 STUCK 없음 |
| 노드 쥐기 | 청소 경로 둘레 `clean_reach_m` 0.18 안 지도 지점을 쥠 | 쥐지 않음. 운반 로봇의 노드 예약에 안 보임 |
| 운반 로봇이 청소 로봇을 피함 | 운반 로봇은 쥔 노드로 기다리고, 막히면 청소 로봇에 비키라고 함 | 운반 로봇(그래프)은 청소 로봇을 보지 않음(우리에게만 비킴). 두 스크립트 로봇이 겹칠 수 있음 |
| 시작 | P-1에서 바퀴 시작 | 바퀴 위 무작위 자리 |

## 4. 켜는 --set

평가(운반 1 + 청소 1 고정):

```
--set robot_max=2 --set robot_min=2 --set robot_cleaner_prob=1
```

학습(지금 학습 줄에 더함, 로봇 0..2대 중 절반 판에 청소 로봇):

```
--set robot_max=2 --set robot_cleaner_prob=0.5
```

`robot_max=2 robot_cleaner_prob=0.5`이면 청소 판의 로봇 수는 1 또는 2(0이 1로 올라감)이고, 청소가 아닌 판은 지금처럼 0..2 균등이다. play.py는 고치지 않아 eval.json에 청소 판 여부가 남지 않는다.

## 5. 기본값이 그대로인 근거

- `robot_cleaner_prob=0`이면 `_cleaner_init`이 경로 저장소를 건드리지 않고(`self.cleaner = None`), `_cleaner_pick`이 난수 없이 `(count, None)`을 돌려주며, `_cleaner_route`, `_cleaner_reset`, `_cleaner_yield`는 불리지 않는다. `robot_step`에는 전과 같은 `xy`와 `robot_wait_s * wait_scale()`가 간다.
- `robot_min=0`이면 `torch.randint(0, nk + 1, (n,))`로 전과 같은 호출이다.
- 시험 `test_default_is_unchanged_draws_and_routes`: `9a09378a`의 `_sample_robots` 소스와 새 소스를 같은 가짜 env(로봇 2대, 256 env)에 붙여 같은 seed로 돌리고, 활성 칸·시계·모든 경로 텐서(모양 포함)와 그 뒤 난수 4개가 같음을 확인했다.
- 관측과 크리틱 텐서 모양은 바꾸지 않았다. 청소를 켜면 마지막 칸의 경로 꼭짓점 수만 바퀴 길이로 늘어난다(청소가 아닌 env는 끝 꼭짓점을 되풀이해 같은 움직임, 시험 `test_cleaner_prob_mixes_slots`).

## 6. 시험

- `python -m pytest learning -q -p no:cacheprovider` 마지막 줄: `268 passed, 5 skipped in 31.92s`.
- 새 시험 파일을 빼고 돌리면 `252 passed, 5 skipped`다(`--ignore=learning/store_arena/test_cleaner_spec.py`). 지시문의 "변경 전 259 passed"와 7개가 다른데, 이 worktree의 출발점 `9a09378a`에서 센 값은 252다. 차이의 까닭은 모른다.
- `test_cleaner_spec.py` 16개: 실제 `coverage.py`(청소 브랜치에서 git으로 읽어 임시 모듈로 올림)와 줄·이동 꺾은선·출발점이 같음, 옛 지도에서 17줄 116.8 s, 줄 성질(x 방향, 몸 반지름 + 여유 이상, 셀 안 간격 0.10 m 이하), 바퀴 순서, 바퀴 시간, intent(줄 위, 다리 이음에서 새 다리 끝으로 바뀜), 출발 시각, 대기 10 s 뒤 조금 물러남, 가장 가까운 상대, 기본값 그대로(옛 소스와 비교), `robot_min`, 운반 1 + 청소 1, 섞인 칸, ghost 칸 피하기, 칸이 모자랄 때 오류.
- 청소 브랜치(`origin/feat/cleaner-yield-2026-11-25`)를 git에서 못 읽으면 실제 `coverage.py`와 비교하는 두 시험은 건너뛴다.
- `ruff`: 새 파일에 남은 지적은 `coverage.py`에서 그대로 옮긴 줄(줄 길이 2, B007 1, B008 1)과 그 꼴을 따른 `clean_lap` 기본 인자(B008)다. 옮긴 코드는 실제와 비교하기 쉽게 고치지 않았다.

## 7. 못 한 것과 정할 것

- Isaac에서 돌려 보지 않았다. env 연결은 메서드 소스를 가짜 env에 붙인 시험으로만 확인했다.
- 운반 로봇(그래프)이 청소 로봇을 피하지 않는다. 실제처럼 하려면 그래프 로봇의 `robot_step`에도 청소 로봇을 상대로 넣어야 하는데 지금 학습의 로봇 움직임이 바뀌므로 하지 않았다.
- 청소 로봇이 다른 로봇과 겹쳐 시작할 수는 있다(후보 8개가 모두 0.30 m 안이면 가장 먼 후보).
- `robot_cleaner_wait_s`는 실제 시간(10 s)이라 `wait_scale()`을 곱하지 않았다.
- 줄 미루기, 집 자리 피난, 노드 쥐기는 옮기지 않았다(3절 표).

## 8. 검수 (Codex, `9a09378a..006d24ab`)

- ghost_pairs에서 칸 0의 그래프 위치를 피하던 것: 고쳤다(칸 0은 짝의 시작점을 피함). 고치기 전 실패하는 시험을 먼저 썼다.
- 다리 이음에서 intent가 앞 다리 끝에서 잘리던 것(이음 뒤 첫 0.05 m 구간): 고쳤다. 시험도 고치기 전 실패함을 확인했다.
- 재검수(`006d24ab..8391d9f8`): 짝이 같이 리셋되지 않으면 ghost는 짝의 지금 자세에 놓이는데 시작점을 피하던 것을 고쳤다(env의 ghost 배치와 같은 규칙). 이음 수정에는 결함이 없다고 했다.
- 후보 8개가 모두 0.30 m 안이면 가장 먼 후보로 시작하는 것: 고치지 않았다(7절에 적음). 시험의 90 %/95 % 기준은 이 동작을 반영한 것이다.

## 결과

(비어 있음: Isaac 평가와 학습을 돌린 뒤 채운다)
