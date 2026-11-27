# 시연용 사람 장애물: 통로를 건너는 사람과 mover 고르기

브랜치 `feat/rl-demo-person-2026-11-26`. Isaac 과 Gazebo 는 돌리지 않았다. 아래 내용은 모두 시뮬레이터 없는 단위 테스트로 확인했다.

## 바꾼 것

- `ArenaSpec.mover_cross_prob` (기본 0.0): 이 확률로 사람(`person`) 구간이 건넘 구간이 된다. 수레는 바뀌지 않는다.
- `ArenaSpec.mover_kinds` (기본 `""`): 빈 값이면 지금처럼 모든 mover 가 나온다. 쉼표 목록(예 `person`)을 주면 목록에 있는 mover 만 나온다. 모르는 이름이면 `ValueError` 가 난다.
- `mover_spec.py`: `find_openings`, `crossing_candidates`, `leg_valid`, `leg_on_path`, `kind_on`, `finish_legs` 함수를 새로 넣었다. `sample_legs` 는 맨 끝에서 `finish_legs` 를 부른다.
- `crossing_eval.py` (새 모듈): 에피소드 표시(`episode_flags`)와 표시별 결과 비율(`rates_by_cross`).
- `store_arena_env.py`: 초기화할 때 선반 틈을 찾아 `sample_legs` 에 넘기고, 리셋할 때 `last_ep_cross` (n, 2) 를 남긴다.
- `play.py`: `episodes.npz` 에 `cross`, `cross_path` 를 넣고, `eval.json` 에 `by_crossing` 을 넣는다. `by_crossing` 은 `cross_on_path`, `cross_off_path`, `no_cross` 세 묶음의 에피소드 수와 결과 비율이다.

## 건넘 구간

- 틈 찾기 (`find_openings`): 긴 변 방향이 같은 두 선반이 한 줄에 서 있으면(두께 범위가 겹치면) 긴 변 방향으로 벌어진 사이를 틈으로 본다. 틈 폭은 0.15~0.50 m 이다(`OPEN_WIDTH_M`). 다음 경우는 틈에서 뺀다.
  - 벽에 닿는 선반도 쓴다. v7 충전 칸막이는 긴 변이 벽에 수직이고 서로 한 줄이 아니라서 틈이 되지 않는다.
  - 틈 안에 다른 선반이 있는 쌍
  - 틈 양쪽 통로 중 한쪽이라도 0.15 m 보다 얕은 경우
  - 통로 끝은 틈 폭 범위 안에서 가장 가까운 선반 면이나 벽이다.
  - 코드에 좌표는 없다. v7 에서는 남쪽 두 진열대 사이 틈 하나(x 1.014~1.264, 폭 0.25)만 찾는다. 남쪽 통로 끝은 벽 y 0, 가운데 통로 끝은 진열대 A y 0.754 이다.
- 구간 (`crossing_candidates`): 후보마다 틈 하나를 고른다.
  - 틈 방향 좌표: 양끝 모두 [g0 + r + 0.02, g1 - r - 0.02] 에서 따로 뽑는다(r = 0.04, 여유 `OPEN_MARGIN_M` 0.02). 직선 구간 전체가 이 범위 안이므로 사람은 선반 끝에서 0.02 m 이상 떨어져 틈을 지난다. 구간이 조금 비스듬할 수 있다.
  - 건너는 좌표: 한 끝은 [아래 통로 끝 + r + 0.02, 아래 통로 중심선], 다른 끝은 [위 통로 중심선, 위 통로 끝 - r - 0.02] 에서 뽑는다. 그래서 양쪽 통로의 중심선까지는 반드시 간다. 벽 여유 r + 0.02 는 기존 `leg_clearance > r + 0.01` 검사를 통과하는 값이다.
- 유효 규칙 (`leg_valid`): 기존 `sample_legs` 와 같다. 벽/선반/박스 여유, 범위 안, 구간 전체가 시작/목표에서 `mover_node_clear_m`, 양끝이 경로에서 `mover_end_path_clear_m` 떨어져야 한다. 후보 256개 중 첫 유효 후보를 쓰고, 없으면 그 에피소드에 사람은 나오지 않는다. 지금 구간과 같은 방식이다.
- 다른 설정과의 관계
  - 건넘 에피소드에서는 `mover_on_path_prob` 와 `mover_leg_m` 를 쓰지 않는다. 위치는 틈이 정하고, 길이는 두 통로 중심선 사이 이상이다(v7 에서 약 0.40~0.67 m).
  - 속도, 양끝 정지, 시작 위치(phase), 양보(`mover_yield_m`, `mover_yield_mix`, `mover_wait_s` 뒤 돌아서기)는 보통 구간과 같다. 그래서 로봇이 가까이 오면 사람이 틈 안이나 통로에서 기다릴 수 있고, `mover_wait_s` 뒤에는 돌아선다.
  - `mover_kinds` 에서 사람을 빼면 건넘 구간도 나오지 않는다(active 가 꺼짐).
- 경로 표시 (`leg_on_path`): 구간 위 사람 몸(반지름 0.04)이 우리 전역 경로를 덮으면 `cross_path` 다. 경로점이 아니라 이웃 경로점을 잇는 선분과 구간 사이 거리로 계산한다(선분끼리 가로지르면 0). 양끝은 경로에서 떨어져 있으므로 덮는 곳은 구간 중간이다.
- 틈이 하나도 없는 배치에서 `mover_cross_prob > 0` 을 주면 `ValueError` 가 난다. 조용히 건넘 없이 돌지 않게 하려는 것이다.

## 기본값에서 달라지지 않음

- `finish_legs` 는 `sample_legs` 의 기존 난수(활성, 속도, 정지, phase)를 다 뽑은 뒤에 불린다. `mover_cross_prob` 가 0 이거나 사람이 아니면 난수를 더 뽑지 않는다.
- 기존 코드에서 바뀐 것은 두 가지뿐이다. `rand(n) < mover_prob` 를 변수 `shows` 로 뺐고(뽑는 순서는 같음), 결과 dict 에 bool 키 `cross`, `cross_path` 를 더했다(기본값에서는 모두 False).
- `mover_kinds` 는 난수를 그대로 뽑고 active 만 끈다. 그래서 `mover_kinds=person` 실행의 사람 구간은 기본 실행과 같다.
- env 의 mover 텐서 모양, `mover_state`, 관측, 보상은 바뀌지 않았다. 새 텐서는 `mover_openings` 와 `last_ep_cross` 이고, 둘 다 기록에만 쓴다.
- 테스트 `test_defaults_give_the_legs_and_draws_of_before` 가 이것을 확인한다. 이 변경 전 `sample_legs` 를 테스트 파일에 그대로 복사해 두고, 시드 0과 1, 두 mover, openings 를 넘긴 경우와 안 넘긴 경우에 대해 비교한다. 모든 출력이 `torch.equal` 로 같아야 하고, 끝난 뒤 생성기 상태(`get_state`)도 같아야 한다.
- Isaac env 전체(`_reset_idx` 의 순서)는 돌려 보지 않았다. env 쪽 변경은 기록용 줄과 `openings=` 인자뿐이다.

## 테스트 (`learning/store_arena/test_mover_cross.py`, 13개)

| 성질 | 테스트 |
|---|---|
| v7 에서 남쪽 진열대 틈 0.25 m 하나 | `test_v7_has_one_opening_the_south_shelf_gap` |
| 세로 줄 틈, 벽 칸막이 제외, 폭 0.6 제외, 위아래로 쌓인 선반(통로) 제외, 얕은 통로 제외, 한 끝이 벽에 닿은 선반의 틈은 찾음 | `test_opening_rules_on_made_up_shelves` |
| 틈 통과와 0.02 여유, 양 통로 중심선 도달, 벽/선반/박스 여유(200 점), 시작/목표 간격, 끝점-경로 간격 | `test_crossing_legs_go_through_the_opening_from_aisle_centre_to_aisle_centre` |
| 가림: 옆 통로 중심선 위 점(틈 중심에서 옆으로 0.35 m)에서 사람까지 직선이 구간 일부에서는 선반에 막히고 일부에서는 열림. 양쪽 통로에서 모두, 유효 구간 40개 | `test_a_shelf_hides_the_crossing_person_for_part_of_the_leg` |
| 경로 표시: 손으로 만든 경우(경로점 둘 사이에서만 몸에 닿는 경우 포함), 그리고 v7 에서 400 점 조밀 검사와 99% 이상 일치하며 참/거짓이 둘 다 나옴 | `test_crossing_flag_for_our_path`, `test_crossing_flag_matches_a_dense_check_on_v7` |
| 수레는 건넘을 켜도 그대로. 사람의 속도/정지/phase 도 그대로이고 건넘 비율은 약 p. 틈이 없으면 오류 | `test_crossing_only_for_the_person_and_appended_draws` |
| mover_kinds: 난수 상태가 같고, 남은 mover 는 그대로, 나머지는 비활성. 모르는 이름은 오류 | `test_mover_kinds_turns_off_the_others_without_changing_draws` |
| 기본값 불변 (위 절) | `test_defaults_give_the_legs_and_draws_of_before` (4개) |
| 에피소드 표시와 표시별 비율 | `test_episode_flags_and_rates_split` |

- 측정값: v7, `obstacle_layouts=4`, 시드 0, `mover_cross_prob=1`, `mover_prob=1` 에서 경로 176개 중 유효한 건넘 구간이 있는 비율은 0.70 이다.
- 나머지 0.30 은 사람이 나오지 않는다. 원인은 두 가지다. 박스가 틈이나 통로 끝을 막는 경우(벽/박스 여유를 통과하는 후보가 있는 비율 0.84), 우리 경로가 틈을 지나서 끝점이 경로 옆에 놓일 수 없는 경우(끝점-경로 규칙을 통과하는 후보가 있는 비율 0.85)다.
- 테스트의 하한은 0.6 이다. 새 테스트라서 기존 기준을 낮춘 것은 없다.

## 실행 줄

평가 (시연 칸: 동적 장애물은 건너는 사람 하나):

```
--set mover_kinds=person --set mover_prob=1.0 --set mover_cross_prob=1.0
```

학습 (지금 구간과 건넘 구간을 반씩, 수레도 남김):

```
--set mover_cross_prob=0.5
```

시연과 똑같이 사람만 두고 학습하려면 `--set mover_kinds=person` 을 더한다. 플릿 로봇 수, 정적 장애물 배치는 이 작업 범위가 아니다(다른 브랜치).

## 결과

(21:34에 채움.) 건너는 사람 설정은 demo1, demo2 칸과 시연 조건 학습에 쓰였다. 건넘이 경로 위인 에피소드의 사람 충돌은 작업 기록 12, 15, 21, 38절과 `docs/rl_tracks_2026-11-26/results/crossing.csv`에 있다. 어느 정책도 사람 충돌을 없애지 못했고(demo1에서 acd3 20.5 %, t_demo_hist 15.3 %), 거울 증강을 합친 t_demo_hist_sym도 이 지표가 줄지 않았다(38절).
