# 청소 로봇 비키고 이어 하기(4-6)와 STOP(4-9) (2026-11-25)

브랜치 `feat/cleaner-yield-2026-11-25`, 출발점 `63f6ea2`(`feat/cleaner-on-main-2026-11-25`). 사용자 결정 전 후보라 main과 다른 청소 브랜치에 합치지 않는다. 헤드리스 세계와 단위 시험만 썼다(Gazebo, 실물 없음).

## 1. 결론

- **막힘이 풀렸다.** v7 운반 2 + 청소 1 기본값 판의 seed 1, 2, 5가 모두 운반 주문을 끝낸다(5/5판, 막힘 seed 없음). `test_world_cleaner.py::test_transport_orders_still_finish`가 통과한다.
- 막힘 꼴(청소 로봇이 Q1 옆 막다른 곳에서 Q1을 쥐고, 운반 로봇은 Q2에서 Q1을 기다림)은 **운반 로봇이 Q2에 닿기 전에** 청소 로봇이 비켜야만 풀린다. 운반 로봇이 이미 Q2에 서 있으면 막다른 곳에서 나가는 길이 Q2 옆 0.165 m를 지나서(두 몸이 닿는 거리) 비킬 점이 없고, 집으로 가는 길도 막혀 `CLEAN_STUCK`이 된다(시험으로 고정함, 3절).
- 운반 주문 지연은 줄었다(지연 평균/최대 8.3/16.8 s → 2.5/10.6 s, 다섯 판 모두). 대신 **청소가 줄었다**: 첫 바퀴에 끝낸 줄 5~11/17 → 3~8/17, 덮은 비율(받은 위치) 0.542 → 0.496, 계획 기준 0.49~0.52 → 0.14~0.71.
- 4-6, 4-9 모두 끝(헤드리스 기준). Gazebo에서 만남 근사를 확인하는 일(계획 8절)은 하지 않았다.
- 시험: fleet_core 전체 `1564 passed, 7 xfailed`, fake_robots 청소 시험 `11 passed`, tools 청소 시험 `16 passed`, pode_follow_wall `40 tests, 0 failures`.

## 2. 구현

### 2-1. 4-6 비키고 이어 하기 (`traffic/zone/cleaning.py`)

- **띠:** 운반 로봇의 지금 위치 → base 구간 끝들(base가 없으면 goal 끝 노드) → 그 로봇이 기다리는 노드를 이은 꺾은선(`_clean_band`). 남은 route 전체를 쓰는 띠(`full`)는 선 채 만남과 비킬 점 고르기에 쓴다.
- **만남(`_clean_meeting`, 매 걸음):** 다음 가운데 하나면 만난다.
  1. 운반 로봇이 청소 로봇이 쥔 노드를 기다린다(막힘 꼴을 바로 잡는 조건).
  2. 선 청소 로봇이 띠 반폭 + 몸 반지름(`clean_yield_band_m/2 + footprint_radius_m` = 0.235 m) 안이다.
  3. 달리는 다리의 남은 꺾은선이 띠에서 `clean_conflict_reach_m` 안이다.
  4. goal이 있는 운반 로봇이 `clean_yield_near_m` 안에서 다가온다(운반 로봇이 움직인 몫만 잰다).
  - 보낼 다리가 없어 선 청소 로봇이 운반 로봇의 남은 route 띠 위에 있으면 미리 비킨다(`_clean_idle_meeting`). seed 2의 막힘은 이 조건이 Q1 막다른 곳에 선 청소 로봇을 운반 로봇이 Q2에 오기 전에 내보내 풀었다.
- **보내기 검사 추가(`_clean_conflict`):** 다리가 운반 로봇이 기다리는 노드를 지나거나 띠에서 `clean_conflict_reach_m` 안이면 보내지 않는다(미룸 이유 `waits for`, `meets ... way`). 보낸 다리가 곧바로 만나 비키기와 보내기가 번갈아 생기지 않게 하려는 것이다.
- **비키기(`_clean_make_way`):** 줄(우회 중이면 우회 goal)을 거두고 진행점을 남긴다(`_clean_requeue`: 줄 위면 그 자리부터, 우회 중이면 우회 시작점부터 줄을 다시 맨 앞에). 지금 자리가 이미 띠 밖이고 기다리는 노드를 쥐지 않았으면 그 자리에 선다. 아니면 청소 격자 빈칸을 가까운 순으로 보며 띠 밖이고 다른 로봇에서 떨어진 점까지 A* 경로(`plan.way`)를 만든다(`_clean_yield_leg`, 후보 40개까지). 경로가 운반 로봇의 첫 구간(지금 위치 → 다음 점)을 가로지르거나, 운반 로봇에 지금보다 더(몸 지름보다) 다가가거나, 다른 로봇에서 `clean_conflict_reach_m` 안을 지나면 쓰지 않는다.
- **이어 하기:** 비킨 다리가 끝나면 서 있고, 다음 걸음에 만남이 없으면 보통 흐름(남은 줄 → 보내기 검사)으로 돌아가 진행점부터 잇는다.
- **집 자리 피난:** 비킬 점이 없거나 같은 운반 로봇과 처음 만난 뒤 `clean_yield_wait_s`가 지나도 아직 만나면 돌아오기 다리(입구 E-1) → `bay_in_1` 후진으로 P-1에 간다(`CLEAN_HOME`). 피난이면 바퀴를 끝내지 않고 남은 줄을 쥔 채 `clean_yield_wait_s` 쉰 뒤 나가서 잇는다(`CLEAN_AT_HOME`). 집 다리를 `clean_stuck_s` 동안 못 보내면 선 채 `CLEAN_STUCK` 경고를 한 번 내고 횟수(`stuck_count`, `clean_view`의 `stuck`)를 센다. 선 동안에도 그 운반 로봇의 길 위면 `clean_retry_s`마다 비킬 점을 다시 찾는다.
- **기존 감시 연결:** 운반 로봇이 청소 로봇 때문에 막히면(`yard_retreat._yield_to_robot`), 길 위의 로봇을 비키게 할 때(`blocked._make_way`), NAV_GOAL 운반 로봇이 청소 로봇 곁에서 `near_stuck_s` 못 나아가면(`watch._watch_near`) 청소 쪽 `_clean_make_way`를 먼저 부른다. 비키지 못하면 운반 로봇은 지금의 막힘 처리를 그대로 받는다.
- **새 파라미터(`fleet_params.yaml`, `common/params.py`, `docs/plan_flip_2026-11-24/fleet_params_plan_split.yaml`):** `clean_yield_band_m 0.30`, `clean_yield_near_m 0.30`, `clean_yield_wait_s 10.0`, `clean_stuck_s 5.0`. 이유는 yaml 주석 한 줄씩.
- **기록:** 사건 `CLEAN_YIELD`, `CLEAN_HOME`, `CLEAN_AT_HOME`, `CLEAN_STUCK`. `tools/cleaner_defer_sweep.py`에 미룸 원인 열 `meet`, `wait`와 마지막 열 `비키기/집/STUCK`(판 전체)을 더했다.

### 2-2. 4-9 STOP과 단계 2의 남은 결함 두 건

- STOP이면 `calls.on_run_mode`가 지금처럼 goal을 거둔 뒤 `_clean_on_stop`이 청소 기록(`job.goal_id`, `job.arrival_by`, BLOCKED 기다림)을 곧바로 지우고 몸 둘레 밖의 지점을 놓는다. RUN이면 다음 걸음에 줄을 진행점부터 다시 보낸다.
- 결함 1(거둔 goal의 기록이 남음): 바깥에서 거둔 goal(STOP, 멈춤 감시 등)은 `_clean_reap`이 위치 보고(`_clean_report`)와 걸음(`_clean_step`) 첫머리에서 지운다. 그래서 거둔 다리를 뒤의 보고가 완료나 실패로 처리하지 않는다. `_clean_cancel`도 `arrival_by`를 지운다(검수 지적으로 고침).
- 결함 2(도착 대기 중 멈춤 감시): `watch._watch_stall`이 청소 로봇의 도착 대기(`job.arrival_by`) 중에는 돌지 않는다.
- STOP 중 BLOCKED 보고로 우회 goal을 보내지 않는다(`_clean_report`가 `running`일 때만 BLOCKED를 센다).

### 2-3. cleaners가 빈 값일 때

새 갈래는 모두 `self._cleaners`가 비면 돌지 않는다(`_clean_on_stop`, `_clean_waits_arrival`, `_watch_near`·`_make_way` 반복, `_yield_to_robot`의 청소 갈래). `test_world_cleaner.py::test_yield_parameters_do_nothing_without_a_cleaner`가 청소 로봇 없는 v7 판에서 새 파라미터를 터무니없는 값으로 바꿔도 사건과 명령이 같음을 본다. 기준 폴더와의 `zone_compare`는 다시 돌리지 않았다(5절).

## 3. 시험

먼저 쓴 시험(`pc/fleet_core/test/test_zone_clean_yield.py`, 19개)과 고친 결함:

| 시험 | 고정하는 것 |
|---|---|
| `test_the_stuck_form_is_undone_by_yielding_before_the_transport_comes` | 운반 로봇이 Q2를 쥐고 오는 중, Q1 막다른 곳의 청소 로봇이 띠 밖으로 비키고 Q1을 놓아 운반 로봇이 잡을 수 있다 |
| `test_a_transport_waiting_for_a_node_the_cleaner_holds_is_a_meeting` | 막힘 꼴(Q2에서 Q1 기다림)이 만남이다 |
| `test_too_late_to_yield_goes_home_and_a_blocked_home_way_is_clean_stuck` | 운반 로봇이 이미 Q2면 집으로 → 집 길도 막혀 `CLEAN_STUCK` 1번 |
| `test_yield_saves_progress_and_resumes_from_it` | 줄 0.3 m에서 비킨 뒤 그 점부터 줄을 잇는다 |
| `test_already_clear_of_the_way_just_stops_and_keeps_progress` | 이미 띠 밖이면 그 자리에 선다 |
| `test_a_yield_asked_during_the_arrival_wait_drops_the_wait` | 검수 지적: 도착 대기 중 비키기 뒤 오래된 대기가 없다 |
| `test_the_yield_path_keeps_off_other_robots_and_their_nodes` | 검수 지적: 다른 로봇과 그 로봇이 쥔 지점을 지나지 않는다 |
| `test_the_meeting_clock_stays_while_on_the_rest_of_the_route` | 검수 지적: 남은 route 위면 만남 시계를 지우지 않는다 |
| `test_no_yield_point_sends_the_cleaner_home_and_the_lap_goes_on` | 비킬 점이 없으면 E-1 → 후진 → P-1, 바퀴를 새로 시작하지 않고 남은 줄로 나간다 |
| `test_still_meeting_after_yielding_goes_home` | `clean_yield_wait_s` 뒤에도 만나면 집으로 |
| `test_a_transport_blocked_by_the_cleaner_asks_it_to_yield` | 운반 로봇의 막힘 처리가 청소 로봇을 비키게 하고 운반 goal은 거두지 않는다 |
| `test_a_moving_transport_closing_in_is_a_meeting` | 다가오는 운반 로봇 |
| `test_stop_cancels_the_line_and_run_resumes_from_progress` | STOP은 줄만 거두고, STOP 중 새 goal 없음, RUN이면 진행점부터 |
| `test_stop_during_the_arrival_wait_does_not_end_the_withdrawn_leg` | 결함 1(STOP) |
| `test_a_goal_withdrawn_while_waiting_for_arrival_is_not_judged` | 결함 1(바깥 취소) |
| `test_blocked_reports_during_stop_do_not_detour` | STOP 중 우회 없음 |
| `test_the_stall_watch_skips_the_arrival_wait` | 결함 2 |

`sim/fake_robots/test/test_world_cleaner.py`에 막히던 seed 2, 5가 주문을 끝내는 시험과 청소 로봇 없는 판의 같음 시험을 더했다. 청소 로봇이 든 위반은 `robot` 칸과 `detail` 둘 다 본다.

기존 시험 기대값은 바꾸지 않았다.

명령(`docs/cleaner_yield_2026-11-25/run_tests.sh`, WSL, `ROS_DOMAIN_ID=41`, 측정은 40):

| 시험 | 결과 |
|---|---|
| 빌드(`build.sh`, `pode_rl_controller` 뺌) | `17 packages finished` |
| `pc/fleet_core` 전체 pytest | `1564 passed, 7 xfailed in 61.15s` |
| `sim/fake_robots/test/test_world_cleaner.py` | `11 passed in 18.73s`(8c5a503 뒤) |
| tools `test_coverage_report.py`, `test_cleaner_defer_sweep.py`, `test_script_help.py` | `16 passed in 1.77s` |
| `pode_follow_wall` colcon test | `40 tests, 0 errors, 0 failures, 0 skipped` |

## 4. 다시 잰 것 (재측정 보고서와 같은 도구, 같은 seed 1~5)

`tools/cleaner_defer_sweep.py`(전 조건, `defer_new.md/.json`), `measure_extra.py`(`extra_new.yaml`). 지도는 같다(v7 sha1 abf223ba45, d2 50b7a07e01). 청소 없음 판 주문 끝은 192.8 s로 같다.

### 4-1. v7 default, seed별 (옛 = `report_2026-11-25_cleaner_on_main.md` 3-3·3-4)

| seed | 막힘 옛 → 새 | 주문 끝 s 옛 → 새 (청소 없음 대비) | 끝낸 줄 옛 → 새 | 미룸 옛 → 새 | 덮은 비율(계획) 옛 → 새 | 덮은 비율(받은 위치) 옛 → 새 | 비키기 / 집 / CLEAN_STUCK | 위반 옛 → 새 |
|---|---|---|---|---|---|---|---|---|
| 1 | 244 s부터 → **없음** | 못 끝냄 → 196.6 (+1.3) | 11/17 → 7/17 | 29 → 46 | 0.494 → 0.306 | 0.591 → 0.433 | 25 / 1 / 0 | 0 → 0 |
| 2 | 143 s부터 → **없음** | 못 끝냄 → 210.6 (−0.4) | 7/17 → 3/17 | 41 → 58 | - → 0.136 | 0.540 → 0.405 | 10 / 2 / 1 | 0 → 0 |
| 3 | 없음 → 없음 | 177.8 → 178.2 (+0.2) | 6/17 → 4/17 | 47 → 55 | 0.519 → 0.289 | 0.531 → 0.360 | 30 / 1 / 1 | 0 → 0 |
| 4 | 없음 → 없음 | 195.0 → 179.2 (+1.0) | 5/17 → 7/17 | 52 → 46 | 0.490 → 0.311 | 0.505 → 0.517 | 26 / 0 / 0 | 0 → 0 |
| 5 | 134 s부터 → **없음** | 못 끝냄 → 212.2 (+10.6) | 7/17 → 8/17 | 41 → 49 | - → 0.708 | 0.541 → 0.765 | 25 / 0 / 0 | 0 → 0 |

- 옛 판의 "끝낸 줄·미룸·덮은 비율"은 막힌 seed에서 막힐 때까지의 값이라 그대로 견주면 안 된다. 받은 위치 덮은 비율은 첫 바퀴의 줄·우회 중 받은 위치로 잰다.
- 청소 로봇과 운반 로봇의 최소 거리: 옛 0.1825 m → 새 0.1888 m(다섯 판 최소).
- `CLEAN_STUCK` 2번(seed 2, 3)은 집 다리를 5 s 넘게 못 보낸 일화다. 두 판 모두 그 뒤 풀려 주문을 끝냈다.

### 4-2. 조건별 요약 (옛 → 새)

| 조건 | 끝낸 줄 | 미룸 | 덮은 비율(받은 위치) | 주문 끝 s | 지연 평균/최대 s | 막힘 seed | 위반(청소) | 비키기/집/STUCK(새) |
|---|---|---|---|---|---|---|---|---|
| default | 5~11 → 3~8/17 | 29~52 → 46~58 | 0.542 → 0.496 | 186.4 (3 못 끝냄) → **195.4** | 8.3/16.8 → **2.5/10.6** | [1, 2, 5] → **없음** | 0(0) → 0(0) | 116/4/2 |
| reach015 | 5~8 → 3~9 | 36~52 → 42~57 | 0.552 → 0.500 | 205.9 (2) → 195.4 | 22.0/38.3 → 2.6/7.3 | [2, 5] → 없음 | 0(0) → 0(0) | 153/4/1 |
| r2_off | 5~14 → 3~8 | 25~52 → 46~58 | 0.572 → 0.529 | 186.4 (3) → 203.0 | 8.3/16.8 → 10.2/38.4 | [1, 2, 5] → 없음 | 0(0) → 0(0) | 114/4/2 |
| r4_off | 9~14 → 9~13 | 19~42 → 25~43 | 0.701 → 0.671 | 199.9 (3) → 194.0 | 5.4/6.0 → 1.1/3.9 | [1, 4] → 없음 | 9(9) → **0(0)** | 227/2/0 |
| ahead05 | 5~14 → 9~14 | 29~48 → 24~41 | 0.686 → 0.763 | 192.6 (4) → 193.5 (1) | 14.4/14.4 → 2.9/9.2 | [1, 3, 5] → [5] | 3(2) → 0(0) | 179/5/3 |
| ahead10 | 11~16 → 4~13 | 18~37 → 9~41 | 0.768 → 0.731 | 201.2 (1) → 196.1 (2) | 10.6/19.2 → 1.2/2.0 | 없음 → [5] | 9(9) → 0(0) | 153/6/3 |
| r3_only | 9~15 → 13~15 | 11~40 → 22~35 | 0.713 → 0.877 | 213.9 (2) → 215.2 (1) | 28.0/59.8 → 18.7/56.9 | [1, 2] → 없음 | 3(2) → 1(1) | 146/8/3 |
| d2_default | 0/9 → 0/9 | 36 → 36 | 0 → 0 | 같음(243.5) | - | 없음 | 0 → 0 | 70/0/0 |

- 표의 "N 못 끝냄"은 주문을 끝내지 못한 판 수다. ahead05·ahead10의 seed 5는 아직 막힌다(옛 기록과 같은 실험 조건이고 기본값이 아니다).
- 바닥 물체 판과 청소 경로 요약(`extra_new.yaml`)은 재측정 보고서와 한 줄(저장소 경로)만 다르다. 청소 로봇 혼자 판은 운반 로봇이 없어 비키기가 돌지 않는다.

## 5. 정할 것 (계획이 정하지 않아 가장 단순한 것을 고름)

1. **청소량이 줄었다.** 막힘을 없앤 값이다. 운반 로봇의 길에서 미리 비키고, 기다리는 노드와 띠에 닿는 다리를 보내지 않아 미룸이 늘었다. 첫 바퀴 덮은 비율이 0.542 → 0.496이다. 이 값을 받아들일지, 띠 폭·다가옴 거리를 줄여 다시 잴지 정해야 한다.
2. **띠의 넓이:** 선 청소 로봇의 만남과 비킬 점 고르기는 `clean_yield_band_m`(0.30, 중심 거리 0.235 m)로, 달리는 다리와 띠는 보내기 검사와 같은 `clean_conflict_reach_m`(0.18)로 본다. 같은 값을 써야 보낸 다리가 곧바로 비키기를 부르지 않는다.
3. **남은 route 전체로 미리 비키기:** 계획은 "목표 노드와 그 뒤 base"만 말한다. 그것만으로는 Q1 막다른 곳에서 나갈 수 있는 때(운반 로봇이 아직 멀 때)를 놓친다. 그래서 보낼 다리가 없어 선 청소 로봇과 비킬 점 고르기에는 남은 route 전체를 쓴다.
4. **비키기 경로가 남이 쥔 지점을 지나는 규칙:** 비켜 주는 그 운반 로봇이 쥔 지점은, 그 로봇이 지점에서 `clean_conflict_ahead_m`(1.0 m) 밖이면 쥐지 않고 지난다. 다른 로봇이 쥔 지점은 경로가 지금보다 그 지점에 다가가면 쓰지 않는다. 지금 자리에서 이미 둘레 안인 지점은 막지 않는다(그 지점에서 멀어지는 길이다). 이 규칙이 없으면 seed 2가 풀리지 않는다. Gazebo에서 실제 거리로 확인해야 한다.
5. **`clean_base_check`가 ahead/off일 때:** 다리와 띠의 만남(보내기와 달리는 중)도 보지 않는다. 실험 조건(r4_off, ahead*, r3_only)의 뜻을 지키고 보내기와 비키기가 같은 기준이 되게 했다. 검수가 "비키기 계약이 약해진다"고 지적했다. 기본값(all)에는 영향이 없다.
6. **"운반 로봇이 지나가면":** 따로 기다리지 않고, 만남이 없어지면 보통 흐름으로 돌아가 보내기 검사가 정한다.
7. **"비킨 뒤에도 정해진 시간 넘게 막힘":** 같은 운반 로봇과 처음 만난 뒤 `clean_yield_wait_s`(10 s)가 지나도 아직 만나는 것으로 본다.
8. **피난 뒤:** 바퀴를 끝내지 않고 남은 줄을 쥔 채 `clean_yield_wait_s` 쉰 뒤 나가서 잇는다(바퀴 끝의 돌아오기에서 피난하면 보통 바퀴 끝으로 친다).
9. **비키는 단계:** 줄과 돌아오기 단계(LINES, BACK)에서만 비킨다. 나가기(OUT, P-1 → E-1 → 출발점 약 0.17 m)와 후진 진입(BAY_IN), 집에 있을 때는 비키지 않는다.
10. **우회 중 거둠:** 우회 시작점(진행점)부터 줄을 다시 쓴다. 우회로 남길 빈틈은 버린다(그 칸을 다시 쓰므로 빈틈이 생기지 않는다).
11. **사람·카트 우회(4-5)에서 막은 것이 운반 로봇이면:** 지금처럼 미룬다. 계획은 4-6을 쓰라고 하지만, 다가옴·띠 조건이 대개 먼저 비키기를 부르므로 이번에 바꾸지 않았다.
12. 재측정 보고서 5절 3(청소 goal 결과 유실을 IR-62처럼 결과 모름으로 두기)은 사용자 결정 전이라 바꾸지 않았다.

## 6. 못 한 것

- Gazebo 만남 판단 확인(계획 8절, 단계 4), 근사가 놓친 만남 수 지표.
- `tools/zone_compare.sh`로 cleaners 빈 값의 기준 폴더 diff를 다시 뜨지 않았다. 대신 같음 시험(2-3)과 fleet_core 전체 시험으로 봤다.
- path 미룸 고치기(defer 결정 2)는 이번 범위가 아니다.

## 7. Codex 검수

- 1차(`63f6ea2..7d269a4`): 지적 5건. 코드로 확인해 4건을 고쳤다(`d4b5708`): 도착 대기 중 비키기 뒤 오래된 대기(`_clean_cancel`), 비키기 경로가 다른 로봇과 그 로봇이 쥔 지점을 지남, 남은 route 위에서 만남 시계가 지워짐, 시뮬 시험이 두 로봇 위반을 놓침. 1건(`clean_base_check` ahead/off에서 다리-띠 만남을 끔)은 의도한 설계라 5절 5에 적었다.
- 2차(`7d269a4..d4b5708`): 지적 2건. ① 청소 로봇이 이미 남이 쥔 지점의 둘레 안에 있을 때, 그 지점에 더 다가가지 않는 비키기 경로는 쥐지 않고 지난다. 코드로 확인했지만 고치지 않았다. 그 경로는 지점에서 멀어지기만 하고, 막으면 둘레 안에 선 청소 로봇이 나갈 길이 없다(5절 4). ② seed 1 시뮬 시험도 detail의 두 로봇 위반을 세야 한다. 고쳤다(`8c5a503`, 기준을 조이는 쪽).
- 3차(`d4b5708..8c5a503`, 시험만): 결함 없음.
- 느슨해진 기준은 없다. 기존 시험의 기대값과 시간 한도는 바꾸지 않았다. `test_zone_clean_conflict.py`도 바꾸지 않았다.

## 8. 파일 (`docs/cleaner_yield_2026-11-25/`)

- `defer_new.md/.json`: 전 조건 미룸 표. `extra_new.yaml`: 청소 경로 요약과 바닥 물체 판.
- `build.sh`, `run_tests.sh`, `run_sweep.sh`, `run_extra.sh`, `quick_test.sh`: 다시 돌리는 명령. `test_*.log`: 시험 출력.
- `repro_stuck.py`: 막힘 꼴 상태 찍기. `yield_events.py`: 한 판의 비키기 사건과 까닭.
