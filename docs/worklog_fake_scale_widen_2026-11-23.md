# ROS 시나리오의 로봇 배율을 0.67~1.0으로 넓힘 (2026-11-23)

사용자 결정(2026-11-22): "ROS 시나리오의 느린 배율(0.67~1.0)은 2번을 사건 기준으로 바꾼 뒤에 넓혀. 지금은 1.0과 0.9 그대로 둬."
2번(stuck_robot, blocked_lane 고장을 사건 기준으로, reinstate는 빈 로봇을 치움)은 main에 들어가 있다
(`docs/worklog_fake_speed_2026-11-22.md` 9절). 이 문서는 그 문서 5절 3번의 실행이다.

브랜치 `feat/fake-scale-widen-2026-11-23`(main 9cbd413에서 시작). push와 병합은 하지 않았다.

## 1. 규칙

모든 ROS 시나리오 파일(`sim/fake_robots/scenarios/*.yaml` 12개)에서 로봇 이름으로 배율을 정한다.

| 로봇 | 전 | 후 | 속도(출처 0.12 m/s x 배율) |
|---|---|---|---|
| pode1 | 1.0 | 1.0 | 0.12 |
| pode2 | 0.9 (real_* 두 파일은 1.0) | 0.67 | 0.080 |
| pode3 | 1.0 | 0.85 | 0.102 |

- 그래서 로봇이 셋인 시나리오는 모두 범위의 양 끝(1.0, 0.67)과 가운데(0.85)를 쓴다. 둘인 `real_two_orders.yaml`은 1.0과 0.67이다.
- pode2를 느린 로봇으로 둔 것은 전부터 pode2가 느린 로봇(0.9)이었기 때문이다. 시나리오 주석이 말하는 순서(예: pode2가 먼저 작업 도크에 닿음)도 그대로 남았다(3절).
- `pc/fleet_manager/test/fms_restart.yaml`(0.1667, 일부러 느린 시험 대역)은 바꾸지 않았다.
- `real_*` 두 파일의 배율은 가짜 로봇으로 돌릴 때만 쓰인다(실물 속도는 start.sh). 그래도 같은 규칙을 따른다.

## 2. 결정 행

| 선택지 | 장점 | 단점 |
|---|---|---|
| ㄱ. 로봇 이름으로 고정(pode1 1.0, pode2 0.67, pode3 0.85) | 한 줄로 설명된다. 모든 파일이 같다. 시나리오 주석의 도착 순서가 전과 같다(pode2가 전에도 느렸음) | 시나리오마다 느린 로봇의 자리가 같다(P-2 출발) |
| ㄴ. 시나리오마다 느린 로봇을 돌린다(파일마다 pode1/2/3 차례) | 느린 로봇이 여러 출발 자리를 거친다 | 규칙이 파일 순서에 묶인다. 주석의 도착 순서가 바뀌는 시나리오가 생겨 다시 맞춰야 한다 |
| ㄷ. 로봇 이름이나 씨드로 뽑는다(해시, Isaac처럼) | 범위를 고루 쓴다 | 고장 시각이 사건이 아닌 시나리오(operator_recovery 60/68 s, auto_isolation 120 s, shared_lane_failure 40/50 s)는 판마다 다른 순간에 고장이 난다. 1차 문서 2절에서 같은 이유로 버렸다 |
| ㄹ. 두 대만 넓힌다(pode1 1.0, pode2 0.67, pode3 1.0) | 바뀌는 줄이 적다 | 가운데 값을 쓰지 않는다 |

고른 것: ㄱ. 이유: 범위의 양 끝과 가운데를 모두 쓰면서 고장 시각이 정해진 시나리오의 뜻(누가 먼저 닿는지)을 지킨다.
결과로 확인했다(3절): 고장을 맞은 로봇과 주문이 전과 같다.

## 3. 바꾸기 전후 비교

같은 PC(WSL, ROS 2 Jazzy, ROS_DOMAIN_ID 49, LOCALHOST)에서 main 9cbd413(전)과 b3b3cdc(후)를 돌렸다.
시나리오 시험(`pc/fleet_manager/test/test_scenario_integration.py`)은 시나리오마다 도메인 50~59를 쓴다.
열의 뜻은 1차 문서 3.1, 9.7과 같다. 여유 = `timeout_s` - 끝 시각. 벽시계 s는 끝 시각 ÷ speed_factor 5다(1차 문서와 같은 계산, 상한은 `WALL_TIMEOUT_S` 420).
ETA 오차는 `docs/distance_remaining_2026-11-21/eta_error.py`의 `samples()`로 `fms.jsonl`을 재생해 잰다(관제가 쓰는 값 - 실제 남은 시간, +면 늦게 닿는다고 본 것).
모든 판이 "기록과 같게 재생"됐다. 이 계산은 1차 문서 9.7의 값을 그 판의 기록으로 다시 내어 한 자리까지 같음을 확인했다.

| 시나리오 | 판정 | 끝 시각(시뮬 s) | timeout_s | 여유(시뮬 s) | 벽시계 s(상한 420) | ETA 절대 오차 평균 s | ETA 오차 평균(부호) |
|---|---|---|---|---|---|---|---|
| basic_orders | 통과 → 통과 | 233.9 → 253.8 | 900 | 666 → 646 | 46.8 → 50.8 | 2.58 → 4.17 | -2.54 → -4.15 |
| order_burst | 통과 → 통과 | 353.8 → 392.2 | 1200 | 846 → 808 | 70.8 → 78.4 | 2.77 → 4.83 | -2.75 → -4.82 |
| shared_lane_failure | 통과 → 통과 | 270.8 → 261.8 | 900 | 629 → 638 | 54.2 → 52.4 | 2.56 → 3.87 | -2.52 → -3.81 |
| stock_shortage | 통과 → 통과 | 181.2 → 193.0 | 900 | 719 → 707 | 36.2 → 38.6 | 2.88 → 3.62 | -2.84 → -3.60 |
| stuck_robot | 통과 → 통과 | 400 → 400(목적) | 400 | 시간 초과(목적) | 80.0 → 80.0 | 2.42 → 4.85 | -2.41 → -4.84 |
| operator_recovery | 통과 → 통과 | 324.5 → 346.0 | 900 | 576 → 554 | 64.9 → 69.2 | 0.51 → 1.11 | -0.32 → -1.02 |
| blocked_lane | 통과 → 통과 | 263.6 → 269.4 | 900 | 636 → 631 | 52.7 → 53.9 | 1.76 → 3.47 | -1.71 → -3.44 |
| auto_isolation | 통과 → 통과 | 253.2 → 263.5 | 900 | 647 → 637 | 50.6 → 52.7 | 1.59 → 3.07 | -1.47 → -3.03 |
| reinstate | 통과 → 통과 | 359.4 → 372.9 | 900 | 541 → 527 | 71.9 → 74.6 | 2.10 → 2.70 | -2.04 → -2.66 |
| arm_fault | 통과 → 통과 | 133.1 → 140.3 | 900 | 767 → 760 | 26.6 → 28.1 | 1.52 → 1.73 | -1.46 → -1.69 |

읽는 법:
- 열 판 모두 통과했다. 끝 시각은 대개 2~11% 늦어졌다(pode2 0.108 → 0.080, pode3 0.12 → 0.102 m/s). shared_lane_failure만 9 s 일찍 끝났다(한 판).
- 가장 작은 여유는 527 시뮬 s(reinstate), 가장 긴 벽시계는 80 s(stuck_robot, 목적)다. 시간 초과가 빠듯해진 곳은 없다.
- ETA는 더 일찍 본다(-)가 커졌다. 관제는 모든 로봇을 출처 0.12로 보는데 pode2, pode3이 그보다 느려서다. 절대 오차 평균은 0.5~2.9 s에서 1.1~4.9 s로 컸다.
  관제는 로봇마다 속도를 모른다. 고칠지는 아래 6절에 적었다(이번에는 고치지 않음).
- main(전)의 ETA 값은 1차 문서 9.7(092fdc4 이후 2단계)과 다르다. 그 뒤 main에 들어간 관제 변경 때문이며 이번 일과 무관하다.

고장 시나리오의 뜻이 지켜졌는지 사건 기록(`events.csv`)으로 보았다(시뮬 s, 전 → 후):

| 시나리오 | 노린 순간 | 전 | 후 |
|---|---|---|---|
| operator_recovery | pode2가 먼저 작업 도크, O-001 HELD, 60 s pode2 치움, 68 s pode1 치움 | HELD 16.5(pode2), 치움 60.8 [WORK_DOCK], 68.8 | HELD 18.6(pode2), 치움 60.8 [HALL, WORK_DOCK], 68.8 |
| auto_isolation | pode1이 O-002 HELD, 120 s 치움 | HELD 40.4(pode1), 치움 120.9 | HELD 43.4(pode1), 치움 120.8 |
| stuck_robot | pode1 출고 도크 뒤 멈춤 | DOCKED 78.7, COMM_LOST 85.7 | DOCKED 87.8, COMM_LOST 94.9 |
| blocked_lane | W_Q에서 막힘 표시 | 표시 48.7 | 표시 52.5 |
| reinstate | pode2 출고 뒤 치움 | 치움 54.6 | 치움 61.5 |
| shared_lane_failure | 40 s pode2 통신 끊김, 50 s pode1 이동 중단 둘 | COMM_LOST 42.9, GOAL_RETRY 58.4/60.9 | COMM_LOST 43.3, GOAL_RETRY 63.3/65.6 |

operator_recovery에서 pode2를 치울 때 HALL도 함께 풀렸다(느려서 그때 아직 HALL을 쥐고 있었음). 기대값(ZONES_CLEARED 2, ORDER_REQUEUED 2, HELD O-001)은 그대로 맞았다.

## 4. 시험 묶음

| 묶음 | 전(main 9cbd413) | 후(b3b3cdc) |
|---|---|---|
| pode_nav_config | 111 passed (조정자 실행) | 111 passed |
| fleet_core | 1230 passed, 7 xfailed (조정자 실행) | 1230 passed, 7 xfailed |
| fake_robots | 1156 passed, 1 skipped, 30 xfailed (조정자 실행) | 1156 passed, 1 skipped, 30 xfailed |
| fleet_manager | 137 passed (조정자 실행, 00:30쯤) / 1 failed, 136 passed (이 실행) | 2 failed, 135 passed |

fleet_manager의 실패 셋은 모두 바꾼 파일을 읽지 않는 시험이다(5절).

## 5. 간헐 실패로 보이는 것(고치지 않음)

| 시험 | 어디서 실패 | 메시지 |
|---|---|---|
| `test_node_wiring.py::test_stop_cancels_the_fill_on_its_server_and_run_sends_what_remains` | main 전체 실행 | `the fill never reached the server`(40 s 동안 `GOAL_NOT_SENT ... not ready`와 ORDER_SENT 반복) |
| `test_node_wiring.py::test_the_heartbeat_comes_twice_a_second` | 후 전체 실행 | 20 s 동안 heartbeat 0개(`[]`) |
| `test_fms_restart.py::test_robots_stop_without_the_fms_and_a_new_one_waits_for_all_of_them` | 후 전체 실행 | 10 s 안에 마지막 goal이 ABORTED가 아님(`{'pode1': 6, 'pode2': 4}`) |

셋 다 바꾼 시나리오 파일을 쓰지 않는다. `test_node_wiring.py`는 시나리오 없이 노드를 띄우고, `test_fms_restart.py`는 바꾸지 않은 `fms_restart.yaml`을 쓴다.
다른 것을 돌리지 않는 WSL에서 하나씩 다시 돌린 결과(도메인 49):

| 시험 | main 9cbd413 | b3b3cdc |
|---|---|---|
| stop_cancels_the_fill... | 5/5 통과 | 5/5 통과 |
| heartbeat_comes_twice_a_second | 5/5 통과 | 5/5 통과 |
| fms_restart | 8/8 통과(3 + 5) | 2/3 통과 |

- node_wiring 둘은 따로 돌리면 늘 통과한다. 전체 묶음 안에서만 한 번씩 났다(앞 시험의 노드·발견 상태를 탈 때로 보인다, 원인은 보지 않음).
- fms_restart의 다시 돌린 실패는 전체 묶음 때와 메시지가 다르다: `'t=401.6 WORK_DOCK released from pode3 while 0.08 m from it'`(관제가
  로봇이 구역에서 0.08 m일 때 놓았다는 위반, `summary["violations"]`). 관제 동작의 문제로 보인다.
- 이 시험의 입력은 전후가 바이트까지 같다: 9cbd413..b3b3cdc는 `sim/fake_robots/scenarios/*.yaml` 12개만 바꾸고, 이 시험은
  `pc/fleet_manager/test/fms_restart.yaml`만 읽는다. 그래서 이번 변경이 원인일 길은 없다고 본다. 다만 main에서는 8번 모두
  통과했고 후에서는 4번 가운데 2번(전체 묶음 1, 다시 돌림 1) 실패해 횟수로는 갈린다. 관제 코드는 고치지 않았다. 따로 볼 일이다.
- 실패 기록: WSL `~/fake_speed_out/scale-after/fleet_manager.log`, `~/fake_speed_out/scale-main/fleet_manager.log`,
  `~/fake_speed_out/scale-rerun/fail_scale-after_test_fms_restart.py_3.log`.

## 6. 바꾼 시험 기대값과 이유

없음. 시나리오의 `expect`, 고장 시각, 시험 코드는 바꾸지 않았다. 고장 시각을 사건으로 바꿀 일도 없었다(모두 통과, 3절에서 노린 순간 확인).

## 7. 느슨해진 기준

없음. 대기 시간, 시간 초과, skip/xfail, 단언은 하나도 바꾸지 않았다.

## 8. 빠듯해진 시험

새로 빠듯해진 시나리오는 없다(가장 작은 여유 527 시뮬 s, 가장 긴 벽시계 80 s / 420 s).
fake_robots와 fleet_manager의 단위 시험은 시나리오 파일의 배율을 쓰지 않으므로(대역 배율은 시험 코드 안) 1차 문서 9.5의 15 s 대기 목록은 이번 변경과 무관하다. 시험별 시간(`--durations`)은 이번에 재지 않았다.

## 9. 알아야 할 포인트

- 관제는 로봇마다 속도를 모르고 모두 출처 0.12로 본다. 그래서 느린 로봇의 ETA는 늘 일찍 본다(-). 실물에서도 로봇마다 실제 속도가 다르면 같다.
  관제가 로봇의 실측 속도를 배우게 할지는 결정할 일이다(이번에는 손대지 않음).
- 새 시나리오를 만들 때도 같은 규칙(pode1 1.0, pode2 0.67, pode3 0.85)을 따른다. 사본 시험(`test_nav_config.py`, `test_nav_speed.py`)은 범위(0 초과 1 이하, 0.67~1.0)만 본다. 규칙 자체는 시험이 지키지 않는다.
- 측정 결과(JSON 줄)는 WSL `~/fake_speed_out/scale-main/scen.jsonl`, `~/fake_speed_out/scale-after/scen.jsonl`에 있다(저장소에 넣지 않음).
