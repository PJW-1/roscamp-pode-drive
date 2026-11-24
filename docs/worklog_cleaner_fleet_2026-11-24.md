# 청소 로봇 관제 쪽(단계 2) 작업 기록 (2026-11-24)

브랜치 `feat/cleaner-fleet-2026-11-24`, 출발점 67cfb85. 계획은 `docs/청소_로봇.md` 4절(4-1, 4-3, 4-4, 4-5, 4-7, 4-8, 4-11, 4-10의 일부)이다. 단계 1(커버리지 계획, `coverage.py`)의 줄과 이동 경로를 쓴다. pode_fsm, msg 필드, action, `orders/`는 바꾸지 않았다.

## 1. 정한 것

- 로봇 3대 가운데 운반 2대, 청소 1대. 청소 로봇은 운반으로 바뀌지 않는다.
- 역할은 launch 인자 `cleaners`(기본 빈 값)로 정한다. 시뮬에서는 pode3이 P-1에서 출발한다.
- P-1은 청소 로봇 집 자리다. 운반 로봇은 P-2, P-3에서만 기다린다.
- STOP에서 청소 로봇은 그 자리에 선다. 4-6(운반 로봇에게 비키기)과 4-9(STOP 뒤 이어 하기)는 단계 3이다. 지금 goal(줄이든 우회든)을 거두는 입구는 `_clean_cancel` 하나로 모았다.
- task_type은 문자열 `CLEAN`이다(팀 합의 전 제안).

## 2. 무엇을 했나

| 계획 | 한 일 | 자리 |
|---|---|---|
| 4-1 역할 | `ZoneTraffic(..., cleaners=)`, `FleetManager(..., cleaners)`, 노드 파라미터 `cleaners`, launch 인자 `cleaners`(pc.launch.py, gz.launch.py). 청소 로봇은 하나만, 이름은 robots 안에 있어야 한다 | `traffic/zone/base.py`, `cleaning.py` `clean_jobs`, `manager.py`, `fleet_manager_node.py` |
| 4-3 운반 흐름에서 빼기 | 주문 코어와 팔 일에는 운반 로봇 목록만 넘긴다. 호출은 `NOT_AVAILABLE`로 거절, 호출 고르기 후보에서 뺀다. 확인·재투입·격리는 주문 코어로 넘기지 않는다 | `manager.py`, `manager_route.py`, `traffic/zone/calls.py` |
| 4-4 dock·lane 금지 | 청소 로봇은 공유 구역(HALL) 밖 지점을 쥐지 않는다: 길 값 무한, `_take` 거절, 다리마다 지점 검사, dock·lane 구역에 배치하지 않음 | `goals.py` `_barred`, `cleaning.py` `_clean_conflict`, `placement.py` |
| 4-5 실행부 | 나가기(`bay_out_1` + E-1에서 출발점) → 줄을 차례로 FOLLOW_PATH → 돌아오기(출발점 → E-1) → `bay_in_1` 후진으로 P-1. 줄마다 지나는 지점(몸 둘레 `clean_reach_m`)을 HALL과 함께 쥐고, 지나간 지점은 놓는다. 겹치면(남이 쥔 지점, 가까이 선 로봇, 남의 base) 미루고 `clean_retry_s` 뒤 다시, 한 바퀴에 `clean_defer_limit`번 넘으면 포기 | `traffic/zone/cleaning.py` |
| 4-5 BLOCKED (11-24 새 결정) | 아래 3절 | `cleaning.py` `_clean_report` 이하 |
| 4-7 우선순위 | 청소 로봇이 있으면 청소 로봇 0, 운반 로봇은 기존 값 + 1. 청소 로봇이 없으면 그대로 | `base.py` `_priority` |
| 4-8 상태와 기록 | `/fleet/robots`, `/fleet/assignments`의 task_type `CLEAN`, 로봇 보기에 `role`, `clean`(바퀴, 단계, 줄, 끝낸 줄, 미룬 줄, 포기, 빈틈). 사건 `CLEAN_*`. 기록 머리에 `cleaners`(있을 때만), replay가 그것으로 관제를 세운다 | `manager_views.py`, `manager_record.py`, `tools/replay.py` |
| 4-11 집 자리 | 운반 로봇의 주차 후보, 비키기 주차 자리, 스테이션 대기 자리에서 P-1을 뺀다. 운반 로봇이 P-1에 놓이면 `CLEAN_HOME_TAKEN` | `turns.py`, `watch.py`, `placement.py` |
| 4-10 값 | `clean_reach_m` 0.18, `clean_blocked_s` 2.0, `clean_retry_s` 5.0, `clean_defer_limit` 3, `clean_detour_margin_m` 0.20, `clean_block_look_m` 0.5. 모두 제안값 | `common/params.py`, `data/fleet_params.yaml` |

## 3. 줄 위 BLOCKED: 기다림 → 우회 → 빈틈 → 미룸

1. 시간은 첫 BLOCKED 보고부터 센다(`BLOCKED_WAIT` 사건). 그 사이 진행점(줄 축에 내린 위치)이 2 cm 넘게 나아가면 보고가 hold로 남은 것으로 보고 기다림을 새로 시작한다. RECOVERING(local_yield) 보고는 BLOCKED가 아니므로 세지 않는다.
2. `clean_blocked_s` 뒤 줄 goal을 거두고, 막은 것이 운반 로봇이면(장애물 자리에서 `near_m` 안) 우회하지 않고 미룬다(4-6은 단계 3).
3. 복귀점: BLOCKED 보고의 장애물 좌표가 있으면 그 점을 줄에 내린 자리, 없으면 진행점에서 `clean_block_look_m`(Pode `block_lookahead_m` 0.5) 앞. 거기서 몸 반지름 + `clean_detour_margin_m` 더 간 첫 청소 격자 빈칸이다. 줄에 닿기 전(이동 구간)에 막혔으면 줄 시작점으로 돌아간다(빈틈 없음).
4. 진행점~복귀점 띠(폭: 몸 지름 + 여유)가 지나는 지점을 모두 쥔다. 띠가 dock·lane 구역이나 keepout_extra에 닿거나, 지점을 쥘 수 없으면 `CLEAN_DETOUR_FAILED` 뒤 미룬다.
5. 우회는 Nav2 goal(MoveCmd mode `nav_goal`)로 같은 goal 수명(번호, 결과, 거절, 유실, 취소)에 올린다. controller는 goal마다 고르지 않는다: 청소 로봇의 Nav2 설정(판마다)이 정한다. MoveCmd에 칸을 더하지 않았다.
6. 복귀점 전에 다시 BLOCKED가 `clean_blocked_s` 이어지면 복귀점을 한 번 더 앞으로 옮긴다. 그래도 막히면 미룬다.
7. 우회가 끝나면 진행점~복귀점 줄 칸 가운데 받은 위치(우회 중 보고를 이은 선)의 청소 폭 안에 들지 않은 칸만 `CLEAN_GAP`으로 남기고, 복귀점부터 남은 줄을 FOLLOW_PATH로 잇는다. 빈틈은 다음 바퀴 줄 목록 뒤에 짧은 줄로 붙는다(`CLEAN_LAP_START ... (G gaps)`).

## 4. 계획과 다른 것

- 돌아오는 길(출발점 → E-1)이 바닥 물체로 막혀 계속 BLOCKED이면 같은 길을 끝없이 다시 보냈다(가짜 세계 150 s). 막힌 뒤에는 출발점까지 Nav2 goal(`back_nav`)로 돌아간 뒤 고정 경로로 들어가게 했다.
- 진행점이 줄에 닿기 전(이동 구간)에 막히면 줄 시작점까지 우회한다. 계획에는 줄 위 경우만 있다.
- 우회 띠 검사의 "금지 구역"은 `keepout_rects(site, 0)`(dock·lane 구역 사각형 + keepout_extra)로 보았다. 여유 0.20이면 띠 반폭이 0.185 m라 벽이나 다른 구역 가까운 줄(v7 16줄 가운데 0, 1, 4, 5, 7, 8, 9, 15번 앞부분)은 우회하지 않고 미룬다. 보수적으로 둔 것이다.
- fakes.launch.py는 관제를 띄우지 않아 인자를 더하지 않았다. fleet_setup의 역할 해석(5-6)은 하지 않았다.
- 결과 없이 끝난 줄(유실, 못 보냄, 거절, heartbeat 끊김)은 그 줄을 다시 앞에 둔다. 거절은 기존 `_goal_failed`(시도 수, 한도 넘으면 격리)를 그대로 탄다.

## 5. 시험

- fleet_core: `cd pc/fleet_core && ROS_DOMAIN_ID=92 python3 -m pytest -q test -p no:cacheprovider` → `1350 passed, 7 xfailed in 166.08s`(같음 비교 실행 안에서, 09e3092). 도착 확인을 고친 뒤 `1362 passed, 7 xfailed in 104.34s`. 새 시험만: `37 passed`
- 새 시험 `pc/fleet_core/test/test_zone_cleaning.py`(37개): 역할, 우선순위, 청소 로봇 하나만, 집 자리, dock·lane 거절, 배치 거절, 줄 FOLLOW_PATH, 못 보낸 줄 다시 보냄, 늦은 거절 버림, 못 보냄 때 지점 놓기, 짧은 BLOCKED, hold 중 진행, RECOVERING 안 셈, 우회와 빈틈(덮인 칸 뺌, 늦은 결과 버림, 남은 줄 이음), 빈틈이 다음 바퀴 줄, 장애물 좌표 없음, 복귀점 없음, 우회 실패, 다시 막힘 한 번 옮김 뒤 미룸, 운반 로봇이 막음, 띠 금지 구역, 기록 머리, 다리마다(out, line, detour, back_nav, back, bay_in) 끝에서 먼 성공은 도착 보고를 기다리고 `arrival_settle_s` 안에 없으면 NOT_AT_TARGET 실패.
- 새 시험 `sim/fake_robots/test/test_world_cleaner.py`(8개): v7 운반 2 + 청소 1(seed 1), 바닥 물체(0.35, 0.4875) 하나로 우회·빈틈·미룸.
- fake_robots 전체: `996 passed, 12 skipped, 30 xfailed in 1046.95s`, tools: `274 passed, 15 skipped`(같음 비교 실행 안에서, 09e3092)
- fleet_manager + launch_wiring(스크래치 colcon install): `3 failed, 172 passed, 34 skipped`. launch_args 2개는 `cleaners` 인자가 없는 launch 맥락에서 깨졌고 09e3092로 고쳐 `75 passed`(test_launch_args + test_launch_wiring). 나머지 1개(scenario reinstate, cleaners 없음)는 시험 셋을 함께 돌려 CPU가 붐빌 때 들어낸 로봇의 바닥 밖 보고 하나가 `INPUT_REJECTED`로 세어진 것이고, 혼자 다시 돌리면 통과했다(`4 passed`, launch_args 포함)
- 같음 비교(cleaners 비움, `/home/ane/zone_baseline/aff00d7`): trace.txt, situations.md, store.md, order_v7.md, order_d2.md 모두 diff 0줄. suite_trace.json 기존 1276개 값 같음, 새 시험 81개(09e3092에서 다시 돌림. b73f4d4에서도 같았다)

## 6. 머리 없는 수치 (v7, pode1 P-2, pode2 P-3, pode3 P-1, cleaners pode3)

| seed | 주문 끝(s) | 첫 바퀴(s) | 끝낸 줄 | 미룸 | 포기한 줄 | 덮은 비율 | 위반 |
|---|---|---|---|---|---|---|---|
| 1 | 225.1 | 108.7 | 11/16 | 25 | 8~12 | 0.631 | I5_STANDING pode1 EXPRESS 163.1 |
| 2 | 223.5 | 124.7 | 9/16 | 31 | 1, 2, 8~11, 15 | 0.573 | 없음 |
| 3 | 179.7 | 107.0 | 8/16 | 33 | 1~3, 8~11, 15 | 0.554 | 없음 |
| 4 | 179.0 | 102.1 | 7/16 | 42 | 1, 2, 8~12, 14, 15 | 0.545 | 없음 |
| 5 | 202.5 | 117.2 | 8/16 | 32 | 1, 2, 8~12, 15 | 0.545 | 없음 |

운반 2대만(청소 로봇 없음): seed 1 196.0 s, 2 210.9 s, 3 178.0 s, 위반 없음. 기본 판에는 바닥 물체가 없어 우회는 일어나지 않는다.

- seed 1의 I5_STANDING: 청소 로봇이 긴 줄 5번으로 O_OUT 지점을 쥔 동안 pode2가 OUTBOUND에서 기다리고(ZONE_WAIT O_OUT), pode1이 급행 O_X에 5 s 넘게 섰다. 청소 로봇이 운반 로봇에게 비키는 4-6(단계 3)의 몫으로 남겼다.
- d2(seed 1~5): 청소 로봇이 HALL 줄마다 운반 로봇이 쥔 주차·대기 지점(P-2, W_Q, N_W) 둘레를 지나 모든 줄이 미뤄지고 포기된다(0/9줄, 바퀴 15.1 s). 바퀴가 곧바로 다시 시작해 미룸 사건이 계속 나온다.
- 바닥 물체 판(청소 로봇만, 물체 (0.35, 0.4875)): 첫 바퀴 242.8 s, 13/16줄, 미룸 13, 포기 2·3·5번, 빈틈 1(0.55 m), 덮은 비율 0.574, 위반 없음.

## 7. 리뷰

Codex 세 번(67cfb85..b73f4d4, 수정 1 ce543f6, 수정 2 19496eb). 고친 것: 결과 없이 끝난 줄이 빠짐, 거둔 goal의 늦은 거절·유실 소식, 못 보냄·유실 때 지점이 남음, dock·lane 구역 배치(지점 구역과 선 자리 구역 모두), 스테이션 대기 자리와 줄 당기기에서 집 자리, 성공 결과를 도착 확인 없이 받음(운반 로봇의 `_succeeded`·`_judge_arrival`과 같은 기준으로: 끝에서 `arrival_tolerance_m`보다 멀면 goal과 점유를 그대로 두고 `arrival_settle_s` 동안 새 보고를 기다리고, 그래도 멀면 NOT_AT_TARGET 실패. 줄·우회는 미루고 나머지 다리는 `_goal_failed` 뒤 다시 보낸다). 고치지 않은 것: 기록 머리의 params에 새 값이 들어감(값을 더할 때마다 같고 replay는 아는 값만 쓴다), 지도 밖 위치 또는 쥔 dock 지점 옆 공유 구역 위치의 배치(공유 구역 지점·자리에 섰으면 계약을 어기지 않는다).

네 번째 리뷰(19496eb..e642228, 고침 세 번을 다 쓴 뒤의 재검수)에서 확정 지적 둘이 남았다. 고침 횟수 규칙에 따라 고치지 않고 사용자에게 넘겼다(8절).

## 8. 남은 일

- 4-6 운반 로봇에게 비키기, 4-9 STOP 뒤 이어 하기(단계 3).
- 우회 구동의 controller 비교(RL, MPPI)는 청소 로봇 Nav2 설정으로 판마다 바꾼다.
- d2처럼 줄이 모두 막힌 판에서 바퀴를 쉬게 할지(사건이 계속 나온다).
- 실물·Gazebo로 우회와 빈틈을 본 적 없다.
- **남은 결함(사용자 결정 대기):** 청소 goal이 바깥에서 거둬지면(STOP의 `on_run_mode` → `_cancel_goal`, 멈춤 감시 `_watch_stall` → `_cancel_goal`) `m.goal`만 지워지고 `job.goal_id`, `job.arrival_by`는 남는다.
  - 도착을 기다리는 중(일찍 온 성공 뒤 `arrival_settle_s` 안)에 STOP이 오면, 다음 위치 보고 때 `_clean_report`(`cleaning.py:479-482`)가 `m.goal`을 보지 않고 `_clean_arrival`을 불러 거둔 다리를 완료나 NOT_AT_TARGET으로 처리할 수 있다. bay_in이면 STOP 중 바퀴 끝이 기록될 수 있다.
  - 도착 대기는 `job.arrival_by`에만 적히고 멈춤 감시는 `m.arrival_by`만 보고 건너뛴다(`watch.py:27`). 그래서 대기 중에도 멈춤 판정이 돌아 goal을 거둘 수 있다.
  - 둘 다 일찍 온 성공 뒤 몇 초 안에 STOP이나 멈춤이 겹쳐야 생긴다.
