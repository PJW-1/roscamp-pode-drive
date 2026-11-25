# 청소 로봇: 최신 v7 지도(진열대 4 cm)에서 다시 잰 결과와 고친 결함 둘 (2026-11-25)

브랜치 `feat/cleaner-yield-main-2026-11-25`. `feat/cleaner-yield-2026-11-25`(dd2f03c, 비키기 4-6과 STOP 4-9)에
main 58b0725(v7 진열대 세 개 두께 4 cm)를 합치고(2246a33), 그 지도에서 나온 결함 둘을 고쳤다(9b9920a).
main에는 합치지 않았다(사용자가 말할 때).

## 요약

- 최신 지도의 청소 경로는 9셀 19줄, 한 바퀴 124.7 s(회전 제외)다. 앞 지도는 8셀 17줄, 116.8 s였다.
- 최신 지도에서 결함 둘이 드러나 고쳤다.
  - **연결 길 미룸:** 청소 로봇이 줄 끝에서 진열대 쪽으로 몇 mm 더 가서 서면 다음 줄로 가는 길을 하나도 보내지 못했다.
    혼자 돈 판 seed 5는 19줄 중 2줄만 끝냈다.
  - **비키기 되풀이:** Gazebo에서 청소 로봇이 비킨 자리에 덜 가서 멈추면 2.5 cm 옆 점으로 비키기를 다시 보냈다.
    한 판에 `CLEAN_YIELD`가 131번 났다.
- 고친 뒤 헤드리스 기본값(seed 1~20): 연결 길 미룸 0번(고치기 전 seed 6~20에서 166번), `CLEAN_STUCK` 1번(고치기 전 11번).
  혼자 돈 판은 5판 모두 19/19줄이다. 덮은 비율과 운반 지연은 고치기 전과 비슷하다(20판 평균 덮은 비율 0.656 → 0.644).
- Gazebo(판 하나씩, 주문 8건): 마지막 출고는 청소 없음 209.1 s, 고치기 전 303.0 s, 고친 뒤 273.1 s다.
  고친 뒤에도 청소 로봇이 운반을 64 s(31 %) 늦췄다. 운반이 도는 동안 청소 로봇은 바퀴마다 19줄 중 1~7줄만 끝냈고,
  남은 미룸은 모두 운반 로봇과의 겹침 판정(남이 쥔 지점, 남의 base, 가까이 선 로봇, 운반 로봇 길과 만남)이다.
- 정할 것은 5절에 있다. 가장 큰 것은 운반이 바쁠 때 겹침 판정을 어디까지 완화할지(계획 Q17)다.

## 1. 최신 main 병합 (2246a33)

- origin/main 58b0725를 충돌 없이 합쳤다. 병합에서 더한 것은 `pc/fleet_core/fleet_core/data/coverage_v7.yaml`
  다시 만들기뿐이다(`python3 -m fleet_core.coverage --layout v7 --write`, 17줄 → 19줄). d2는 바뀌지 않았다.
- 시험(병합 직후): fleet_core 1564 passed, 7 xfailed. `sim/fake_robots/test/test_world*.py` 142 passed.
  `tools/test` 333 passed, 3 skipped. Codex 검수: 결함 없음.
- 경로 그림 `docs/img/coverage_v7.png`와 `docs/청소_로봇.md`의 수치를 이 지도로 고쳤다.

## 2. 고친 결함 (9b9920a)

### 2-1. 줄 끝에서 reach 안에 서면 연결 길을 하나도 못 보냄

- **증상:** 최신 지도의 미룸 표(고치기 전)에서 청소 로봇 혼자 돈 판이 seed에 따라 2~19줄만 끝냈다. 미룸은 다섯 판 합계
  114번이 모두 path("the joined path is not clear after rounding")였고, seed 5 한 판이 68번이다. Gazebo 판에서도 둘째 바퀴부터 바퀴마다 1줄만 끝냈다.
- **원인:** 줄 1은 남쪽 서쪽 진열대 모서리(0.70, 0.3475) 옆 (0.6125, 0.3125)에서 끝난다. 이 점은 진열대에서 0.094 m라
  reach(0.093 m)를 겨우 넘는다. seed 5에서 청소 로봇은 (0.621, 0.317)에 섰고, 이 자리는 진열대에서 0.085 m다.
  칸은 빈칸이라 `CleanPlan.way`는 지금 자리부터 경로를 검사했고, 첫 점이 reach 안이라 어느 줄로 가는 길이든 검사에 걸렸다.
  로봇은 움직이지 않으니 나머지 17줄이 모두 미뤄졌다가 포기됐다.
  재현: `PYTHONPATH=pc/fleet_core:sim/fake_robots python3 docs/cleaner_yield_main_2026-11-25/why_path_defer.py 5`.
- **고침:** 지금 자리가 빈칸이어도 막힌 것과 벽에서 reach보다 가까우면, 칸 밖에 섰을 때처럼 가장 가까운 빈칸 중심부터
  검사한다. 지금 자리에서 그 중심까지의 짧은 이동(1.8 cm 안)은 검사하지 않는다. 칸 밖에 섰을 때 이미 쓰던 방식이다.
- **시험:** `test_zone_cleaning.py::test_a_stop_just_inside_reach_still_joins_the_next_lines`.
  고치기 전 코드에서는 줄 2부터 다리가 None이라 실패한다.

### 2-2. 비킨 자리에 덜 가서 멈추면 비키기를 되풀이함

- **증상:** 고치기 전 Gazebo 판에서 청소 로봇이 약 9 s 동안 "stands in pode2's way"로 비키기 goal을 10번 다시 보냈다.
  목표점은 그때마다 2~3 cm씩 바뀌었다. 10 s 뒤 집으로 가려다 `CLEAN_STUCK`이 났고, 판 전체에서 `CLEAN_YIELD`가 131번 났다.
  헤드리스 가짜 로봇은 goal 끝점에 정확히 서므로 이 문제가 보이지 않았다.
- **원인:** 비킬 점은 운반 로봇 띠에서 반폭 + 몸 반지름(0.235 m) 바로 밖의 가장 가까운 격자점이다. Nav2는 goal에서
  xy 허용치 0.05 m(`general_goal_checker`) 안에서 멈춘다. 덜 가서 멈춘 자리는 다시 띠 안이라 "띠 안에 서 있음"이 다시 났고,
  새 비킬 점은 2.5 cm 옆이라 Nav2가 곧바로 성공을 돌려줬다.
- **고침:** 새 파라미터 `clean_yield_margin_m`(0.05, Nav2 xy 허용치와 같게)만큼 더 떨어진 점에서 고른다.
  `fleet_params.yaml`과 `docs/plan_flip_2026-11-24/fleet_params_plan_split.yaml`에 같은 값을 넣었다.
- **시험:** `test_zone_clean_yield.py::test_the_yield_point_leaves_room_for_stopping_short`
  (비킬 점 거리, 다리를 따라 오다 0.05 m 덜 가서 성공해도 다시 비키지 않음), `test_yield_parameters_are_loaded`.
- **Codex 검수:** 결함 없음. 검사 범위가 준 곳 하나(2-1의 짧은 이동)는 계약대로이고 커밋 메시지에 이유가 있다고 확인했다.
  좁은 곳에서 비킬 점을 못 찾아 집으로 가는 경우가 늘 수 있다는 지적이 있었다. 아래 3절의 `CLEAN_STUCK` 수로는 늘지 않았다.

시험(고친 뒤): fleet_core 1569 passed, 7 xfailed. 세계 시험 142 passed. `tools/test` 333 passed, 3 skipped.

## 3. 헤드리스 미룸 표 (운반 2 + 청소 1, v7, 고친 지도)

표 전체: `docs/cleaner_yield_main_2026-11-25/defer_before_fix.md`, `defer_fix.md`(조건 16개 × seed 1~5),
`default_seeds6_20_before_fix.md`, `default_seeds6_20_fix.md`. 도구: `tools/cleaner_defer_sweep.py`.

| 조건 | seed | 끝낸 줄 | 덮은 비율 평균 | 주문 끝 평균 | 지연 평균/최대 | 최소 거리 | path 미룸 | `CLEAN_STUCK` |
|---|---|---|---|---|---|---|---|---|
| 혼자, 고치기 전 | 1~5 | 2~19/19 | 0.814 | - | - | - | 114 | 0 |
| 혼자, 고친 뒤 | 1~5 | 19/19 | 1.000 | - | - | - | 0 | 0 |
| 기본값, 고치기 전 | 1~5 | 3~14/19 | 0.708 | 199.6 s | 6.6 / 12.2 s | 0.171 m | 31 | 2 |
| 기본값, 고친 뒤 | 1~5 | 7~12/19 | 0.558 | 195.2 s | 2.2 / 9.6 s | 0.199 m | 0 | 0 |
| 기본값, 고치기 전 | 6~20 | 3~10/19 | 0.638 | 196.7 s | 2.5 / 9.3 s | 0.171 m | 166 | 9 |
| 기본값, 고친 뒤 | 6~20 | 4~16/19 | 0.672 | 198.3 s | 4.1 / 22.1 s | 0.176 m | 0 | 1 |

- 청소 없음은 주문 끝 평균 192.9 s(seed 1~5), 194.2 s(seed 6~20)다. 지연은 같은 seed의 청소 없음 판과의 차이다.
- seed 1~5만 보면 덮은 비율이 0.708 → 0.558로 내려갔지만, seed 1이 14줄 0.850에서 7줄 0.465가 된 것처럼 판마다
  끝내는 줄이 달라 크게 흔들린다. seed 6~20에서는 0.638 → 0.672로 올랐고, 20판 평균은 0.656 → 0.644다.
- 기본값이 아닌 조건(`ahead10`, 남의 지점·base를 1 m 앞까지만 봄)의 seed 4는 고친 뒤 900 s까지 주문을 끝내지 못했다.
  청소 로봇이 HALL 동쪽 막다른 곳((1.86, 0.19), Q2·Q1 옆)에서 pode1의 동쪽에 갇혔다. 비킬 점이 없어 집으로 가려
  했지만 집으로 가는 길의 Q2를 pode1이 쥐고 있었다(`CLEAN_STUCK`). pode1은 청소 로봇 앞에서 막혀 막힘 표시, 다른 길,
  60 s 뒤 다시 시도를 끝없이 되풀이했다. 고치기 전에도 같은 지도에서 `ahead05` seed 5가 서로 막혀 멈췄다.
  비키기 보고서(`docs/report_2026-11-25_cleaner_yield.md`)가 알려진 한계로 적은 막다른 곳 막힘과 같은 꼴이다.

## 4. Gazebo 판 (v7, 시나리오 `gz_v7`, 주문 8건, 판 하나씩)

결과 파일: `docs/cleaner_yield_main_2026-11-25/gazebo/{no_cleaner,before_fix,fix}/`(summary, safety, events).

| 판 | 마지막 출고(시뮬 시각) | 청소 없음 대비 | `CLEAN_YIELD` | 최소 거리 | 위반 |
|---|---|---|---|---|---|
| 청소 없음 | 209.1 s | 기준 | 0 | 0.207 m | 0 |
| 청소 있음, 고치기 전 | 303.0 s | +93.9 s (+45 %) | 131 | 0.194 m | 0 |
| 청소 있음, 고친 뒤 | 273.1 s | +64.0 s (+31 %) | 13 | 0.199 m | 0 |

- 판 감시(`scenario_monitor`)가 낸 판 시간(224.8 s, 338.9 s, 427.8 s)은 비교에 쓰지 않는다. 감시는 시나리오 시간
  초과가 아니면 모든 주문이 끝나고 모든 로봇이 주차해야 끝나는데(`outcome.py` `_complete`), 청소 로봇은 바퀴를 마쳐야
  주차하므로 판 시간에 청소 바퀴가 섞인다.
- 청소 바퀴(고친 뒤): 1바퀴 4/19줄(덮은 비율 0.140), 2바퀴 4/19(0.132), 3바퀴 1/19(0.010), 4바퀴 7/19(0.194),
  5바퀴 15/19(0.749). 5바퀴는 운반이 거의 끝난 뒤다. 고치기 전에는 1바퀴 5/19(0.525) 뒤 2~5바퀴가 모두 1/19였다.
- 고친 뒤 판 전체(5바퀴)의 미룸 이유는 운반 로봇이 쥔 지점 127번, 운반 로봇 base가 가까이 지남 101번, 운반 로봇이
  가까이 서 있음 20번, 달리는 다리가 운반 로봇 길과 만남 1번이다. path 미룸은 0번이다.
- 헤드리스 지연(평균 2~4 s)과 Gazebo 지연(64 s)의 차이가 크다. Gazebo 로봇이 느리고 멈춤이 길어 청소 로봇이 쥔
  지점(O_IN, O_OUT, M_E, E-3 등)을 운반 로봇이 기다리는 시간이 늘어난 것으로 보지만, 판이 하나라 원인 몫은 재지 않았다.

## 5. 정할 것

1. **운반이 바쁠 때의 겹침 판정(계획 Q17):** 지금 기본값(남이 쥔 지점과 남의 base를 모두 봄)으로는 Gazebo에서 운반이
   도는 동안 거의 청소하지 못한다. 헤드리스에서 `r3_only`(남이 쥔 지점, 남의 base, 다리와 운반 로봇 띠의 만남(보낼 때 검사와 달리는 중 비키기)을
   보지 않음. 공유 구역 밖 지점, 가까이 선 로봇, 운반 로봇이 기다리는 지점은 그대로 봄)는 덮은 비율 0.876,
   지연 6.0 / 10.6 s, 위반 0이다(seed 1~5, 고친 뒤). 완화한 조건을 Gazebo에서 몇 판 돌려 볼지 정해야 한다.
2. **동쪽 막다른 곳 막힘(계획 Q18):** 운반 로봇이 Q2·Q1로 올 때 청소 로봇이 그 동쪽에 있으면 둘 다 움직이지 못할 수 있다.
   계획 4-6은 이때 운반 goal이 실패할 수 있다고 봤지만, 실제로는 60 s마다 다시 시도하며 끝나지 않는다.
   운반 로봇이 그쪽으로 오기 전에 청소 로봇이 막다른 곳의 줄을 미루게 할지 등을 정해야 한다.
3. **줄을 거의 못 끝낸 바퀴(계획 Q15):** Gazebo 고친 뒤 판의 3바퀴는 1/19줄로 24 s 만에 끝나고 곧바로 다음 바퀴를
   돌았다. Q15는 줄을 하나도 못 끝낸 바퀴만 다루므로, 이처럼 거의 못 끝낸 바퀴까지 쉬었다 돌게 할지도 함께 정해야 한다.
4. **Gazebo 판 시간:** 판 감시가 청소 로봇 주차를 기다리는 것은 그대로 두고, 운반 지표는 마지막 출고 시각으로 본다.
5. main 병합은 사용자가 말할 때 한다.

## 6. 화면으로 보기

이 브랜치를 따로 빌드한 설치 폴더(`~/ros2camp/podedrive_ws/install_cy`)가 있다. 격리 검사(`sim/fleet_gz_sim/fleet_gz_sim/isolation.py`)는
ROS_DOMAIN_ID 43~101, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, 고정 피어 없음, RMW `rmw_fastrtps_cpp`, Fast DDS 설정 변수 없음,
작업 폴더에 `DEFAULT_FASTRTPS_PROFILES.xml`·`DEFAULT_FASTDDS_PROFILES.xml` 없음을 본다.

```bash
source ~/ros2camp/podedrive_ws/install_cy/local_setup.bash
export ROS_DOMAIN_ID=57 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE \
      FASTDDS_DEFAULT_PROFILES_FILE FASTDDS_ENVIRONMENT_FILE FASTDDS_BUILTIN_TRANSPORTS
cd /tmp && ros2 launch fleet_gz_sim gz.launch.py layout:=v7 scenario:=gz_v7 sim_dock_stub:=true cleaners:=pode3 rviz:=fleet
```
