# RL 정책 계보 (goal_reach, store_arena)

2026-11-21부터 학습한 정책의 출처를 한곳에 모은 기록이다. 보고서·작업 기록·실행 폴더(`run.json`, 체크포인트의 `iter`)에서 확인한
것만 적었고, 증거가 없는 칸은 **모름**이다.

- **숫자는 같은 평가 세트·같은 에피소드 안에서만 견준다.** 세트가 다르면(env 수, 에피소드 수, seed, 지도, 속도 상한, 평가 방법) 같은
  정책도 숫자가 다르다(예: ppo_k 0.12 무작위가 11-22 밤 72.9 %, 11-23 레시피 세트 73.1 %). 세트는 아래 "평가 세트"에 있다.
- **체크포인트(.pt)는 이 PC의 `D:/isaac/runs/goal_reach/...`, `D:/isaac/runs/store_arena/...`에만 있고 git에 없다.** 정책 폴더
  `pode/pode_rl_controller/policies/<이름>/`의 `policy.onnx`와 `policy_spec.json`은 git에 있다.
- **반복 칸**은 "이번 실행의 반복 수 / 체크포인트 `iter`"다. `iter`는 rsl_rl의 반복 번호라 처음부터 N번이면 N−1이고, `--init`·`--resume`로
  이어 갈 때마다 번호 하나가 겹친다(예: ppo_k 500 + 500번 → `iter` 998).
- **브랜치·커밋 칸**의 커밋은 보고서가 학습 커밋이라고 적은 것만 쓴다. 정책 폴더의 `source.commit`은 내보낸(export) 커밋이며 따로 표시했다.
  실행 폴더의 `git/` 기록은 IsaacLab 저장소(`release/3.0.0`, 3939445)라 이 저장소의 커밋이 아니다.
- **지도 판**: v7 지도는 세 번 바뀌었다. (1) 진열대 A와 남쪽 서쪽 진열대의 서쪽 끝이 서로 바뀐 판(main 8d106e5에서 고침,
  `docs/worklog_rl_acd3_2026-11-25.md:8`), (2) 끝을 고쳤지만 진열대 A 두께가 5 mm인 판(`docs/report_2026-11-25_rl_acd4.md:5`),
  (3) 모든 진열대를 4 cm로 고친 판(`docs/report_2026-11-25_rl_acd4.md:4`), 그리고 2026-11-26부터 (4) 줄자로 잰 진열대 v7(main 3ccf2a3f).
  acd2 이전 실행은 8d106e5 이전 지도를 썼지만, 그 사이 yaml이 바뀌었는지는 기록이 없어 "8d106e5 이전(세부 모름)"으로 적었다.

## 계보 표

약어: GR = `docs/worklog_goal_reach_rl_2026-11-21.md`, SA = `docs/worklog_store_arena_2026-11-21.md`,
B = `docs/worklog_store_arena_b_2026-11-21.md`, 4B = `docs/worklog_store_arena_4b_2026-11-21.md`,
4C = `docs/worklog_store_arena_4c_2026-11-21.md`, FU = `docs/worklog_store_arena_4c_followup_2026-11-22.md`,
RT = `docs/worklog_retrain_012_2026-11-22.md`, NR = `docs/report_2026-11-22_night.md`, RW = `docs/worklog_recipe_2026-11-22.md`,
MR = `docs/report_2026-11-23_morning.md`, A1 = `docs/report_2026-11-24_rl_acd.md`, AW = `docs/worklog_rl_acd_2026-11-24.md`,
A3 = `docs/report_2026-11-25_rl_acd3.md`, A3W = `docs/worklog_rl_acd3_2026-11-25.md`, A4 = `docs/report_2026-11-25_rl_acd4.md`.
실행 폴더는 `D:/isaac/runs/`를 뺀 경로다. seed와 설정은 그 폴더의 `run.json`에서 읽었다(없으면 그렇게 적음).
레시피 글자: a = 이전 행동 관측 + 행동 변화 벌점, b = 라이다 이전 프레임, c = 비대칭 actor-critic(critic만 실제 위치·속도),
d = 에피소드마다 최고 속도 U(0.08, 0.16)와 그 관측(RW:12-15). 최고 속도는 11-22부터 0.12 m/s, 그 전은 0.25 m/s(FU:85 B50).

| 이름(실행 폴더) | 날짜 | 브랜치·커밋 | 부모 체크포인트 | 반복(이번/누적) | seed | 지도 판 | 기본과 다른 설정(--set) | 레시피(a/b/c/d/의도) | 보상·랜덤화 변경 | 관측 칸 수 | 평가 세트와 결과(비교 대상, 짝 비교 여부) | Gazebo 결과 | 상태와 이유 | 기록 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| goal_reach base_4096 (`goal_reach/base_4096`) | 11-21 | 모름 | 처음부터(`run.json`에 init·resume 칸 없음, 첫 학습 GR:148) | 300 / iter 299 | 42 | 빈 바닥, 목표 1~2 m | 없음(`spec_overrides` {}) | 없음 | 첫 학습(PPO, 4096 env) | 41 | 256 env × 처음 4판 = 1024판, 결정적: 성공 100 %(GR:122). 학습 곡선에서 22 s에 성공 100 %(GR:116). 평가 seed 모름 | 없음 | 기록용. onnx는 나중에 지움(NR:28) | GR |
| ppo_a (`store_arena/ppo_a/c1`) | 11-21 | 모름 | 처음부터(`run.json` resume "") | 300 / iter 299 | 42 | 매장 경기장, 8d106e5 이전(세부 모름) | 없음 | 없음 | 4a 첫 판 | 41 | 없음 | 없음 | 탈락: 잠금·남은 실행기 혼선으로 버리고 ppo_b로 다시 돌림(B:122) | B |
| store_arena B = 4a (`store_arena/ppo_b/c1`) | 11-21 | 모름 | 처음부터(`run.json` resume "") | 1000 / iter 999 | 42 | 매장 경기장, 상자 0~3개, 8d106e5 이전(세부 모름) | 없음 | 없음 | 경로 위 0.40 m 앞 점을 따라감, 진행 보상은 남은 경로 길이(B:3, B:31) | 41 | 4a 세트(1024 env × 4 = 4096, 결정적): 학습 배치 96.5 %, 못 본 배치(`route_seed=1`) 95.7 %(B:103). seed 모름 | 없음 | 기록용(4a 최종). 배포 기록 없음 | B, SA |
| ppo_c, ppo_d (`store_arena/ppo_c/c1`, `ppo_d/c1`) | 11-21 | 모름 | 4a(ppo_b)에서 이어 감(4B:59-60); 경로는 `run.json`이 없어 모름 | c: 223번(마지막 저장 iter 1200), d: 127번(iter 1100) | 모름(`run.json` 없음) | 매장 경기장 + 사람·카트 | 모름 | 없음 | c: 비키지 않는 mover, d: mover가 기다림 | 41 | 학습 중 성공 c 51 %, d 57 %(4B:59-60) | 없음 | 탈락(4B:59-62) | 4B |
| ppo_e, ppo_f (`store_arena/ppo_e/c1`, `ppo_f/c1`) | 11-21 | 모름 | 처음부터(4B:61-62) | e: 244번, f: 224번(마지막 저장 iter 200) | 모름(`run.json` 없음) | 매장 경기장 + 사람·카트 | 모름 | 없음 | e: mover 기다림, f: 기다림 + 돌아섬 | 41 | 학습 중 성공 e 56 %, f 66 %(4B:61-62) | 없음 | 탈락 | 4B |
| ppo_g = 4b(0.25 m/s) (`store_arena/ppo_g/c1`) | 11-21 | 모름 | 처음부터(`run.json` resume "", 4B:5) | 500 / iter 499 | 42 | 매장 경기장 + 사람·카트, 8d106e5 이전(세부 모름) | 없음 | 없음 | 움직이는 사람·카트(양보 0.15 m, 2 s 뒤 돌아섬, 다리 끝은 경로 밖, B21~B23), `obstacle_layouts` 32 → 96(4B:39) | 41 | 4b 세트(1024 env × 4 = 4096, 결정적): 94.6 %, 못 본 배치 93.6 %; 4a 시험 세트 95.1 %(4a 95.7 %)(4B:5) | 없음 | 폐기: 속도를 0.12로 통일(FU:85 B50) | 4B |
| ppo_h = 4c(0.25 m/s) (`store_arena/ppo_h/c1`, `c2`) | 11-21 | 모름 | 처음부터(`c1/run.json` resume "") | 600(300 × 2) / iter 598 | 42 | 4c 경기장(다른 로봇 0~2대), 8d106e5 이전(세부 모름) | `robot_max=2`, `intent_robots=2` | 의도 | 대본 로봇 0~2대 + 가까운 로봇 2대의 의도 관측 | 55 | 4c 세트(1024 × 4, seed 7, 4C:139): 84.0 %, 같은 세트의 ppo_g 81.0 %, 의도 지움 83.5 %(4C:145-148). 짝 비교 아님(4C:164) | 없음 | 폐기: 속도 통일(FU:85 B50) | 4C, FU |
| ppo_i (`store_arena/ppo_i/c1`) | 11-22 | 모름 | 처음부터로 보이나 증거 없음: 모름(`run.json` 없음, 명령에 `INIT=` 없음 FU:64) | 185번에서 멈춤(FU:69) / 마지막 저장 iter 150 | 모름 | 4c 경기장, 새 로봇 규칙 | `robot_max=2 robot_routes=12 robot_retreat=1 mover_noyield_prob=0.5 intent_robots=2`(FU:65) | 의도 | 새 대본 로봇 규칙 | 55 | 없음(학습 중 83.7 %, FU:69) | 없음 | 폐기: 0.12 결정으로 멈춤(FU:70). 짝 ppo_j는 시작 안 함(FU:71) | FU |
| ppo_k = `store_arena_4b` (`store_arena/ppo_k/c1`, `c2`) | 11-22 | `feat/retrain-012-2026-11-22`; 학습 커밋 모름(export 95e2054, RT:59) | 처음부터(`c1/run.json` resume "", RT:3) | 1000(500 × 2, 53분) / iter 998 | 42 | 4b 경기장, 8d106e5 이전(세부 모름) | 없음(새 기본값: 스침 충돌 0.005 m, 벽 근처 벌점 −0.1, 모서리 출발 0.25, mover 양보 섞기, FU:157-165) | 없음 | 위 새 기본값, 0.12 m/s, 에피소드 62.5 s(RT:17) | 41(정책 형식 2) | 학습 배치 4096판 82.2 %, 못 본 배치 2048판 80.8 %, 528개 출발 스침 0(NR:45-50), seed 7(RT:26-31). 11-22 4c 세트(4096판) 무작위 72.9 %, 마주 오기 45.5 %(NR:63-65). 레시피 세트 73.1 / 47.0 / 72.8(MR:49) | 11-22 v7, O_IN → M_E 6판씩(NR:75-87, RL은 ppo_k로 추정); 11-23 item 8 표(MR:202-222); 11-24·11-25 Q2 → E-3·c_moving 20판(A1:47-49, A3:68-71) | 실물 기본 정책(`tools/test/test_policy_files.py:14`), onnx 커밋 | NR, RT |
| ppo_l (`store_arena/ppo_l/c1`, `c2`) | 11-22 | `feat/retrain-012-2026-11-22`(RT:3); 커밋 모름 | 처음부터(RT:79) | 600(300 × 2, 계획 1200) / iter 598 | 1 | 4c 경기장, 8d106e5 이전(세부 모름) | `robot_max=2`, `robot_priority_prob=0.5` | 없음 | 로봇과 함께 학습 | 41(RT:72) | 11-22 4c 세트(4096판, 짝 아님): 무작위 71.8 %(ppo_k 72.9 %), 마주 오기 43.9 %(45.5 %)(NR:62-65) | 없음 | 기록용: ppo_k를 넘지 못해 내보내지 않음(NR:67) | NR, RT |
| 레시피 ctrl (`store_arena/recipe/t_ctrl`) | 11-22~23 | `feat/recipe-2026-11-22`(시작 092fdc4, RW:3); 학습 커밋 모름 | `ppo_k/c2/model_998.pt`(`run.json` init) | 200 / iter 1197 | 1 | 8d106e5 이전(세부 모름) | `robot_max=2 robot_priority_prob=0.5 handover_start_prob=0.15` | 없음(대조군) | 없음 | 41 | 레시피 세트: 74.0 / 48.9 / 73.5 %(MR:48) | 없음 | 기록용(대조군) | RW, MR |
| 레시피 a, a3, a10 (`recipe/t_a`, `t_a3`, `t_a10`) | 11-22~23 | ctrl과 같음 | `recipe/init_a.pt`(ppo_k를 넓힘) | 200 / iter 1197 | 1 | 같음 | ctrl 설정 + `prev_action_obs=1`; `w_action_rate` −0.05 / −0.15 / −0.5, `w_w_flip` −0.5 / −1.5 / −5.0 | a(1배·3배·10배) | 행동 변화 벌점 | 43 | 레시피 세트(ctrl과 짝): a 74.2 / 48.7 / 73.0, a3 74.0 / 47.3 / 73.3, a10 73.9 / 45.6 / 73.9(MR:50-52) | 없음 | a는 조합에 듦, a3·a10은 탈락(MR:72-73) | RW, MR |
| 레시피 b (`recipe/t_b`) | 11-22~23 | 같음 | `recipe/init_b.pt` | 200 / iter 1197 | 1 | 같음 | ctrl 설정 + `lidar_frames=3 lidar_stride_steps=4` | b | 없음 | 113 | 74.1 / 49.4 / 73.7(MR:53) | 없음 | 탈락(B77, MR:72-73) | RW, MR |
| 레시피 c = `store_arena_c` (`recipe/t_c`) | 11-22~23 | 같음(export f178e50, `policy_spec.json` source) | `recipe/init_c.pt`(= ppo_k model_998, actor 가중치 같음) | 200 / iter 1197 | 1 | 같음 | ctrl 설정 + `critic_privileged=1` | c | 없음 | 41(형식 2, RW:118) | 75.1 / 50.2 / 74.3(+1.1 / +1.3 / +0.8, SE 0.40 / 0.54 / 0.38, MR:54) | item 8(MR:203-223) | 6번의 출발점(B79, MR:77-78), 기록용 | RW, MR |
| 레시피 d (`recipe/t_d`) | 11-22~23 | 같음 | `recipe/init_d.pt` | 200 / iter 1197 | 1 | 같음 | ctrl 설정 + `speed_cap_obs=1 speed_cap_range=0.08,0.16` | d | 진행 보상을 상한별로(RW:57) | 42 | 73.7 / 48.2 / 73.9; 0.08 70.5 / 47.2, 0.16 74.0 / 48.1(MR:55, 67) | 없음 | 탈락(B77) | RW, MR |
| 레시피 a+c (`recipe/t_combo_ac`) | 11-22~23 | 같음 | `recipe/init_combo_ac.pt` | 200 / iter 1197 | 1 | 같음 | ctrl 설정 + `prev_action_obs=1 critic_privileged=1`, a 벌점 1배 | a+c | 행동 변화 벌점 | 43(형식 3) | 74.2 / 46.5 / 73.9(MR:56) | 없음 | B78 규칙의 답이었으나 B79로 쓰지 않음(MR:75-78) | RW, MR |
| 레시피 a+c+d = `store_arena_acd` = acd200 (`recipe/t_combo_acd`) | 11-22~23 | 같음(export f178e50) | `recipe/init_combo_acd.pt`(ppo_k model_998을 44칸으로 넓힘, 새 칸 가중치 0) | 200 / iter 1197 | 1 | 같음 | ctrl 설정 + `prev_action_obs=1 critic_privileged=1 speed_cap_obs=1 speed_cap_range=0.08,0.16 w_action_rate=-0.05 w_w_flip=-0.5` | a+c+d | 행동 변화 벌점 | 44(형식 3) | 74.3 / 48.6 / 73.2(MR:57); 0.08 71.3 / 47.5, 0.16 73.9 / 46.6(MR:68). 소수 둘째 자리 0.12 무작위 74.34, 마주 오기 48.63(A1:19-20) | item 8(MR:204-224) | 기록용. acd2의 레시피 | RW, MR |
| 의도 6번 첫 시도 (`recipe/aborted_ac_*`) | 11-22 | 같음 | a+c(`recipe/aborted_ac_init_*.pt`) | 멈춤(c1의 마지막 저장 iter 1200) | 모름(`run.json` 없음; 폴더 이름에 `_s1`) | 같음 | 모름 | a+c(+의도) | 없음 | 43 | 없음 | 없음 | 탈락: a+c에서 시작했다가 B79로 멈춤, 쓰지 않음(RW:137) | RW |
| 의도 6번: 의도 없음 (`recipe/t_i6_noint_s{1,2,3}_c1`, `_c2`) | 11-22~23 | 같음 | `recipe/t_c/model_1197.pt`(`run.json` init) | seed마다 500(250 × 2) / iter 1695 | 1, 2, 3(seed마다 따로 실행) | 같음 | ctrl 설정 + `critic_privileged=1` | c | 없음 | 41 | 레시피 세트 무작위·마주 오기만: 76.2~76.8 / 49.8~52.1 %(MR:126-128) | 없음 | 기록용(의도 비교의 기준) | RW, MR |
| 의도 6번: 의도 (`recipe/t_i6_int_s{1,2,3}_c1`, `_c2`) | 11-22~23 | 같음 | `recipe/init_i6_int.pt`(`run.json` init; t_c를 넓힘, MR:113) | seed마다 500 / iter 1695 | 1, 2, 3 | 같음 | 위 + `intent_robots=2` | c + 의도 | 없음 | 55 | 의도 없음과 짝: 무작위 −0.77 ± 0.14, 마주 오기 −0.78 ± 0.41 %p(seed 사이 SE, MR:143-144) | 없음 | 탈락: 레시피에 넣지 않기를 권고(MR:291), 다시 하려면 관측·보상을 바꾼 새 실험(MR:292) | RW, MR |
| 의도 6번: 의도 + 우선순위 (`recipe/t_i6_intprio_s{1,2,3}_c1`, `_c2`) | 11-22~23 | 같음 | `recipe/init_i6_intprio.pt` | seed마다 500 / iter 1695 | 1, 2, 3 | 같음 | 위 + `intent_priority=1` | c + 의도 + 우선순위 | 없음 | 57 | 무작위 −0.56 ± 0.27, 마주 오기 −1.64 ± 0.65 %p(MR:145-146, 약 2.5 SE로 잡음 안이라 할 수 없음) | 없음 | 탈락: 같은 권고(MR:291) | RW, MR |
| acd2 seed 1, 2 (`acd24/t_acd_s1_c1`·`c2`, `t_acd_s2_c1`·`c2`) | 11-24 | `feat/rl-acd-train-2026-11-24`(시작 9c58d67, AW:3); 학습 커밋 모름 | `recipe/init_combo_acd.pt`(AW:38) | 500(250 × 2) / iter 1496 | 1, 2 | v7 진열대 끝 바뀐 판(A3W:8) | a+c+d 설정(acd200과 같음, AW:27-29) | a+c+d | 행동 변화 벌점 | 44 | 일곱 칸 평균 62.22 / 62.45 %(seed 3은 62.83, AW:61) | 없음 | 탈락: B85로 seed 3을 고름(AW:61) | A1, AW |
| acd2 = `store_arena_acd2` (`acd24/t_acd_s3_c1`, `c2`) | 11-24 | 같음(export 9c58d67, `policy_spec.json` source) | `recipe/init_combo_acd.pt`(c1 `run.json` init; c2는 c1 이어 감) | 500(250 × 2) / iter 1496 | 3 | v7 진열대 끝 바뀐 판(A3W:8) | a+c+d 설정 | a+c+d(의도 없음) | 행동 변화 벌점 | 44(형식 3) | 11-24 일곱 칸(seed 셋 평균): 어느 칸도 4b보다 낮지 않음, 0.12 무작위 74.75 ± 0.32 대 4b 73.14, 분당 부호 바뀜 80.3 → 45.9(A1:9, 19) | 11-24 Q2 → E-3·c_moving 20판: 둘 다 20/20(4b 19/20), Q2 → E-3 판별 몸 여유 중앙값 3.3 대 4b 6.5 cm(A1:47-50) | 실물 후보로 올리기를 권함(A1:67); 사용자 결정 기록 없음. acd3로 이어짐 | A1, AW |
| acd3 형제 t_map (`acd3/t_map`) | 11-25 | `feat/rl-acd3-2026-11-25`(A3:3); 학습 커밋 모름 | `acd24/t_acd_s3_c2/model_1496.pt` | 250 / iter 1745 | 3 | v7 끝 고친 판(진열대 A 5 mm) | a+c+d 설정 | a+c+d | 지도만 | 44 | 다섯 칸 63.30(acd2 62.94, 짝)(A3:39) | 없음 | 탈락: 1 %p 안이면 t_wall 우선 규칙(A3:59) | A3, A3W |
| acd3 형제 t_lidar (`acd3/t_lidar`) | 11-25 | 같음(319ec95, A3W:21) | `acd24/t_acd_s3_c2/model_1496.pt` | 250 / iter 1745 | 3 | 같음 | a+c+d 설정 + `lidar_scale_range=0.95,1.03` | a+c+d | lidar 거리 배율 0.95~1.03(관측에만) | 44 | 다섯 칸 62.99(A3:39) | 없음 | 탈락(같은 규칙) | A3, A3W |
| acd3 = `store_arena_acd3` (`acd3/t_wall`) | 11-25 | 같음(export 4553dc4, A3:93) | `acd24/t_acd_s3_c2/model_1496.pt`(`run.json` init) | 250 / iter 1745 | 3 | v7 끝 고친 판(진열대 A 5 mm, A3:4, A4:5) | a+c+d 설정 + `near_wall_m=0.08 w_near_wall=-0.2` | a+c+d | 벽·진열대 0.08 m 안 벌점 −0.2 | 44(형식 3) | 다섯 칸 63.01(acd2 62.94, 짝 차이 모두 2 SE 안, A3:39, 45-49), 분당 부호 바뀜 감소(A3:34-37) | Q2 → E-3·c_moving 20판 모두 20/20(4b·acd2도 20/20); Q2 → E-3 몸 여유 중앙값 0.024 대 4b 0.058 m, 평균 차 −0.021(SE 0.005) 2 SE 밖(A3:68-81) | 실물 후보(커밋 c190452a 메시지 "the real-robot candidate"; A4:18 acd3 유지 권고). 사용자 결정은 저장소 밖 기록에 있다: Drive 문서 「전체 할일 요약」의 "RL 실물 후보: 끝: acd3로 정함" 줄 | A3, A3W |
| acd4 = s4 (`acd4/t_s4`) | 11-25 | `feat/rl-acd4-2026-11-25`(A4:3); 학습 커밋 모름 | `acd3/t_wall/model_1745.pt`(`run.json` init) | 150 / iter 1894 | 3 | v7 4 cm 진열대 판(A4:4) | t_wall과 같음 | a+c+d | 없음(지도만) | 44(체크포인트 첫 층) | 네 칸 60.32(같은 지도의 acd3 60.14, 짝 차이 넷 모두 2 SE 안, A4:13-14, 46-51) | 없음(A4:57) | 기록용: onnx로 내보내지 않음(A4:6), 후보를 바꿀 근거 없음(A4:17) | A4 |

## 계보 그림

```
처음부터 ── goal_reach base_4096 (300)
처음부터 ── ppo_a (300, 버림)
처음부터 ── ppo_b = 4a (1000) ──┬── ppo_c (버림)
                                 └── ppo_d (버림)
처음부터 ── ppo_e, ppo_f (버림)
처음부터 ── ppo_g = 4b 0.25 (500, 폐기)
처음부터 ── ppo_h = 4c 0.25 (600, 폐기)
모름     ── ppo_i (185번에서 멈춤, 폐기)
처음부터 ── ppo_l (600, 기록용)
처음부터 ── ppo_k = store_arena_4b (1000, iter 998)
              ├── t_ctrl (200)
              ├── init_a ── t_a, t_a3, t_a10 (200)
              ├── init_b ── t_b (200)
              ├── init_d ── t_d (200)
              ├── init_combo_ac ── t_combo_ac (200) ── aborted_ac_* (멈춤)
              ├── init_c ── t_c = store_arena_c (200, iter 1197)
              │               ├── t_i6_noint_s1..3 (500)
              │               ├── init_i6_int ── t_i6_int_s1..3 (500)
              │               └── init_i6_intprio ── t_i6_intprio_s1..3 (500)
              └── init_combo_acd ──┬── t_combo_acd = store_arena_acd (200, iter 1197)
                                   ├── t_acd_s1, t_acd_s2 (500, 탈락)
                                   └── t_acd_s3 = store_arena_acd2 (500, iter 1496)
                                         ├── t_map, t_lidar (250, 탈락)
                                         └── t_wall = store_arena_acd3 (250, iter 1745)
                                               └── t_s4 = acd4 (150, iter 1894, onnx 없음)
```

`init_*.pt`는 `widen_checkpoint.py`로 시작 체크포인트를 새 입력 칸에 맞게 넓힌 것이다(새 칸 첫 층 가중치 0, RW:24). `init_c.pt`와
`init_combo_acd.pt`의 actor 첫 41칸·둘째 층이 `ppo_k/c2/model_998.pt`와 같음을 확인했다(둘 다 `iter` 998).

## 평가 세트

같은 세트 안에서만 견준다. 세트마다 정책을 env 번호로 짝지을 수 있는지가 다르다.

| 세트 | 쓴 곳 | 칸 | env·에피소드 | seed | 짝 비교 |
|---|---|---|---|---|---|
| goal_reach | GR:122 | 빈 바닥 | 256 env × 처음 4판 = 1024 | 모름 | 아님 |
| 4a·4b 세트 | B:95-103, 4B:82-88 | 학습 배치, 못 본 배치(`route_seed=1`), 4a 시험 세트(4B:110) | 1024 env × 4 = 4096, 결정적 | 모름 | 아님 |
| 4c 세트(0.25) | 4C:139-148 | 4c 경기장, 못 본 배치, 의도 지움 | 1024 × 4 | 7 | 아님(4C:164) |
| 11-22 밤 ppo_k | NR:43-46, RT:26-31 | 학습 배치 4096판, 못 본 배치 2048판 | 1024 env | 7 | 아님 |
| 11-22 밤 4c 세트(0.12) | NR:58-65 | 무작위, 마주 오기(`robot_max=2 robot_priority_prob=0.5`) | 4096판(1024 × 4) | 7 | 아님 |
| 레시피 세트(11-23) | MR:41, RW:79 | 0.12 무작위, 마주 오기(`robot_headon=1`), 인계 출발(`handover_start_prob=1`); d·acd는 0.08·0.16 무작위·마주 오기도 | 4096 env × 첫 에피소드(`play.py --episodes 1`) | 7 | 예, 0.12 칸끼리 env 번호로. 0.08·0.16은 짝 아님 |
| 11-24 일곱 칸 | A1:15-39, AW:20, 32 | 0.12 무작위·마주 오기·인계 출발, 0.08·0.16 무작위·마주 오기. 지도: 진열대 끝 바뀐 v7. 4b는 `--capped` | 4096 × 첫 에피소드 | 7 | 예(acd는 seed 셋 평균 ± seed 사이 SE). 4b·ctrl·c·acd200의 0.12는 11-22·23 파일 재사용(A1:62) |
| 11-25 다섯 칸 | A3:28-49, A3W:23-29 | 0.12·0.08 무작위·마주 오기 + lidar 배율 0.958 고정 0.12 무작위. 지도: 끝 고친 v7(A 5 mm). acd2도 다시 잼 | 4096 × 첫 에피소드 | 7 | 예 |
| 11-25 네 칸 | A4:29-51 | 0.12·0.08 무작위·마주 오기. 지도: 4 cm 진열대 v7 | 4096 × 첫 에피소드 | 7 | 예(s4와 acd3). "acd3 옛 지도" 열은 짝 아님(A4:12, 31) |
| 11-26 기준 | `pd-rl-base` `docs/rl_tracks_2026-11-26/jobs_base.txt` | 0.12 무작위·마주 오기(`speed_cap_range=0.12,0.12`). 지도: 줄자 진열대 v7(main 3ccf2a3f) | 4096 × 첫 에피소드 | 7 | 같은 지도·명령의 판끼리만 |

견줄 수 없는 예: acd2의 0.12 무작위 74.75(11-24, seed 셋 평균, 바뀐 지도)와 74.44(11-25, seed 3, 고친 지도). acd3의 다섯 칸 63.01과
네 칸 60.14·60.32(칸과 지도가 다름). Gazebo는 날마다 빌드와 월드가 달라 같은 보고서 안에서만 견준다(A3:64, A4:58).

## 2026-11-26 밤 (진행)

지도는 줄자로 잰 진열대 v7(main 3ccf2a3f)이다. 계획과 판정 기준은 `docs/worklog_rl_tracks_2026-11-26.md`, 작업 목록은
`docs/rl_tracks_2026-11-26/jobs_*.txt`, 결과 표는 `docs/rl_tracks_2026-11-26/results/tables.md`(도구 `n26_tables.py`)다.
실행 폴더는 `D:/isaac/runs/store_arena/n26/<이름>`이고, 학습마다 `run_config.json`이 있다(커밋과 지도 sha256 포함).

학습은 모두 acd3(`acd3/t_wall/model_1745.pt`)에서 출발해 4096 env로 300번(이어 학습 둘은 600번), seed 3이다. 설정은 acd3과 같은 a+c+d +
`near_wall_m=0.08 w_near_wall=-0.2`에 아래 "다른 점"만 더했다. 관측은 라이다 이력을 쓴 실행(116칸)을 빼고 모두 44칸(형식 3)이다.

| 이름(실행 폴더) | 날짜 | 브랜치·커밋 | 부모 체크포인트 | 반복(이번/누적) | seed | 지도 판 | 기본과 다른 설정(acd3 설정에 더한 것) | 레시피 | 보상·랜덤화 변경 | 관측 칸 수 | 평가 세트와 결과(비교 대상, 짝 여부) | Gazebo 결과 | 상태와 이유 | 기록 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| t_ctrl (`n26/t_ctrl`) | 11-26 | `feat/rl-tracks-2026-11-26` 52719475 | `acd3/t_wall/model_1745.pt` | 300 / 2044 | 3 | 줄자 진열대 v7 | 없음 | a+c+d | 없음(같은 설정으로 300번 더: 다른 실험의 대조) | 44 | rand 75.95 %(acd3와 짝 차이 -0.37 %p, SE 0.34: 2 SE 안), aisle 55.35 %, demo1 43.33 %, demo2 39.70 %. head는 평가 전 | 없음 | 끝. 대조로만 쓴다(acd3와 차이가 없다) | 작업 기록 2절, 8c절 끝 |
| t_std (`n26/t_std`) | 11-26 | 같음 52719475 | 같음 | 300 / 2044 | 3 | 같음 | `--init_std 0.3` | a+c+d | 탐색 std(v, w)를 0.3으로 되돌려 시작 | 44 | aisle 67.90 %(acd3 +11.28 %p, SE 0.66. t_ctrl +12.55, 0.66), demo1 52.49 %(acd3 +8.84, 0.67. t_ctrl +9.16, 0.66), demo2 48.49 %(acd3 +8.18, 0.61. t_ctrl +8.79, 0.60). 모두 같은 에피소드 짝. rand 79.27 %(acd3 +2.95, 0.47), head 52.54 %(acd3 +5.13, 0.61) | 열한째 묶음(17절): `f_cart` 통로 판 5/5, 충돌 0. `g_demo` 도착 2/5(판정하지 않음. 도착하지 못한 세 판은 모두 닿았다: 둘은 닿은 뒤에 섰고 하나는 서지 않고 틈 남쪽으로 내려갔다). `j_cross`, `k_cross`: 닿지 않고 지난 판 없음. 충돌 판의 합 15/20(t_demo_hist 14: "차이가 보이지 않는다"), 도착 판 15/20. 플릿 시뮬 PASS(17b절) | 끝. `store_arena_std`로 저장소에 넣었다(`nav_rl:=rl_std`, 시뮬 비교용). Isaac 시연 칸의 성공이 t_demo_hist와 2 SE 안이고 충돌이 더 적다(시간 초과는 더 많다). rand 조건을 채웠다. 다만 rand에서 w 부호 바뀜이 t_ctrl보다 늘어(47.2 대 39.8회/분) 2절에 미리 적은 기준대로 실물 후보로는 올리지 않는다(작업 기록 17a절). 실물에서는 돌려 보지 않았다 | 작업 기록 2절, 8c절 끝, 17절 |
| t_sysid (`n26/t_sysid`) | 11-26 | 같음 e51add4b | 같음 | 300 / (진행) | 3 | 같음 | `cmd_delay_steps_range=0,2 cmd_lag_tau_range=0.0,0.12 cmd_gain_v_range=0.95,1.05 cmd_gain_w_range=0.85,1.15` | a+c+d | 명령 응답 랜덤화(지연, 1차 지연, 이득) | 44 | (진행) | 없음 | 진행 | 작업 기록 3절, `docs/report_2026-11-26_rl_sysid.md` |
| t_dyn (`n26/t_dyn`) | 11-26 | 같음 4015354f | 같음 | 300 / 2044 | 3 | 같음 | `near_dyn_m=0.10 w_near_dyn=-1.0` | a+c+d | 움직이는 물체·로봇 0.10 m 안 벌점 | 44 | (진행) | 없음 | 학습 끝, 평가 대기 | 작업 기록 5절 |
| t_gam (`n26/t_gam`) | 11-26 | 같음 4015354f | 같음 | 300 / (진행) | 3 | 같음 | `--gamma 0.995 --init_std 0.3` | a+c+d | 할인율 0.99 → 0.995 | 44 | (진행) | 없음 | 진행 | 작업 기록 5절 |
| t_demo (`n26/t_demo`) | 11-26 | 같음 f21329b7 | 같음 | 300 / 2044 | 3 | 같음 | `obstacle_demo_share=0.5 mover_cross_prob=0.5 robot_cleaner_prob=0.5` | a+c+d | 시연 조건을 절반 섞음: 통로 상자(경로가 상자를 지남), 틈을 건너는 사람, 청소 로봇 | 44 | rand 75.02 %(acd3 -1.29, SE 0.37: 2 SE 넘게 낮다), head 45.07 %(acd3 -2.34, 0.46), aisle 55.98 %, demo1 41.65 %(t_ctrl -1.68, 0.47), demo2 39.36 %(t_ctrl -0.34, 0.41). 같은 에피소드 짝 | 없음 | 탈락: 시연 칸이 오르지 않았고(H-c를 채우지 못함) rand가 떨어졌다. 저장소에 넣지 않는다 | 작업 기록 8절, 8c절 끝, 13절, `docs/report_2026-11-26_rl_demo_*.md` |
| t_demo_std (`n26/t_demo_std`) | 11-26 | 같음 f21329b7 | 같음 | 300 / 2044 | 3 | 같음 | t_demo + `--init_std 0.3` | a+c+d | 위 + 탐색 std를 0.3으로 되돌려 시작 | 44 | rand 77.51 %(acd3 +1.20, SE 0.48), head 48.10 %(acd3 +0.68, 0.59), aisle 70.41 %(acd3 +13.79, 0.67. t_std +2.51, 0.54), demo1 51.25 %(acd3 +7.59, 0.66. t_std -1.25, 0.59), demo2 47.80 %(acd3 +7.50, 0.59. t_std -0.68, 0.56). 같은 에피소드 짝 | `j_cross`, `k_cross` 5판씩(13b, 13c절, 판정하지 않음): 사람과 닿지 않고 도착한 판 0/5, 0/5. 벤치에만 올렸고 저장소에는 넣지 않았다 | 끝. t_demo_hist의 대조. 후보로 올리지 않는다(demo1에서 t_demo_hist보다 2 SE 넘게 낮다) | 작업 기록 8절, 8c절 끝, 12절, 13절 |
| t_caps (`n26/t_caps`) | 11-26 | 같음 c6a6bd02 | 같음 | 300 / (진행) | 3 | 같음 | `--caps_temporal 0.04 --caps_spatial 0.2 --caps_sigma 0.1`, `w_action_rate=0 w_w_flip=0` | a+c+d(부드러움 벌점 없음) | 부드러움을 보상 벌점 대신 손실(행동 평균)에 건다 | 44 | (진행) | 없음 | 진행 | 작업 기록 10절, `docs/report_2026-11-26_rl_caps.md` |
| t_caps_std (`n26/t_caps_std`) | 11-26 | 같음 c6a6bd02 | 같음 | 300 / (진행) | 3 | 같음 | t_caps + `--init_std 0.3` | a+c+d(부드러움 벌점 없음) | 위 + 탐색 std를 0.3으로 되돌려 시작 | 44 | (진행) | 없음 | 진행 | 작업 기록 10절 |
| t_sym (`n26/t_sym`) | 11-26 | 같음 3e770458 | 같음 | 300 / (진행) | 3 | 같음 | `--symmetry aug` | a+c+d | 좌우 거울 증강(표본마다 거울 사본을 붙여 학습) | 44 | (진행) | 없음 | 진행 | 작업 기록 11절, `docs/report_2026-11-26_rl_symmetry.md` |
| t_demo_hist (`n26/t_demo_hist`) | 11-26 | 같음 51b51d39 | `n26/init_acd3_hist3.pt` (acd3에 `widen_checkpoint.py --add_lidar_history`) | 300 / 2044 | 3 | 같음 | t_demo_std 설정 + `lidar_frames=3 lidar_stride_steps=4` | a+b+c+d | 라이다 이력 + std 0.3 + 시연 조건 절반 | 116 | rand 77.37 %(acd3 +1.05, SE 0.47), head 51.12 %(acd3 +3.71, 0.68. t_demo_std +3.03, 0.62), aisle 72.19 %(acd3 +15.58, 0.71. t_std +4.30, 0.57), demo1 52.81 %(acd3 +9.16, 0.68. t_demo_std +1.56, 0.56. t_std +0.32, 0.60), demo2 48.27 %(acd3 +7.96, 0.63). demo1에서 건넘이 경로 위인 에피소드의 사람 충돌 15.3 %(t_demo_std 17.3 %, 짝 차이 -2.07 %p, SE 0.68). 같은 에피소드 짝 | 아홉째 묶음(13절): `f_cart` 통로 판 5/5, 충돌 0. `g_demo` 도착 4/5(acd3 3/5, 판정하지 않음). 보충 묶음 `j_cross`, `k_cross`(13b, 13c절): 닿지 않고 지난 판 없음(acd3도 같다). 플릿 시뮬 세 번 PASS(7c절). 열한째 묶음(17절): `f_cart` 5/5, 충돌 0. `g_demo` 도착 1/5. `j_cross`, `k_cross`: 닿지 않고 지난 판 없음. 충돌 판의 합 14/20, 도착 판 11/20 | 끝. 후보: `store_arena_demohist`로 저장소에 넣었다(`nav_rl:=rl_demohist`). 실물에서는 돌려 보지 않았다 | 작업 기록 7c절, 8c절 끝, 12절, 13절 |
| t_demo_hist2 (`n26/t_demo_hist2`) | 11-26 | 같음 74e82c53 | `n26/t_demo_hist/model_2044.pt` (`--resume`) | 600 / 2643 | 3 | 같음 | t_demo_hist와 같다(이어 학습이라 `--init`, `--init_std` 없음) | a+b+c+d | 없음(같은 설정으로 600번 더) | 116 | demo1 54.27 %(부모 +1.46, SE 0.54. acd3 +10.62, 0.68), demo2 50.05 %(부모 +1.78, 0.51), aisle 73.95 %(부모 +1.76, 0.49), rand 78.54 %(부모 +1.17, 0.41. acd3 +2.22, 0.48). 충돌은 네 칸 모두 부모보다 2 SE 넘게 적다. demo1의 건넘 경로 위 사람 충돌은 17.0 %로 부모(15.3 %)보다 높다(+1.74, 0.66). head는 49.27 %로 부모(51.12 %)보다 낮다(−1.86, 0.57. 다른 로봇과의 충돌 +4.22, 0.55). 같은 에피소드 짝 | 열한째 묶음(17절): `f_cart` 통로 판 5/5, 충돌 0. `g_demo` 도착 5/5(판정하지 않음. 카트 접촉 0/5, 사람 접촉 3/5). `j_cross`, `k_cross`: 닿지 않고 지난 판 없음(네 정책 모두). 충돌 판의 합 13/20(부모 14: "차이가 보이지 않는다"), 도착 판 18/20(부모 11). 새 사본 확인(17d절): 플릿 시뮬 `nav_rl:=rl_demohist2` PASS(주문 8건, 215 s. 선택 명령 전의 첫 주행은 RPP) | 끝(학습 4510 s). "더 길게 학습하면 오른다". 17절에 미리 적은 기준을 채워 `store_arena_demohist2`로 저장소에 넣었다(`nav_rl:=rl_demohist2`, onnx sha256 앞 12자 `2b58f8fbb442`). `store_arena_demohist`는 그대로 둔다. 건너는 사람은 Isaac과 Gazebo 모두에서 나아지지 않았다. 실물에서는 돌려 보지 않았다 | 작업 기록 15절, 17절, 17c절, 17d절 |
| t_demo_std2 (`n26/t_demo_std2`) | 11-26 | 같음 74e82c53 | `n26/t_demo_std` 마지막 모델 (`--resume`) | 600 / (진행) | 3 | 같음 | t_demo_std와 같다(이어 학습) | a+c+d | 없음(같은 설정으로 600번 더) | 44 | (진행) | 없음 | 진행 | 작업 기록 15절 |
| t_demo_hist_dyn (`n26/t_demo_hist_dyn`) | 11-26 | 같음 e9e19c69 | `n26/init_acd3_hist3.pt` (t_demo_hist와 같다) | 300 / (진행) | 3 | 같음 | t_demo_hist 설정 + `near_dyn_m=0.10 w_near_dyn=-1.0` | a+b+c+d | 위 + 움직이는 것(사람, 움직이는 카트, 로봇) 0.10 m 안 벌점 | 116 | (진행) | 없음 | 진행 | 작업 기록 16절 |
| t_hist_std (`n26/t_hist_std`) | 11-26 | 같음 51b51d39 | `n26/init_acd3_hist3.pt` (t_demo_hist와 같다) | 300 / 2044 | 3 | 같음 | t_demo_hist 설정에서 시연 조건 셋(`obstacle_demo_share`, `mover_cross_prob`, `robot_cleaner_prob`)을 뺀 것 | a+b+c+d | 라이다 이력 + std 0.3, 시연 조건 없음(시연 조건 × 이력 2×2의 빈 칸) | 116 | demo1 51.27 %(t_std -1.22, SE 0.59), demo2 47.12 %(-1.37, 0.57), aisle 68.65 %(+0.76, 0.57), rand 77.98 %(-1.29, 0.41), head 49.27 %(-3.27, 0.61). 충돌은 다섯 칸 모두 t_std보다 2 SE 넘게 많다(demo1 +3.64, 0.65). 같은 에피소드 짝 | 없음(조건을 채우지 못해 올리지 않았다) | 끝(학습 2122 s). head에서 가설과 반대, "이력만으로도 충돌이 늘었다". 내보내지 않았다 | 작업 기록 18절 |
| t_std_s4 (`n26/t_std_s4`) | 11-26 | 같음 52719475 | 같음 | 300 / (진행) | 4 | 같음 | t_std와 같다(`--seed 4`뿐) | a+c+d | seed 반복: 같은 설정을 seed만 바꿔 다시 학습(실행 사이의 흔들림을 잰다) | 44 | (진행) | 없음 | 큐에서 대기(학습 전). 내보내지 않고 저장소에 넣지 않는다 | 작업 기록 22절 |
| t_demo_hist_s4 (`n26/t_demo_hist_s4`) | 11-26 | 같음 51b51d39 | `n26/init_acd3_hist3.pt` (acd3에 `widen_checkpoint.py --add_lidar_history`) | 300 / (진행) | 4 | 같음 | t_demo_hist와 같다(`--seed 4`뿐) | a+b+c+d | seed 반복: 같은 설정을 seed만 바꿔 다시 학습(실행 사이의 흔들림을 잰다) | 116 | (진행) | 없음 | 큐에서 대기(학습 전). 내보내지 않고 저장소에 넣지 않는다 | 작업 기록 22절 |

평가(모두 `--num_envs 4096 --episodes 1 --seed 7`, a+c+d 관측, `robot_max=2 robot_priority_prob=0.5 speed_cap_range=0.12,0.12`).
칸 이름: rand(무작위), head(`robot_headon=1`), realrand(실측 명령 응답 고정: `cmd_delay_steps_range=1,1 cmd_lag_tau_range=0.065,0.065`),
corner·tee·entry(`robot_scenario=<이름> mover_prob=0`). 같은 칸의 판끼리만 짝 비교한다.

| 이름 | 체크포인트 | 칸 | 결과 | 상태 |
|---|---|---|---|---|
| e_acd3_rand | acd3 | rand | 성공 76.32 %, 충돌 23.27 %, 시간 초과 0.42 %. 충돌의 가까운 것: 벽·진열대 114, 상자 45, 움직이는 물체 436, 로봇 358. w 부호 바뀜 43.3회/분 | 끝 |
| e_acd3_head | acd3 | head | 성공 47.41 %, 충돌 52.17 %. 가까운 것: 벽·진열대 195, 상자 40, 움직이는 물체 414, 로봇 1488. 45.1회/분 | 끝 |
| e_acd3_realrand, e_acd3_rand_chk, e_acd3_rand_chk2, e_acd3_corner, e_acd3_tee, e_acd3_entry | acd3 | 이름대로 | (진행) | 진행 |
| e_ctrl_rand | t_ctrl | rand | 성공 75.95 %, 충돌 23.29 %, 시간 초과 0.76 %(acd3와 짝 차이 -0.37 %p, SE 0.34) | 끝 |
| e_ctrl_head, e_std_rand, e_std_head, e_sysid_*, e_dyn_*, e_gam_* | 위 학습의 마지막 모델 | rand, head(+ sysid·ctrl은 realrand) | (진행) | 진행(e_std_rand, e_std_head는 끝났다: 작업 기록 17a절) |
| e_{acd3, ctrl, std, demo, demostd, demohist}_{aisle, demo1, demo2} | acd3 또는 위 학습의 마지막 모델 | aisle(통로 상자 하나, 사람·로봇 없음), demo1·demo2(상자 1·2 + 틈을 건너는 사람 + 운반 1·청소 1). 설정은 작업 기록 8절 | 성공 %는 아래 표. 짝 차이와 충돌 종류는 작업 기록 8c절 끝, 12절 | 끝 |
| e_{demo, demostd, demohist}_{rand, head} | 위 학습의 마지막 모델 | rand, head | 성공 %는 아래 표. 짝 차이는 작업 기록 12절, 13절 | 끝 |
| e_demohist2_{demo1, rand, demo2, aisle}, e_demohist2m_demo1 | t_demo_hist2의 마지막 모델(m은 중간 점 `model_2350.pt`) | 이름대로 | 성공 54.27, 78.54, 50.05, 73.95 %. 중간 점 demo1 54.76 %(부모 +1.95, SE 0.52). 작업 기록 15절 | 끝 |
| e_demohist2_head | t_demo_hist2의 마지막 모델 | head(다섯 칸 표를 채우는 값, 작업 기록 15절) | 성공 49.27 %. 부모(51.12 %)보다 −1.86 %p(SE 0.57)로 2 SE 넘게 낮다. 다른 로봇과의 충돌 +4.22 %p(0.55), 벽·진열대 −1.81 %p(0.38). 판정에는 쓰지 않는다 | 끝 |
| e_histstd_{demo1, aisle, rand, head, demo2} | t_hist_std의 마지막 모델 | 이름대로 | 성공 51.27, 68.65, 77.98, 49.27, 47.12 %. 작업 기록 18절 | 끝 |
| e_stds4_{demo1, aisle, rand, head}, e_demohists4_{demo1, aisle, rand, head} | t_std_s4, t_demo_hist_s4의 마지막 모델 | 이름대로(작업 기록 22절) | (진행) | 큐에서 대기 |
| e_caps_*, e_capsstd_*, e_sym_*, e_demostd2_*, e_demohistdyn_* | 위 학습의 마지막 모델 | 작업 기록 10, 11, 15, 16절 | (진행) | 진행 |
| e_{acd3, std, demohist}_demo1c08 | acd3 또는 위 학습의 마지막 모델 | demo1에서 속도 상한만 0.08로 낮춘 것(작업 기록 19절) | 성공 44.63, 53.86, 52.05 %. 건넘 경로 위 사람 충돌은 상한 0.12보다 +2.54, +3.30, +6.27 %p(SE 1.27, 1.22, 1.19)로 세 정책 모두 늘었다. 시간 한도와 다른 로봇의 속도가 달라 성공 %는 다른 줄과 견주지 않는다 | 끝 |
| e_{acd3, std, demohist}_aislet | acd3 또는 위 학습의 마지막 모델 | aisle 평가를 궤적 1024개로 다시 돌린 것(시간 초과의 끝 동작, 작업 기록 20절) | 추적된 시간 초과 76, 100, 37건 가운데 제자리에서 돌거나 오간 것이 39, 53, 28건, 서 있던 것이 34, 45, 9건. 기존 aisle 평가와 에피소드 결과가 42, 73, 27개 달라(시뮬이 똑같이 되풀이되지 않는다) 이 분포를 기존 평가에 옮겨 말하지 않는다 | 끝 |
| e_{acd3, std, demohist}_demo1t | acd3 또는 위 학습의 마지막 모델 | demo1 평가를 궤적 1024개로 다시 돌린 것(건너는 사람이 닿기 전에 보인 시간, 작업 기록 21절) | 추적된 사람 충돌 115, 97, 81건 가운데 3 s 넘게 보이던 사람이 92, 60, 50건, 1~3 s 보이던 사람이 21, 34, 31건, 보인 지 1 s 안은 0건, 서 있는 로봇에 사람이 닿은 것이 2, 3, 0건. 기존 demo1 평가와 에피소드 결과가 104, 146, 72개 달라 이 분포를 기존 평가에 옮겨 말하지 않는다 | 끝 |
| e_demohist_demo1t{f20a30, f20a60, f15a60} | t_demo_hist의 마지막 모델 | demo1 평가(궤적 1024개)에 앞 막기 `--front_stop` 0.20 m·30도, 0.20 m·60도, 0.15 m·60도를 얹은 것(작업 기록 23절) | 성공 35.74, 17.80, 28.27 %(앞 막기 없는 평가 52.54 %), 시간 초과 39.45, 63.48, 47.49 %. 건넘 경로 위 사람 충돌 23.89, 12.39, 22.67 %(없는 평가 15.88 %). 통로 안 통과 13.79, 0.00, 0.32 %(없는 평가 36.25 %). 사람 충돌이 준 설정은 모두 성공이 크게 줄어 플러그인 옵션은 만들지 않았다 | 끝 |
| e_demohist_demo1t{w26a60, w26a60n} | t_demo_hist의 마지막 모델 | demo1 평가(궤적 1024개)에 긴 창의 다가옴 규칙 `--closing_stop 0.26,60 --closing_tune lookback_s=1.0,spread_deg=4,closing_mps=0.08,min_rays=2`를 얹은 것. 둘째는 막기가 보는 스캔에 잡음 0.02 m(`noise_m=0.02`)를 더한 것(작업 기록 25절) | 끝 | 건넘 경로 위 사람 충돌 15.88 % → 4.05 / 4.19 %, 성공 52.54 % → 57.84 / 57.62 %, 통로 안 통과 36.25 % → 42.87 / 42.30 %. 둘 다 미리 적은 기준을 통과 |
| e_demohist_demo1t{c26a60, c26a90, p30a60, p30a90} | t_demo_hist의 마지막 모델 | demo1 평가(궤적 1024개)에 다가옴 규칙 `--closing_stop` 0.26 m·60도, 0.26 m·90도와 사람 위치를 아는 규칙 `--person_stop` 0.30 m·60도, 0.30 m·90도를 얹은 것(작업 기록 24절) | 끝 | 건넘 경로 위 사람 충돌 15.88 % → 13.24 / 17.06 / 5.00 / 6.31 %, 성공 52.54 % → 53.20 / 51.98 / 56.40 / 55.93 %. 다가옴 ±60도와 사람 위치 둘이 미리 적은 기준을 통과 |

끝난 평가의 성공 %(4096 에피소드. 전체 표는 `docs/rl_tracks_2026-11-26/results/tables.md`):

| 정책 | rand | head | aisle | demo1 | demo2 |
|---|---|---|---|---|---|
| acd3 | 76.32 | 47.41 | 56.62 | 43.65 | 40.31 |
| t_ctrl | 75.95 | 평가 전 | 55.35 | 43.33 | 39.70 |
| t_std | 79.27 | 52.54 | 67.90 | 52.49 | 48.49 |
| t_demo | 75.02 | 45.07 | 55.98 | 41.65 | 39.36 |
| t_demo_std | 77.51 | 48.10 | 70.41 | 51.25 | 47.80 |
| t_demo_hist | 77.37 | 51.12 | 72.19 | 52.81 | 48.27 |
| t_demo_hist2 | 78.54 | 49.27 | 73.95 | 54.27 | 50.05 |
| t_hist_std | 77.98 | 49.27 | 68.65 | 51.27 | 47.12 |

## 기록 규칙

1. 학습을 시작하기 전에 이 표에 **진행** 줄을 넣고, 끝나면 채운다(탈락한 판도 지우지 않는다).
2. 줄마다 위 칸을 모두 채운다: 이름(실행 폴더), 날짜, 브랜치·커밋, 부모 체크포인트, 반복(이번/누적), seed, 지도 판, 기본과 다른 설정,
   레시피, 보상·랜덤화 변경, 관측 칸 수, 평가 세트와 결과(비교 대상, 짝 여부), Gazebo 결과, 상태와 이유, 기록.
3. 학습마다 실행 폴더에 `run_config.json`을 쓴다(`train.py`).
4. `policy_spec.json`의 `source`에 `parent_checkpoint`, `iterations_this_run`, `iterations_total`, `seed`, `map_layout`,
   `run_config_sha256`을 넣는다(`learning/goal_reach/export_policy.py`가 체크포인트 폴더의 `run_config.json`, 없으면 `run.json`에서 채움).
   `parent_checkpoint`가 ""이고 `iterations_this_run`이 있으면 처음부터 학습한 것이고, 둘 다 비어 있으면 모르는 것이다.
5. 후보 정책의 `.pt`, onnx, `run_config.json`은 Drive 폴더 `RL_정책_백업`에 올린다.
6. 같은 평가 세트·같은 에피소드 안에서만 견준다.
7. 학습이 끝날 때마다 바뀐 줄을 보고한다.
