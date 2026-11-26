# RL 정책 빨리 써 보기 (저장소만으로)

이 문서는 다른 PC나 로봇에서 저장소만 받아 RL 주행 정책을 돌리는 순서를 모은 것이다. 자세한 설명은 각 절 끝의
문서에 있다. 정책 파일(`policy.onnx`, `policy_spec.json`)은 저장소에 들어 있으므로 따로 받을 것이 없다.

## 1. 어느 브랜치를 받나

`main`에는 아직 RL 정책 파일이 없다(main 병합은 사용자가 정한다). 아래 가운데 하나를 받는다.

| 브랜치 | 들어 있는 것 | 쓰는 곳 |
|---|---|---|
| `feat/rl-policy-onnx-2026-11-26` | main + 정책 다섯(`store_arena_4b`, `c`, `acd`, `acd2`, `acd3`) + 시뮬 인자 `nav_rl:=rl_acd3` | 시뮬만. main과 가장 가깝다 |
| `feat/rl-real-optin-2026-11-26` | 위 + 실물 배포·시작 스크립트의 RL 켜기(`WITH_RL`, `RL_POLICY`). 플러그인 코드는 위와 같다 | 시뮬과 실물 |
| `feat/rl-tracks-2026-11-26` | 위 전부 + 11-26 작업(플러그인 출력 평활 옵션, 학습 코드, 측정 도구, 문서) + 시연 조건 후보 정책 `store_arena_demohist`(`nav_rl:=rl_demohist`) + 시뮬 비교용 정책 `store_arena_std`(`nav_rl:=rl_std`) + demohist를 600번 더 학습한 정책 `store_arena_demohist2`(`nav_rl:=rl_demohist2`) + 시뮬 시제품 `aisle_first` | 전부. 새 옵션은 기본값에서 꺼져 있다 |

```bash
git fetch origin
git checkout feat/rl-tracks-2026-11-26   # 또는 위 표의 다른 브랜치
```

## 2. 빌드에 필요한 것

`pode_rl_controller`는 ONNX Runtime이 있어야 빌드된다. 없으면 이 패키지에서 `colcon build`가 멈춘다.

시뮬을 빌드하려면 제조사 로봇 모델 `pinky_description`이 먼저 빌드되어 있어야 한다(`fleet_gz_sim`이 그것을 찾는다).
받는 곳과 빌드 순서는 `sim/fleet_gz_sim/README.md`의 "준비 (한 번)"에 있고, 아래는 그것을 `~/pinky_deps_ws`에 빌드해 둔 경우다.

```bash
sudo apt update && sudo apt install ros-jazzy-onnxruntime-vendor ros-jazzy-nav2-rotation-shim-controller
source /opt/ros/jazzy/setup.bash
source ~/pinky_deps_ws/install/local_setup.bash   # pinky_description (시뮬을 빌드할 때)
cd <작업공간> && colcon build
source install/setup.bash                          # 새 셸마다 위 두 줄과 이 줄을 다시 한다
colcon test --packages-select pode_rl_controller --event-handlers console_direct+   # 정책마다 추론 일치 시험
```

sudo를 못 쓰면 `.deb`를 `dpkg -x`로 풀어 `CMAKE_PREFIX_PATH`에 넣는다(`pode/pode_rl_controller/README.md`).

## 3. 시뮬에서 달려 보기

```bash
# 플릿 시뮬: 로봇마다 RL controller 둘(FollowPathRL = shim으로 감싼 정책, FollowPathRLAlone)을 더한다
ros2 launch fleet_gz_sim gz.launch.py nav_rl:=rl_acd3
# 로봇 하나를 RL로 바꾸기(되돌리기는 data: FollowPathGoal). 그다음 보내는 Nav2 goal부터 RL이다
# Nav2의 선택 노드가 transient_local·reliable로 듣기 때문에 QoS 두 옵션을 빼면 보내지 못하고 기다린다
ros2 topic pub --times 3 --rate 2 --keep-alive 1 --qos-durability transient_local --qos-reliability reliable \
  /pode1/controller_selector std_msgs/msg/String "{data: FollowPathRL}"
```

- **바뀌었는지 꼭 확인한다.** RL이면 Nav2 goal로 앞으로 가는 동안 controller_server 로그에
  `FollowPathRL: 20 calls, infer mean ...` 줄이 1초마다 나온다(`nav_log_level:=info`. 제자리에서 도는 동안에는 나오지 않는다).
  goal 하나가 끝나도록 **지금 실행의 로그에** 그 줄이 한 번도 없으면 RL로 가지 않은 것으로 보고 선택을 다시 보낸다. 그래도
  없으면 같은 로그에서 `Failed to load`와 controller_server의 오류를 찾는다. 한 번만 보내는 형태(`--once`)는 11-26 플릿 시뮬
  세 번의 선택 아홉 번 가운데 한 번이 오류 없이 끝나고도 먹지 않았다(작업 기록 7c절. 까닭은 가리지 못했다).
- 플릿 시뮬은 launch 10 s쯤 뒤에 첫 주문 주행이 시작된다. 선택 명령이 닿기 전에 시작된 Nav2 goal은 기본 controller(RPP)로
  가고, 시뮬에서는 달리는 중에 선택이 닿으면 그 goal 안에서 RL로 바뀌었다(작업 기록 17d절). RPP는 앞이 막혔을 때만 경고를
  내므로, RPP 경고가 없다는 것만으로 RL이었다고 보지 않는다. RL 추론 로그가 난 구간만 RL이다.

- `nav_rl:=rl`은 정책 `store_arena_4b`, `nav_rl:=rl_acd3`은 `store_arena_acd3`다.
- `nav_rl:=rl_demohist`는 `store_arena_demohist`다(`follow_path_rl_demohist.yaml`).
- `nav_rl:=rl_demohist2`는 `store_arena_demohist2`다(`follow_path_rl_demohist2.yaml`).
- `nav_rl:=rl_std`는 `store_arena_std`다(`follow_path_rl_std.yaml`). 시뮬 비교용이다: 미리 정한 기준(w 부호 바뀜)에 걸려 실물
  후보로는 올리지 않았다(작업 기록 17a절).
- `store_arena_demohist`는 시연 조건 후보다: 시뮬에서만 확인했고 실물에서는 돌려 본 적 없다
  (근거는 `docs/worklog_rl_tracks_2026-11-26.md` 12~13절).
- `store_arena_demohist2`는 `store_arena_demohist`를 같은 설정으로 600번 더 학습한 것이다: 시뮬에서만 확인했고 실물에서는
  돌려 본 적 없다. Gazebo 열한째 묶음에서 미리 적은 기준을 채워 넣었다(근거는 작업 기록 15절, 17절, 17c절). 건너는 사람
  앞에서 서지 못하는 것은 `store_arena_demohist`와 같다. Isaac의 마주 오는 로봇 칸(head)은 `store_arena_demohist`보다
  낮았다(성공 49.3 대 51.1 %, 작업 기록 15절).
- 로봇 한 대로 컨트롤러를 견주는 묶음은 `tools/nav_bench/run_bench.sh`다. RL 컨트롤러 설정은
  `sim/fleet_gz_sim/nav_overrides/follow_path_rl_acd3.yaml`을 `BENCH_OVERRIDES`로 준다.
- 인자 설명: `sim/fleet_gz_sim/README.md`.

통로 안 회피 시연 시나리오(로봇 한 대, `feat/rl-tracks-2026-11-26`에만 있다)는 아래처럼 다시 돌린다. 시나리오는
`docs/rl_tracks_2026-11-26/gz_demo.yaml`에 있다: `f_cart`(통로 카트 하나), `g_demo`(카트 둘과 틈을 오가는 사람), `h_block`(통로
막힘), `j_cross`·`k_cross`(로봇 앞을 쉬지 않고 오가며 건너는 사람), `l_once`·`m_once`(로봇 앞을 한 번만 건너는 사람).

```bash
# acd3와 demohist를 시나리오마다 5판씩. aisle_first:=true는 "통로 먼저, 막히면 우회"(시뮬 전용 시제품)다
# plan_keep:=false는 길 유지(keeper)를 끈다: 작업 기록의 열두째 묶음까지는 모두 keeper가 꺼진 채로 돌았다
BENCH_LOCK= BENCH_DOMAIN=72 BENCH_REPEATS=5 BENCH_LAUNCH_ARGS="aisle_first:=true plan_keep:=false" \
BENCH_CONFIG=$PWD/docs/rl_tracks_2026-11-26/gz_demo.yaml \
BENCH_OVERRIDES=$PWD/docs/rl_tracks_2026-11-26/results/gz_demo_cand/overrides.yaml \
BENCH_CONTROLLERS="FollowPathRLShimACD3 FollowPathRLShimDemoHist" \
  tools/nav_bench/run_bench.sh ~/rl_demo_out f_cart g_demo k_cross
python3 tools/nav_bench/report.py --config docs/rl_tracks_2026-11-26/gz_demo.yaml --legs ~/rl_demo_out/legs --out ~/rl_demo_out/report
```

- `BENCH_LOCK=`(빈 값)은 학습 PC의 무거운 작업 잠금을 쓰지 않게 한다. 도메인은 43~101만 받는다.
- 판별 결과는 `~/rl_demo_out/report/legs.csv`(성공, 통로 안인지, 몸 여유 최솟값)다. 지금까지의 결과와 읽는 법은
  `docs/worklog_rl_tracks_2026-11-26.md` 8d, 13, 13b, 13c절에 있다. 건너는 사람(`j_cross`, `k_cross`)은 두 정책 모두 닿는다.
- 길 유지(`plan_keep`)는 main 441aaf93부터 기본으로 켜진다. 이 브랜치도 그 main을 merge했으므로 `plan_keep:=false`를 주지
  않으면 keeper가 켜진 채로 돈다. 후보 정책을 keeper와 같이 돌린 결과는 작업 기록 28절에 있다.
- 플러그인의 다가옴 막기 옵션(`closing_stop_range`, 기본 꺼짐. `pode/pode_rl_controller/README.md`)은 여기에 켜는 법을 적지
  않는다: `j_cross`, `k_cross`에서 켜고 돌린 10판 가운데 사람과 닿지 않고 도착한 판이 없었다(첫 건넘에서는 섰지만 그 뒤에
  닿았고, 도착 시간은 1.8~3.7배였다. 작업 기록 26절). 한 번만 건너는 사람(`l_once`, `m_once`)의 결과는 30절에 있다.
- 플러그인의 1초 로그 줄(추론 시간, 다가옴 막기가 막은 주기 수)은 INFO다. 시뮬 launch의 Nav2 로그 수준은 기본이 warn이라
  보려면 `BENCH_LAUNCH_ARGS`에 `nav_log_level:=info`를 더한다.

## 4. 실물에서 달려 보기

**이 길은 아직 로봇에서 돌려 본 적이 없다.** PC에서는 스크립트를 대역 ssh·rsync로만 시험했다. 처음에는 바퀴를 띄운
채로 시작한다. 아무 옵션도 주지 않으면 배포와 시작은 예전과 똑같다.

```bash
# PC(저장소 루트): 로봇에 ONNX Runtime과 shim이 있는지 먼저 보고, 없으면 설치 방법만 찍고 끝난다
WITH_RL=1 tools/real_pode/deploy.sh pinky@<로봇 주소>
# 로봇
cd ~/nav2fleet_ws && colcon build
RL_POLICY=store_arena_acd3 ~/nav2fleet_real/start.sh pode1 0.12 0.10 1.5708
# RL로 바꾸기(PC는 SUPER=1 source tools/real_pode/pc_env.sh 뒤). 그다음 보내는 Nav2 goal부터 RL이다
ros2 topic pub --times 3 --rate 2 --keep-alive 1 --qos-durability transient_local --qos-reliability reliable \
  /pode1/controller_selector std_msgs/msg/String "{data: FollowPathRL}"
```

- 기본 controller는 그대로 RPP(FollowPathGoal)다. 고정 경로(pode_fsm의 FollowPath)는 늘 RPP다.
- **RL로 바뀌었는지 로그로 확인한 뒤에 결과를 적는다.** goal 하나가 끝나도록 지금 실행의 로그에 `FollowPathRL: N calls, infer ...`
  줄이 없으면 RL로 가지 않은 것으로 본다(3절의 설명과 같다. 보는 명령은 `tools/real_pode/README.md`의 8번이고, 예전 실행의
  로그가 같은 폴더에 남아 있으므로 가장 새 로그만 본다).
- 정책 이름은 저장소의 정책 폴더 이름이면 된다(`RL_POLICY=store_arena_demohist`도 같은 방식이다). `store_arena_demohist`는
  라이다 이력을 더 보는 정책이고 실물에서 돌려 본 적이 없다.
- RL controller는 `start.sh`의 속도 인자를 따르지 않고 정책의 최고 속도 0.12 m/s까지 낸다.
- 전체 순서, 되돌리기, 시간 로그 보는 법, 적어 둘 것, 알려진 위험: `tools/real_pode/README.md`의 "RL 정책으로 달려 보기".
- 근거와 남은 결정: `docs/report_2026-11-26_rl_real_optin.md`, `docs/report_2026-11-24_rl_real_prep.md`.

Pi 4에서 RL 추론과 MPPI의 계산 시간을 주행 없이 재려면 `tools/real_pode/compute_bench.sh`를 쓴다
(`docs/report_2026-11-26_pi4_compute.md`의 점검표. `feat/rl-tracks-2026-11-26`에만 있다).

## 5. 정책을 새로 학습했을 때

체크포인트는 `learning/store_arena/checkpoints/`(후보만)와 학습 PC의 `D:/isaac/runs/`에 있다. 새 정책을 저장소에
넣는 순서는 `pode/pode_rl_controller/README.md`(정책 폴더, 시험 fixture, 추론 일치 시험 목록, 시뮬 덮어쓰기 파일)와
`learning/store_arena/POLICIES.md`의 기록 규칙에 있다.

`feat/rl-tracks-2026-11-26`에서 `store_arena_demohist`와 `store_arena_std`를 넣은 순서다(둘째는 이 절만 보고 넣으면서 빠진
곳을 채웠다). 정책 이름 `store_arena_<이름>`을 `<NAME>`, 체크포인트를 `<CK>`로 적는다.

1. 내보내기(Isaac Lab을 깐 python으로, CPU에서 돌렸다):
   ```bash
   CUDA_VISIBLE_DEVICES= <Isaac Lab python> learning/goal_reach/export_policy.py --checkpoint <CK> \
     --out pode/pode_rl_controller/policies/<NAME> --fixtures pode/pode_rl_controller/test/fixtures/<NAME> \
     --goal-rule lookahead --lookahead-m 0.40 --follow-window 20 --route-spec learning/store_arena/route_spec.py \
     --source-branch <브랜치> --source-commit <학습 코드 커밋> \
     --set prev_action_obs=1 --set speed_cap_obs=1 --set speed_cap_range=0.08,0.16
   ```
   `--set`은 학습 때 관측을 정한 설정과 같아야 한다. 라이다 이력 정책(demohist)은 `--set lidar_frames=3 --set lidar_stride_steps=4`를
   더했다. `--source-commit`에 넣을 학습 코드 커밋은 실행 폴더 `run_config.json`의 `git.commit`에 있다(실행 폴더의 `git/`은
   Isaac Lab 쪽 차이만 담는다). 같은 곳의 `git.branch`는 학습을 돌린 작업 사본의 브랜치라서, `--source-branch`에는 그 커밋이
   들어 있는 통합 브랜치 이름을 적었다. 내보내기가 끝에 적는 onnxruntime 대 torch 최대 차이를 확인한다.
2. `pode/pode_rl_controller/CMakeLists.txt`의 추론 일치 시험 목록과 그 README의 정책 표에 `<NAME>`을 더한다. 표의 칸은 관측 수
   (`policy_spec.json`의 `obs_dim`), 체크포인트, `policy.onnx`의 sha256 앞 12자(`sha256sum`), 한 줄 설명이다.
3. 시뮬에서 고르려면 `sim/fleet_gz_sim/nav_overrides/follow_path_rl_<이름>.yaml`(관측이 같은 정책의 파일에서 첫 줄 주석과
   `policy` 두 곳만 바꾼 것)을 만들고, `gz.launch.py`의 `RL_FILES`와 `nav_rl` 인자 설명 글, `sim/fleet_gz_sim/README.md`의
   `nav_rl` 줄에 더한다. 시험도 맞춘다: `sim/fleet_gz_sim/test/test_launch_wiring.py`의 선택 이름 목록과 그 아래 정책 이름
   표(고른 이름 → 정책 이름) 둘 다, `tools/test/test_policy_files.py`의 덮어쓰기 파일 목록.
4. `learning/store_arena/checkpoints/<NAME>/`에 체크포인트, 실행 폴더의 `run.json`과 `run_config.json`을 두고 `index.json`과
   그 폴더 `README.md`의 표에 줄을 더한다. `index.json`의 항목은 앞 항목과 같은 키 여덟 개다: `policy`(정책 이름),
   `file`(이 폴더 기준 체크포인트 경로), `sha256`과 `bytes`(체크포인트 파일의 해시와 크기), `iter`(누적 반복 수, 파일 이름의
   숫자), `obs_dim`, `copied_from`(베껴 온 원래 경로), `run_json`(이 폴더 기준 `run.json` 경로). `README.md`는 표 말고 앞쪽의 정책을 늘어놓은 두 문장도 같이 고친다. 이 문서의 1절 표와 3절에도 이름을 더한다. `POLICIES.md` 계보표를 채우고
   Drive `RL_정책_백업`에 같은 파일을 올린다.
5. 새 파일을 모두 `git add`한 뒤에 시험한다(`test_policy_files.py`는 git이 추적하는 파일만 본다. 커밋까지 해야 다른 PC에서 받을
   수 있다): `python -m pytest tools/test/test_policy_files.py learning/store_arena/test_checkpoints.py sim/fleet_gz_sim/test/test_launch_wiring.py`,
   그리고 2절의 colcon build와 `colcon test --packages-select pode_rl_controller`. `test_launch_wiring.py`는 ROS를 source한
   셸에서만 돈다(ROS가 없는 python에서는 파일째 건너뛴다).

## 6. 확인된 것과 아닌 것 (2026-11-26)

- GitHub에서 `feat/rl-policy-onnx-2026-11-26`을 새로 받은 사본만으로 WSL에서 빌드했고, 정책 다섯의 추론 일치 시험이
  통과했다(onnxruntime C++ 대 torch 차이 1.4e-6, 건너뜀 0).
- 그 사본의 빌드만 source한 셸에서 Gazebo로 acd3를 달렸다: `follow_path_rl_acd3.yaml`, 로봇 한 대, Q2→E-3 두 판 모두
  SUCCEEDED(12.5 s, 15.2 s).
- 같은 빌드로 플릿 시뮬도 돌렸다: `gz.launch.py layout:=v7 gui:=false rviz:=none sim_dock_stub:=true nav_rl:=rl_acd3`,
  세 로봇의 `controller_selector`에 `FollowPathRL`. 세 로봇 모두 정책 `store_arena_acd3`를 읽고 RL로 주행했으며
  시나리오(주문 8건)가 PASS로 끝났다(218 s).
- `feat/rl-tracks-2026-11-26`의 `store_arena_demohist`: WSL에서 추론 일치 시험이 통과했고, 플릿 시뮬
  `nav_rl:=rl_demohist`를 두 번 돌려 두 번 모두 시나리오가 PASS(주문 8건, 215 s)였다. 첫 번째에서는 pode3의 선택이
  먹지 않아 pode3만 RPP로 갔고, 두 번째에서는 세 로봇 모두 RL로 갔다(작업 기록 7c절). 이 확인은 학습 PC의 작업 사본
  빌드로 했다.
- GitHub에서 `feat/rl-tracks-2026-11-26`(d4d7c77e)을 새로 받은 사본만으로도 확인했다: WSL 빌드, `pode_rl_controller` 시험
  8개 통과, 플릿 시뮬 `nav_rl:=rl_demohist`에서 세 번 보내는 선택 명령으로 세 로봇 모두 RL로 주행했고 시나리오가
  PASS(주문 8건, 216 s)였다(작업 기록 7c절).
- `store_arena_std`도 GitHub에서 `feat/rl-tracks-2026-11-26`(e5aa8258)을 새로 받은 사본만으로 확인했다: WSL 빌드,
  `pode_rl_controller` 시험 9개와 launch 연결 시험 90개 통과, 플릿 시뮬 `nav_rl:=rl_std`에서 세 번 보내는 선택 명령으로 세 로봇
  모두 RL로 주행했고 시나리오가 PASS(주문 8건, 219 s)였다(작업 기록 17b절). 한 번 돌린 것이다.
- `store_arena_demohist2`도 GitHub에서 `feat/rl-tracks-2026-11-26`(d36d469a)을 새로 받은 사본만으로 확인했다: WSL 빌드,
  `pode_rl_controller` 시험 10개와 launch 연결 시험 91개 통과, 플릿 시뮬 `nav_rl:=rl_demohist2`에서 시나리오가 PASS(주문 8건,
  215 s)였다(작업 기록 17d절). 한 번 돌린 것이다.
- 위 플릿 확인들의 "세 로봇 모두 RL로 주행했다"는 **선택 명령이 닿은 뒤에 시작된 Nav2 goal이 모두 RL이었다**는 뜻이다. 시나리오의
  첫 주문은 launch 10 s쯤 뒤에 시작되는데 선택 명령은 그 뒤(30~50 s)에 닿아서, 다시 본 확인 다섯 번 모두 pode1의 첫 goal은
  RPP로 끝났고 pode2와 pode3의 첫 goal은 달리는 중에 RL로 바뀌었다. 고정 경로 구간은 늘 RPP다(작업 기록 17d절).
- 실물: 돌려 보지 않았다.
