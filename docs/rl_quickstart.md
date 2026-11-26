# RL 정책 빨리 써 보기 (저장소만으로)

이 문서는 다른 PC나 로봇에서 저장소만 받아 RL 주행 정책을 돌리는 순서를 모은 것이다. 자세한 설명은 각 절 끝의
문서에 있다. 정책 파일(`policy.onnx`, `policy_spec.json`)은 저장소에 들어 있으므로 따로 받을 것이 없다.

## 1. 어느 브랜치를 받나

`main`에는 아직 RL 정책 파일이 없다(main 병합은 사용자가 정한다). 아래 가운데 하나를 받는다.

| 브랜치 | 들어 있는 것 | 쓰는 곳 |
|---|---|---|
| `feat/rl-policy-onnx-2026-11-26` | main + 정책 다섯(`store_arena_4b`, `c`, `acd`, `acd2`, `acd3`) + 시뮬 인자 `nav_rl:=rl_acd3` | 시뮬만. main과 가장 가깝다 |
| `feat/rl-real-optin-2026-11-26` | 위 + 실물 배포·시작 스크립트의 RL 켜기(`WITH_RL`, `RL_POLICY`). 플러그인 코드는 위와 같다 | 시뮬과 실물 |
| `feat/rl-tracks-2026-11-26` | 위 전부 + 11-26 작업(플러그인 출력 평활 옵션, 학습 코드, 측정 도구, 문서) + 시연 조건 후보 정책 `store_arena_demohist`(`nav_rl:=rl_demohist`) + 시뮬 시제품 `aisle_first` | 전부. 새 옵션은 기본값에서 꺼져 있다 |

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

- **바뀌었는지 꼭 확인한다.** Nav2 goal로 달리는 동안 controller_server 로그에 `FollowPathRL: 20 calls, infer mean ...` 줄이 1초마다
  나와야 한다(`nav_log_level:=info`). 그 줄이 없으면 그 로봇은 RPP로 가고 있는 것이므로 선택을 다시 보낸다. 한 번만 보내는
  형태(`--once`)는 11-26 플릿 시뮬 세 번의 선택 아홉 번 가운데 한 번이 오류 없이 끝나고도 전달되지 않았다(작업 기록 7c절).

- `nav_rl:=rl`은 정책 `store_arena_4b`, `nav_rl:=rl_acd3`은 `store_arena_acd3`다.
- `nav_rl:=rl_demohist`는 `store_arena_demohist`다(`follow_path_rl_demohist.yaml`).
- `store_arena_demohist`는 시연 조건 후보다: 시뮬에서만 확인했고 실물에서는 돌려 본 적 없다
  (근거는 `docs/worklog_rl_tracks_2026-11-26.md` 12~13절).
- 로봇 한 대로 컨트롤러를 견주는 묶음은 `tools/nav_bench/run_bench.sh`다. RL 컨트롤러 설정은
  `sim/fleet_gz_sim/nav_overrides/follow_path_rl_acd3.yaml`을 `BENCH_OVERRIDES`로 준다.
- 인자 설명: `sim/fleet_gz_sim/README.md`.

통로 안 회피 시연 시나리오(로봇 한 대, `feat/rl-tracks-2026-11-26`에만 있다)는 아래처럼 다시 돌린다. 시나리오는
`docs/rl_tracks_2026-11-26/gz_demo.yaml`에 있다: `f_cart`(통로 카트 하나), `g_demo`(카트 둘과 틈을 오가는 사람), `h_block`(통로
막힘), `j_cross`·`k_cross`(로봇 앞을 건너는 사람).

```bash
# acd3와 demohist를 시나리오마다 5판씩. aisle_first:=true는 "통로 먼저, 막히면 우회"(시뮬 전용 시제품)다
BENCH_LOCK= BENCH_DOMAIN=72 BENCH_REPEATS=5 BENCH_LAUNCH_ARGS="aisle_first:=true" \
BENCH_CONFIG=$PWD/docs/rl_tracks_2026-11-26/gz_demo.yaml \
BENCH_OVERRIDES=$PWD/docs/rl_tracks_2026-11-26/results/gz_demo_cand/overrides.yaml \
BENCH_CONTROLLERS="FollowPathRLShimACD3 FollowPathRLShimDemoHist" \
  tools/nav_bench/run_bench.sh ~/rl_demo_out f_cart g_demo k_cross
python3 tools/nav_bench/report.py --config docs/rl_tracks_2026-11-26/gz_demo.yaml --legs ~/rl_demo_out/legs --out ~/rl_demo_out/report
```

- `BENCH_LOCK=`(빈 값)은 학습 PC의 무거운 작업 잠금을 쓰지 않게 한다. 도메인은 43~101만 받는다.
- 판별 결과는 `~/rl_demo_out/report/legs.csv`(성공, 통로 안인지, 몸 여유 최솟값)다. 지금까지의 결과와 읽는 법은
  `docs/worklog_rl_tracks_2026-11-26.md` 8d, 13, 13b, 13c절에 있다. 건너는 사람(`j_cross`, `k_cross`)은 두 정책 모두 닿는다.

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
- **RL로 바뀌었는지 로그로 확인한 뒤에 결과를 적는다.** 주행 중 `FollowPathRL: N calls, infer ...` 줄이 없으면 RPP로 간 것이다
  (3절의 설명과 같다. 보는 명령은 `tools/real_pode/README.md`의 8번).
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
  전달되지 않아 pode3만 RPP로 갔고, 두 번째에서는 세 로봇 모두 RL로 갔다(작업 기록 7c절). 이 확인은 학습 PC의 작업 사본
  빌드로 했다.
- 세 번 보내는 선택 명령은 대역 구독자로 문법과 전달만 확인했다.
- 실물: 돌려 보지 않았다.
