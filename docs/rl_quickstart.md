# RL 정책 빨리 써 보기 (저장소만으로)

이 문서는 다른 PC나 로봇에서 저장소만 받아 RL 주행 정책을 돌리는 순서를 모은 것이다. 자세한 설명은 각 절 끝의
문서에 있다. 정책 파일(`policy.onnx`, `policy_spec.json`)은 저장소에 들어 있으므로 따로 받을 것이 없다.

## 1. 어느 브랜치를 받나

`main`에는 아직 RL 정책 파일이 없다(main 병합은 사용자가 정한다). 아래 가운데 하나를 받는다.

| 브랜치 | 들어 있는 것 | 쓰는 곳 |
|---|---|---|
| `feat/rl-policy-onnx-2026-11-26` | main + 정책 다섯(`store_arena_4b`, `c`, `acd`, `acd2`, `acd3`) + 시뮬 인자 `nav_rl:=rl_acd3` | 시뮬만. main과 가장 가깝다 |
| `feat/rl-real-optin-2026-11-26` | 위 + 실물 배포·시작 스크립트의 RL 켜기(`WITH_RL`, `RL_POLICY`). 플러그인 코드는 위와 같다 | 시뮬과 실물 |
| `feat/rl-tracks-2026-11-26` | 위 전부 + 11-26 밤 작업(플러그인 출력 평활 옵션, 학습 코드, 측정 도구, 문서) | 전부. 새 옵션은 기본값에서 꺼져 있다 |

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
colcon test --packages-select pode_rl_controller --event-handlers console_direct+   # 정책 다섯의 추론 일치 시험
```

sudo를 못 쓰면 `.deb`를 `dpkg -x`로 풀어 `CMAKE_PREFIX_PATH`에 넣는다(`pode/pode_rl_controller/README.md`).

## 3. 시뮬에서 달려 보기

```bash
# 플릿 시뮬: 로봇마다 RL controller 둘(FollowPathRL = shim으로 감싼 정책, FollowPathRLAlone)을 더한다
ros2 launch fleet_gz_sim gz.launch.py nav_rl:=rl_acd3
# 로봇 하나를 RL로 바꾸기(되돌리기는 data: FollowPathGoal). 그다음 보내는 Nav2 goal부터 RL이다
# Nav2의 선택 노드가 transient_local·reliable로 듣기 때문에 QoS 두 옵션을 빼면 --once가 보내지 못하고 기다린다
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable   /pode1/controller_selector std_msgs/msg/String "{data: FollowPathRL}"
```

- `nav_rl:=rl`은 정책 `store_arena_4b`, `nav_rl:=rl_acd3`은 `store_arena_acd3`다.
- 로봇 한 대로 컨트롤러를 견주는 묶음은 `tools/nav_bench/run_bench.sh`다. RL 컨트롤러 설정은
  `sim/fleet_gz_sim/nav_overrides/follow_path_rl_acd3.yaml`을 `BENCH_OVERRIDES`로 준다.
- 인자 설명: `sim/fleet_gz_sim/README.md`.

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
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable \
  /pode1/controller_selector std_msgs/msg/String "{data: FollowPathRL}"
```

- 기본 controller는 그대로 RPP(FollowPathGoal)다. 고정 경로(pode_fsm의 FollowPath)는 늘 RPP다.
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
- 그 사본만으로 Gazebo에서 acd3를 달리는 확인: (채움)
- 실물: 돌려 보지 않았다.
