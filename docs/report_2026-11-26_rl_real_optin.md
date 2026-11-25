# 2026-11-26 실물 RL 선택 경로 (WITH_RL, RL_POLICY)

목적: 새로 받은 저장소만으로 실물 Pode에서 RL controller(예: `store_arena_acd3`)를 띄울 수 있게 한다.
명시적으로 고를 때만 켜지고, 고르지 않으면 배포·시작은 예전과 같다. 로봇에는 닿지 않았고, 이 길은 로봇에서 아직 돌려 보지 않았다.
배경은 `docs/report_2026-11-24_rl_real_prep.md`(이하 11-24 보고)의 2.2절, 2.3절, "사용자가 정할 것" 2번이다.

## 바뀐 것

| 파일 | 내용 |
|---|---|
| `tools/real_pode/make_params.py` | `--rl-policy NAME`. 주면 `controller_plugins` 끝에 `FollowPathRL`, `FollowPathRLAlone`을 더하고, 두 블록을 `sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml`에서 줄 그대로 가져와 `policy`만 NAME으로 바꿔 `controller_plugins` 줄 바로 아래에 넣는다. 결과를 yaml로 읽어 "기본 사본 + 두 블록 + 목록 두 이름"과 같은지 확인한 뒤에만 쓴다. 이름은 `[A-Za-z0-9_]+`만 받고, 정책 폴더(`pode/pode_rl_controller/policies/NAME`, 저장소 또는 로봇 `~/nav2fleet_ws/src`)에 `policy.onnx`와 `policy_spec.json`이 없으면 아무것도 쓰지 않고 종료 2. 실물 목록에 MPPI나 RL 이름이 이미 있으면 거절. 주지 않으면 코드 경로가 예전과 같다 |
| `tools/real_pode/start.sh` | `RL_POLICY=NAME`이면 `make_params.py`에 `--rl-policy NAME`, launch에 `log_level:=info`를 더한다. 비면 꺼짐. `NO_MOTION=1`과 함께 주면 거절(종료 2) |
| `tools/real_pode/deploy.sh` | `WITH_RL=1`이면 (1) 아무것도 복사하기 전에 ssh로 `ros2 pkg prefix onnxruntime_vendor`를 보고, 없으면 11-24 보고 1.2절의 설치 방법(apt, 오프라인 .deb와 SHA256)을 찍고 종료 1, 이어서 `nav2_rotation_shim_controller`도 같은 식으로 확인(없으면 apt 명령을 찍고 종료 1), (2) 평소 복사 뒤 `pode/pode_rl_controller`와 `follow_path_rl.yaml`을 `~/nav2fleet_ws/src/` 아래 같은 경로로 보내고, (3) 로봇에서 `MAKEFLAGS=-j1 colcon build --packages-select pode_rl_controller --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF`. 그 밖의 값은 거절(종료 2) |
| `tools/test/test_real_pode_rl.py` | 새 시험(아래) |
| `tools/real_pode/README.md` | 파일 표의 deploy.sh 줄, 새 절 "RL 정책으로 달려 보기"(순서, 바꾸기·되돌리기, 시간 로그, 적을 것, 위험) |

시뮬 파일 `follow_path_rl.yaml`의 RL 블록은 원래 시뮬 전용이고, 그 머리 주석에는 "실물 nav2_params.yaml에는 아직 넣지 않는다"가 남아 있다.
이번 변경도 저장소 원본 `nav2_params.yaml`에는 넣지 않았다. 로봇의 `params/` 사본에 선택했을 때만 들어간다. 그 주석은 고치지 않았다.

## PC에서 시험한 것

1. 기본 출력이 그대로인지: 바꾸기 전 코드로 `--speed 0.12/0.05`, `--fixed-line-check` 유무 네 경우의 사본을 만들어 두고, 바꾼 뒤 같은 명령의 결과를 `cmp`로 견줬다. 8개 파일 모두 같았다(`same s0.12/nav2_params.yaml` … `same f0.05/avoidance.yaml`).
   시험 `test_default_output_is_byte_identical_to_before`는 같은 비교를 커밋 `951d71bb`의 `make_params.py`(git show)와 바이트로 한다. 그 커밋이 없는 작업본(복사본)에서는 건너뛴다.
2. RL 출력: 정책 폴더 다섯 모두에 대해 RL 사본 = 시뮬 파일의 두 블록(policy만 바뀜), 목록 = 기본 목록 + 두 이름(MPPI 없음), 그 밖은 기본 사본과 같음. 글자 단위로도 `controller_plugins` 줄 하나만 바뀌고 나머지는 줄이 더해지기만 한다. 시뮬 파일 값을 바꾸면 사본이 따라간다.
3. 틀린 이름(`nope`, 빈 이름, `../…`, 절대 경로, 공백, `;`)과 `policy.onnx` 또는 `policy_spec.json`이 없는 폴더는 예외·종료 2이고 출력 폴더가 생기지 않는다.
4. 스크립트 정적 검사: deploy.sh에서 RL 관련 줄(`rl_pkg`, `rl_yaml`, `onnxruntime`, `pode_rl_controller`, `--packages-select`)은 정의 두 줄 말고는 모두 `if [ -n "$with_rl" ]` 블록 안, 기본 `packages=(...)`에 RL 없음. start.sh에서 `--rl-policy`와 `log_level`은 각각 `[ -z "$rlp" ] ||`, `[ -z "$9" ] ||` 뒤에만 있다.
5. 스크립트 실행(대역 ssh·rsync·ros2·make_params, WSL bash): deploy.sh 기본(`WITH_RL` 없음, `0`, 빈 값)은 rsync 9번, RL·onnxruntime·colcon 호출 없음. `WITH_RL=1`에 onnxruntime이 없으면 ssh 확인 한 번 뒤 종료 1(복사 없음), 있으면 RL 패키지와 yaml rsync, 빌드 한 번. start.sh 기본은 make_params 인자 `--speed 0.12 --out params`, launch 인자는 예전과 같고, `RL_POLICY`를 주면 `--rl-policy`와 `log_level:=info`가 더해진다.
6. `bash -n tools/real_pode/deploy.sh`, `bash -n tools/real_pode/start.sh` 통과. WSL에 shellcheck가 없어 돌리지 못했다.
7. 기존 시험: 아래 "명령과 결과".

## 로봇 없이는 모르는 것

- 로봇의 `ros2 pkg prefix onnxruntime_vendor` 검사가 ssh 비대화 셸에서 실제로 맞게 도는지(로봇 기본 셸이 bash라고 가정했다).
- Pi 4에서 `pode_rl_controller` 빌드 시간·메모리, `-DBUILD_TESTING=OFF` 빌드가 성공하는지.
- 실물 controller_server가 두 플러그인을 싣는지(특히 RotationShim 패키지가 로봇에 있는지, `angular_disengage_threshold`를 읽는 버전인지).
- 추론 시간, scan age, 제어 주기 놓침, CPU. `log_level:=info`가 Nav2 노드 전체의 기록을 늘려 Pi 4 부하를 얼마나 더하는지.
- 몸 뒤 왼쪽 0.075 m 점이 실제 주행에서 정책을 얼마나 흔드는지.

## 사용자가 정할 것 (11-24 보고에서 남은 것과 이번에 생긴 것)

1. 몸 뒤 왼쪽 0.075 m 점(11-24 결정 1): 빔 가리기 파라미터, 재학습, 그대로 시험 중 무엇. 이번 변경은 아무것도 가리지 않는다.
2. 실물에 RL을 띄우는 길(결정 2): 이번에 (b) 코드 변경 쪽을 만들었다. 손으로 하는 (a)도 그대로 된다.
3. 실물 첫 시험을 RL 혼자로 할지 RL+shim으로 할지(결정 3): 두 controller가 다 실리므로 `controller_selector`로 고른다. 시뮬 파일은 지금 shim이 `FollowPathRL`(기본)이다.
4. Pi 4에서 gtest를 돌릴지(결정 4): deploy.sh는 `-DBUILD_TESTING=OFF`로 빌드한다(빌드 시간과 `ament_cmake_gtest` 의존을 피하려고). 돌리려면 로봇에서 `--cmake-args -DBUILD_TESTING=ON`으로 다시 빌드하고 `colcon test`.
5. 로봇 인터넷(결정 5): deploy.sh는 설치하지 않고 방법만 알린다.
6. `scan_timeout` 0.5 s(결정 6): 시뮬 파일 값을 그대로 쓴다. 올리면 시뮬과 실물이 같이 바뀐다(값의 출처가 하나라서).
7. (새로) RL controller는 start.sh의 SPEED를 따르지 않는다. 시뮬 파일에 `max_linear_vel`이 없어 정책 v_max(0.12 m/s)까지 낸다. SPEED가 0.12보다 낮으면 make_params.py가 WARNING만 찍는다. 실물에서 RL만 느리게 하려면 시뮬 파일과 다른 값을 넣는 규칙이 필요하다(이번에는 "policy만 바꾼다"는 조건 때문에 하지 않았다).
8. (새로) 시간 로그를 보려고 `RL_POLICY`를 주면 Nav2 노드 전체가 `log_level:=info`가 된다. controller_server만 올리려면 `nav2.launch.py`에 인자를 더해야 한다(이번에는 손대지 않았다).

## 명령과 결과

- Windows `python -m pytest tools/test/test_real_pode_rl.py -q -p no:cacheprovider`: `29 passed, 11 skipped`.
  건너뛴 11개는 bash 스크립트 실행 시험이다(Windows에서는 건너뛰게 했다, 아래 WSL에서 돌았다). 기준 커밋 바이트 비교 6개는 여기서 돌았다.
- WSL(작업본을 `~/nf_src/pd-rl-real`로 복사, 새 git 저장소로 커밋, `source /opt/ros/jazzy/setup.bash`, `pode_nav_config`까지 colcon 빌드 뒤 그 install을 source):
  - `python3 -m pytest tools/test/test_real_pode.py tools/test/test_real_pode_rl.py tools/test/test_policy_files.py`: `192 passed, 10 skipped`.
    건너뛴 것: 기준 커밋 바이트 비교 6개(복사본에는 커밋 `951d71bb`이 없다, Windows에서 돌았다), `pode_fsm not installed` 2개(빌드하지 않은 패키지), 나머지 2개는 기존 시험 파일의 건너뜀이고 이유는 이번에 보지 않았다.
  - `python3 -m pytest pode/pode_nav_config/test`: `132 passed`. 빌드 없이 돌리면 같은 시험이 바꾸기 전 커밋에서도 `64 failed`(PackageNotFoundError)였다. 빌드한 뒤 모두 통과했다.
- 바꾸기 전 커밋으로 같은 WSL 시험을 돌린 값: `154 passed, 8 skipped`(새 시험 파일 없음, ROS를 source하지 않음).
- 검수(Codex, 읽기 전용) 두 번. 첫 번째 지적 둘(shim이 없으면 controller_server가 뜨지 못함, 기본 배포 시험이 느슨함)은 고쳤다(deploy.sh가 shim도 확인, 기본 배포 호출 전체 비교).
  두 번째 지적 둘(대역이 인자 경계를 합쳐 기록함, shim 확인 명령 문자열을 통째로 단언하지 않음)은 시험을 더 엄격히 하라는 것이라 고치지 않았다.
