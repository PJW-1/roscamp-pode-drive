# 보고서: Pi 4 계산량 측정 도구 (RPP, MPPI 단계, RL 정책) (2026-11-26)

"RL 정책(작은 ONNX MLP 한 번 추론)이 Pi 4에서 MPPI보다 훨씬 싸다"를 숫자로 보이려고 측정 도구를 만들었다.
로봇에서는 사용자가 손으로 돌린다. 이 보고서의 숫자는 **PC(WSL) 시험 숫자이고 Pi 4 숫자가 아니다.**

## 1. 무엇을 재나

`pode/pode_compute_bench`의 `compute_bench`(C++ 실행 파일 하나)가 Nav2 controller 플러그인을 pluginlib로 띄우고,
controller_server와 같은 인터페이스 `nav2_core::Controller::computeVelocityCommands`를 고정 장면에서 반복해 부른다.

- 장면: v7 경기장 지도 `tools/real_pode/maps/arena_v7_a.yaml`(2.5 cm, 실물 전역 costmap과 같은 지도). 로봇은 가운데
  통로 (0.40, 0.58), 방향 0, 속도 0.10 m/s로 고정. 경로는 통로를 따라 x 1.75 m까지 간 뒤 남쪽으로 꺾어 y 0.15 m까지(2.5 cm 간격).
- costmap: `Costmap2DROS` 하나. 실물 `local_costmap` 값(3 m 굴림 창, 2.5 cm, 다각형 발자국, inflation 0.15 m / 3.0)에
  obstacle 층 대신 정적 지도 층을 넣었다. 지도가 들어온 것을 확인한 뒤 측정 동안은 갱신을 멈춘다(`pause()`).
- 경우(`pode/pode_compute_bench/config/cases.yaml`):
  - `rpp`: 실물이 쓰는 `FollowPath`(RPP). 값은 실행할 때 설치된 `pode_nav_config/params/nav2_params.yaml`에서 그대로 읽는다.
  - `mppi_l0`~`mppi_l4`: `nav2_params.yaml`의 `FollowPathMPPI` 값을 모두 읽고 `batch_size`, `time_steps`, `model_dt`만 덮는다.
    RotationShim 없이 `MPPIController`만 띄운다(경로와 같은 방향이면 shim은 MPPI에 그대로 넘긴다).
  - `rl`: `pode_rl_controller::FollowPathRL`, 정책 `store_arena_4b`. 값은 시뮬 `FollowPathRLAlone`과 같고 `scan_topic`만
    다르다. 측정 프로세스가 640 광선 합성 LaserScan을 10 Hz로 보낸다.
  - `onnx_only`: 같은 정책의 ONNX Runtime 추론만(`OnnxPolicy::run`, CPU 1 스레드, 고정 41차원 관측).
- 부르는 방식: 경우마다 그 경우가 가정하는 주기(20 Hz 또는 10 Hz)로 부른다. 처음 100번은 버리고 다음 600번을 잰다.
  잰 값은 `computeVelocityCommands` 호출 하나의 벽시계 시간이다.
- 결과: `samples.csv`(호출마다), `runs.csv`(경우마다 CPU 시간), `bench_table.py`가 만든 `summary.csv`, `table.md`.
  p50, p99, max, 평균(ms), p99 / 50 ms(20 Hz 예산), 프로세스 CPU 비율 = 측정 구간의 (user+system) / 벽시계(한 코어 = 100 %),
  측정 스레드만의 CPU 비율.

재지 않는 것:

- costmap 갱신 비용(실물 local costmap은 5 Hz로 라이다를 넣고 inflation을 다시 한다). 측정 중에는 갱신을 멈춘다.
- 실제 라이다 콜백, AMCL, TF, controller_server의 나머지 일(경로 받기, 진행 검사, cmd_vel 내기), DDS 통신.
- 움직이는 장면: 로봇이 서 있고 경로가 같다. MPPI의 계산량은 batch_size x time_steps에 거의 비례하고 장면에 덜 기대지만,
  RPP의 충돌 검사와 RL 관측 만들기는 장면에 따라 조금 달라질 수 있다 [추측].
- RotationShim의 비용, 주행 성능(어느 단계의 MPPI가 실물에서 잘 따라가는지)은 이 도구 밖이다.

## 2. 속도 명령을 내지 않는다는 확인

- 측정 프로세스는 controller가 돌려준 Twist를 버린다. 코드에 cmd_vel publisher가 없다.
- 끝에서 그래프의 모든 publisher(그 도메인에서 보이는 것 전부)를 `publishers.txt`에 적고, 이름에 `cmd_vel`이 든 토픽이
  하나라도 있으면 종료 코드 3으로 실패한다. 플러그인이 다 떠 있을 때(경우마다 warm-up 뒤) 모은다.
- 로봇 스크립트 `tools/real_pode/compute_bench.sh`는 측정 전에 로봇 도메인(env.sh, SUPER=1)에서 cmd_vel 토픽마다 publisher
  수를 보고 하나라도 있으면 시작하지 않는다. controller_server 프로세스가 있어도 시작하지 않는다.
  측정은 다른 도메인(기본 71)과 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`에서 돌려 로봇 스택과 섞이지 않는다.

PC 시험에서 확인한 것:

- PC 시험의 `publishers.txt`(`docs/pi4_compute_2026-11-26/pc_dry_run/publishers.txt`): 27개, `cmd_vel`이 든 토픽 0개.
  모두 이 프로세스의 노드(`/compute_bench`, `/costmap`, TF listener)다. 로그 끝줄:
  `27 publishers in the graph, 0 on a cmd_vel topic (list in publishers.txt)`, 스크립트 종료 코드 0.
- 거부 경로는 가짜 `ros2`(PATH 앞에 둔 bash 스크립트)로 돌렸다. 실제 ROS 그래프로 cmd_vel publisher를 띄워 본 것은 아니다.
  - `nav2_mppi_controller` 없음 → `nav2_mppi_controller가 없다. 설치: sudo apt install ros-jazzy-nav2-mppi-controller`, 종료 1
  - `/pode1/cmd_vel` publisher 1개 → `/pode1/cmd_vel 에 publisher가 1 개 있다. 그 노드를 내린 뒤 다시 한다:` 와 노드 이름, 종료 1
  - `BENCH_DOMAIN=6` → `BENCH_DOMAIN=6 은 로봇 도메인이다. 다른 값을 쓴다`, 종료 1
  - 세 경우 모두 결과 폴더를 만들기 전에 멈췄다.

## 3. MPPI 단계를 고른 방법

L0은 지금 시뮬 `FollowPathMPPI`(batch 1000, 83 단계, dt 0.05 s = 4.15 s 앞, 20 Hz)이다. MPPI 계산은 대략
batch_size x time_steps 궤적 점에 비례하므로 그 곱을 단계마다 크게 줄였다. MPPI(Jazzy)는 controller 주기가 model_dt와 같아야
하므로(더 길면 `Controller period more then model dt, set it equal to model dt` 오류. 문자열은 설치된
`libmppi_controller.so`에서 확인했고 그 경우를 돌려 보지는 않았다), dt 0.1 s 단계는 10 Hz를 가정한다.

| 단계 | batch | 단계 수 | dt (s) | 앞보기 (s) | 0.12 m/s에서 (m) | 가정 주기 | batch x 단계 | L0 대비 |
|---|---|---|---|---|---|---|---|---|
| L0 | 1000 | 83 | 0.05 | 4.15 | 0.50 | 20 Hz | 83 000 | 1 |
| L1 | 500 | 56 | 0.05 | 2.80 | 0.34 | 20 Hz | 28 000 | 0.34 |
| L2 | 300 | 40 | 0.05 | 2.00 | 0.24 | 20 Hz | 12 000 | 0.14 |
| L3 | 200 | 20 | 0.10 | 2.00 | 0.24 | 10 Hz | 4 000 | 0.048 |
| L4 | 100 | 15 | 0.10 | 1.50 | 0.18 | 10 Hz | 1 500 | 0.018 |

- L1의 56 단계는 Nav2 기본 time_steps이다. L2~L3의 2.0 s 앞보기(0.24 m)는 RPP 앞보기 상한 0.25 m와 비슷하다.
- L4(0.18 m 앞, 100개)는 주행 품질을 보장하는 값이 아니라 "가장 가볍게 하면 얼마나 드나"의 아래 끝이다. L2 이하 단계가
  실물이나 시뮬에서 경로를 잘 따르는지는 재지 않았다(모름).
- 비율 열 "p99 / 50 ms"는 모든 경우에 20 Hz 예산(50 ms)으로 나눴다. 10 Hz 단계의 실제 예산은 100 ms이다.

## 4. PC 시험 결과 (PC 숫자, Pi 4 숫자가 아님)

- 기계: Intel i7-10700K(16 스레드), WSL2, ROS 2 Jazzy, ROS 도메인 71(LOCALHOST). 같은 PC에서 다른 일(GPU 학습 등)이
  같이 돌 수 있었다(시작 때 loadavg 1.17). 숫자는 `docs/pi4_compute_2026-11-26/pc_dry_run/`(`table.md`, `summary.csv`,
  `runs.csv`, `samples.csv`, `sysinfo.txt`, `publishers.txt`).
- 명령(WSL, 빌드한 install을 읽는 환경 파일을 `ENV_FILE`로 줬다):
  `ENV_FILE=<pc 환경> OUT_ROOT=/tmp/cb_pc bash tools/real_pode/compute_bench.sh` (N=600, WARMUP=100, 약 6분)

| 경우 | 플러그인 | batch | steps | dt (s) | 주기 (Hz) | n | 오류 | 넘침 | p50 (ms) | p99 (ms) | max (ms) | 평균 (ms) | p99 / 50 ms | 프로세스 CPU (%) | 측정 스레드 CPU (%) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| rpp | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |  |  |  | 20 | 600 | 0 | 0 | 0.241 | 0.847 | 5.781 | 0.290 | 0.017 | 0.8 | 0.6 |
| mppi_l0 | `nav2_mppi_controller::MPPIController` | 1000 | 83 | 0.05 | 20 | 600 | 0 | 0 | 2.974 | 4.720 | 25.561 | 3.209 | 0.094 | 6.6 | 6.5 |
| mppi_l1 | `nav2_mppi_controller::MPPIController` | 500 | 56 | 0.05 | 20 | 600 | 0 | 0 | 0.819 | 1.978 | 8.395 | 0.940 | 0.040 | 2.1 | 1.9 |
| mppi_l2 | `nav2_mppi_controller::MPPIController` | 300 | 40 | 0.05 | 20 | 600 | 0 | 0 | 0.332 | 2.270 | 4.971 | 0.415 | 0.045 | 1.1 | 0.9 |
| mppi_l3 | `nav2_mppi_controller::MPPIController` | 200 | 20 | 0.1 | 10 | 600 | 0 | 0 | 0.165 | 0.547 | 3.675 | 0.197 | 0.011 | 0.4 | 0.2 |
| mppi_l4 | `nav2_mppi_controller::MPPIController` | 100 | 15 | 0.1 | 10 | 600 | 0 | 0 | 0.088 | 0.510 | 1.450 | 0.106 | 0.010 | 0.3 | 0.1 |
| rl | `pode_rl_controller::FollowPathRL` |  |  |  | 20 | 600 | 0 | 0 | 0.152 | 1.063 | 3.735 | 0.180 | 0.021 | 0.6 | 0.4 |
| onnx_only | `OnnxPolicy store_arena_4b` |  |  |  | 20 | 600 | 0 | 0 | 0.057 | 0.342 | 1.208 | 0.074 | 0.007 | 0.4 | 0.2 |

넘침: 그 경우의 주기(1/주기 Hz)보다 오래 걸린 호출 수. 프로세스 CPU: 측정 구간의 (user+system) / 벽시계, 한 코어 = 100 %.

읽는 법(PC 숫자에서만 말할 수 있는 것):

- 평균으로 MPPI L0(3.21 ms)은 RL 플러그인 전체(0.180 ms)의 약 18배, ONNX 추론만(0.074 ms)의 약 43배다. RPP는 0.290 ms.
- MPPI 평균은 batch x 단계에 거의 비례해 줄었다(L0 → L1: 곱 0.34배, 시간 0.29배. L2: 0.14배, 0.13배).
- 따로 잰 ONNX 추론 평균(0.074 ms)은 RL 플러그인 평균의 약 40 %다. 나머지는 스캔 → 광선 변환, 관측, TF 조회로 보인다
  (플러그인 안의 추론 시간을 따로 표에 넣지는 않았다. 플러그인 로그 `infer mean`이 `bench.log`에 있다).
- max 열은 PC의 다른 일과 WSL 스케줄링이 섞인 값이라 하나만 보고 판단하지 않는다(L0 max 25.6 ms, 넘침 0).
- Pi 4(Cortex-A72 1.5 GHz)에서는 모두 몇 배 느려질 것으로 보이지만 배수는 모른다. Pi 4 숫자로 주장하려면 로봇 측정이 필요하다.

## 5. 로봇에서 돌리는 명령

실물 측정 체크리스트에 붙일 부분(로봇은 움직이지 않는다):

```text
[Pi 4 계산량 측정] 측정 약 7분(Pi 4 빌드 시간은 모름). 로봇은 서 있다. 속도 명령을 내지 않는다
1. PC(저장소 최신 HEAD): 키트 복사  tools/real_pode/deploy.sh pinky@<로봇IP>
2. PC: deploy.sh에 없는 두 패키지 복사
   git archive HEAD pode/pode_rl_controller pode/pode_compute_bench | ssh pinky@<로봇IP> 'tar -x -C ~/nav2fleet_ws/src'
3. 로봇: 없으면 설치  sudo apt install ros-jazzy-nav2-mppi-controller ros-jazzy-onnxruntime-vendor
4. 로봇: 빌드(시험 빼고)
   cd ~/nav2fleet_ws && colcon build --packages-select pode_rl_controller pode_compute_bench --cmake-args -DBUILD_TESTING=OFF
5. 로봇: 주행 스택 내리기  cd ~/nav2fleet_real && ./stop.sh
6. 로봇: 측정  cd ~/nav2fleet_real && ./compute_bench.sh
   - "cmd_vel 에 publisher가 있다"로 멈추면 적힌 노드를 내리고 다시 한다
   - 디스커버리 서버(관제 PC)가 꺼져 있으면  DISCOVERY_SERVER=off ./compute_bench.sh
   - 다른 무거운 일(빌드, rosbag)은 돌리지 않는다. 끝에 표가 화면에 나온다
7. 돌려받을 것: 폴더 통째로
   scp -r pinky@<로봇IP>:~/nav2fleet_real/logs/compute_bench/<날짜_시각> .
   (table.md, summary.csv, runs.csv, samples.csv, sysinfo.txt(온도, 스로틀), bench.log, publishers.txt)
   마지막 줄 "compute_bench 종료 코드 0"인지 같이 적어 준다
```

## 6. 알려진 제약과 남은 일

- `deploy.sh`는 `pode_rl_controller`와 `pode_compute_bench`를 복사하지 않는다. 그래서 위 2번처럼 따로 복사한다.
  `deploy.sh`의 패키지 목록에 넣을지는 정하지 않았다(바꾸지 않았다).
- `compute_bench`는 결과를 다 쓴 뒤 `std::_Exit`로 끝난다. NodeOptions 생성자로 만든 `Costmap2DROS`를 없애면
  소멸자에서 SIGSEGV가 났다(PC에서 재현, 원인은 찾지 않았다).
- 시간 제한이나 허용치를 늘리거나 시험을 건너뛴 것은 없다. RL의 `scan_timeout`은 시뮬 값 0.5 s 그대로다.
- Pi 4 숫자는 아직 없다. 로봇 측정 뒤 `table.md`를 이 보고서에 더한다.
