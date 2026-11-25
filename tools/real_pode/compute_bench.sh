#!/usr/bin/env bash
# 로봇에서 controller 계산 시간을 잰다(RPP, MPPI 단계들, FollowPathRL, ONNX 추론만):  ./compute_bench.sh
# 로봇은 움직이지 않는다: 속도 명령을 내지 않고, 다른 노드가 cmd_vel을 내고 있으면 시작하지 않는다.
# 빌드하지 않는다(pode_rl_controller, pode_compute_bench가 nav2fleet_ws에 빌드되어 있어야 한다).
# 결과: logs/compute_bench/<날짜_시각>/ 의 table.md, summary.csv, runs.csv, samples.csv, publishers.txt, sysinfo.txt, bench.log
#   N=600 WARMUP=100      경우마다 잴 호출 수와 버릴 호출 수
#   BENCH_DOMAIN=71       측정 프로세스만 쓰는 도메인(LOCALHOST). 로봇 도메인(6, 12)은 쓸 수 없다
#   ALLOW_EMPTY_GRAPH=1   로봇 그래프에 노드가 하나도 없어도 시작한다(스택을 모두 내렸을 때만)
#   ENV_FILE=...          env.sh 대신 읽을 환경(PC 시험용). OUT_ROOT=...  결과 상위 폴더
set -euo pipefail
here=$(dirname "$(readlink -f "$0")")
N=${N:-600}
WARMUP=${WARMUP:-100}
BENCH_DOMAIN=${BENCH_DOMAIN:-71}
OUT_ROOT=${OUT_ROOT:-$here/logs/compute_bench}

# ---- 환경: 로봇 그래프 전체를 보려고 SUPER=1로 읽는다 ----
set +u
SUPER=1 source "${ENV_FILE:-$here/env.sh}" || { echo "환경을 읽지 못했다: ${ENV_FILE:-$here/env.sh}"; exit 1; }
set -u
case $BENCH_DOMAIN in
  6|12) echo "BENCH_DOMAIN=$BENCH_DOMAIN 은 로봇 도메인이다. 다른 값을 쓴다"; exit 1 ;;
esac

# ---- 설치 확인(빌드는 하지 않는다) ----
if ! ros2 pkg prefix nav2_mppi_controller > /dev/null 2>&1; then
  echo "nav2_mppi_controller가 없다. 설치: sudo apt install ros-jazzy-nav2-mppi-controller"
  exit 1
fi
for pkg in pode_rl_controller pode_compute_bench; do
  if ! ros2 pkg prefix "$pkg" > /dev/null 2>&1; then
    echo "$pkg 가 빌드되어 있지 않다. PC에서 소스를 복사한 뒤 로봇에서:"
    echo "  cd ~/nav2fleet_ws && colcon build --packages-select pode_rl_controller pode_compute_bench"
    echo "  (pode_rl_controller는 ros-jazzy-onnxruntime-vendor가 필요하다)"
    exit 1
  fi
done

# ---- 주행 스택이 돌고 있으면 멈춘다(속도 명령, CPU 다툼) ----
if pgrep -f '__node:=controller_server|/controller_server( |$)' > /dev/null; then
  echo "controller_server가 돌고 있다. ./stop.sh로 내린 뒤 다시 한다"
  exit 1
fi
if ! topics=$(timeout 20 ros2 topic list --no-daemon 2>/dev/null); then
  echo "ROS 그래프를 읽지 못했다(도메인 $ROS_DOMAIN_ID). 디스커버리 서버가 꺼져 있으면 DISCOVERY_SERVER=off 로 다시 한다"
  exit 1
fi
# 디스커버리 서버에 닿지 못해도 빈 그래프로 끝날 수 있다: 노드가 하나도 안 보이면 확인 못 한 것으로 본다
nodes=$(timeout 20 ros2 node list --no-daemon 2>/dev/null | grep -c . || true)
if [ "$nodes" = 0 ] && [ "${ALLOW_EMPTY_GRAPH:-0}" != 1 ]; then
  echo "로봇 그래프(도메인 $ROS_DOMAIN_ID)에 노드가 하나도 안 보인다. 디스커버리 서버 연결을 확인한다"
  echo "(서버가 꺼져 있으면 DISCOVERY_SERVER=off, 스택을 모두 내린 것이 맞으면 ALLOW_EMPTY_GRAPH=1)"
  exit 1
fi
for t in $(grep cmd_vel <<< "$topics" || true); do
  pubs=$(timeout 20 ros2 topic info "$t" --no-daemon | sed -n 's/^Publisher count: //p' || true)
  if ! [[ $pubs =~ ^[0-9]+$ ]]; then
    echo "$t 의 publisher 수를 읽지 못했다. 시작하지 않는다"
    exit 1
  fi
  if [ "$pubs" != 0 ]; then
    echo "$t 에 publisher가 $pubs 개 있다. 그 노드를 내린 뒤 다시 한다:"
    timeout 20 ros2 topic info -v "$t" --no-daemon | grep -E 'Node name|Node namespace' || true
    exit 1
  fi
done
echo "cmd_vel publisher 없음(도메인 $ROS_DOMAIN_ID, 노드 $nodes개, cmd_vel 토픽 $(grep -c cmd_vel <<< "$topics" || true)개 확인)"

# ---- 측정: 다른 도메인, 이 컴퓨터 안에서만 ----
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=$BENCH_DOMAIN ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
out=$OUT_ROOT/$(date +%F_%H%M%S)
mkdir -p "$OUT_ROOT"
mkdir "$out" || { echo "$out 이 이미 있다. 잠시 뒤 다시 한다"; exit 1; }
{
  echo "host $(hostname)  date $(date -Is)"
  uname -a
  grep -m1 -E 'Model|model name' /proc/cpuinfo || true
  echo "nproc $(nproc)  loadavg $(cat /proc/loadavg)"
  cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || true
  if command -v vcgencmd > /dev/null; then vcgencmd measure_temp; vcgencmd get_throttled; fi
  echo "N=$N WARMUP=$WARMUP BENCH_DOMAIN=$BENCH_DOMAIN"
} > "$out/sysinfo.txt"
nav2=$(ros2 pkg prefix pode_nav_config)/share/pode_nav_config/params/nav2_params.yaml
bench=$(ros2 pkg prefix pode_compute_bench)/lib/pode_compute_bench
echo "결과 폴더 $out (약 $(( (WARMUP + N) * 6 / 20 + (WARMUP + N) * 2 / 10 + 30 )) 초)"
set +e
"$bench/compute_bench" --nav2-params "$nav2" --map "$here/maps/arena_v7_a.yaml" --out "$out" \
  --n "$N" --warmup "$WARMUP" > "$out/bench.log" 2>&1
code=$?
set -e
if command -v vcgencmd > /dev/null; then
  { echo "after:"; vcgencmd measure_temp; vcgencmd get_throttled; } >> "$out/sysinfo.txt"
fi
grep -E 'case .*calls|publishers in the graph|FATAL' "$out/bench.log" || true
if [ -s "$out/runs.csv" ]; then
  python3 "$bench/bench_table.py" "$out" --title "$(hostname) $(date +%F) (N=$N)"
fi
echo "compute_bench 종료 코드 $code (0 정상, 2 호출 오류, 3 cmd_vel publisher 발견). 결과: $out"
exit $code
