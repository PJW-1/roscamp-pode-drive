#!/usr/bin/env bash
# 출력 평활 Gazebo 비교(rl_acd3_2026-11-25/gz_run.sh와 같은 방식). WSL에서: bash gz_run.sh [OUT]
# shim으로 감싼 acd3의 평활 없음, ema_alpha_w 0.5, 0.25와 RPP(FollowPathGoal), MPPI를
# Q2 -> E-3(a_hall), c_moving, d_robot에서 칸마다 GZ_REPEATS판(기본 10), GUI 없음.
# SRC, B는 이 브랜치를 합친 소스와 그 빌드(전체 colcon build). 잠금은 run_bench.sh가 시나리오마다 잡고 놓는다.
SRC=${SRC:-$HOME/nf_src/pd-rl-base}
B=${B:-$HOME/gg_build/pd-rl-base}
REPEATS=${GZ_REPEATS:-10}
OUT=${1:-$HOME/rl_gz_check/smooth_$(date +%m%d_%H%M%S)}
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_deps_ws/install/setup.bash"
source "$HOME/pinky_deps_ws/install/local_setup.bash"
export CMAKE_PREFIX_PATH=$HOME/ort_probe/x/opt/ros/jazzy:${CMAKE_PREFIX_PATH:-}
export AMENT_PREFIX_PATH=$HOME/ort_probe/x/opt/ros/jazzy:${AMENT_PREFIX_PATH}
source "$B/install/setup.bash"
set -u
f=$B/install/pode_rl_controller/share/pode_rl_controller/policies/store_arena_acd3/policy.onnx
[ -f "$f" ] || { echo "missing $f"; exit 2; }
# 평활 파라미터가 없는 빌드면 세 RL 칸이 같은 것이 되므로 멈춘다
so=$B/install/pode_rl_controller/lib/libpode_rl_controller.so
grep -qa "ema_alpha_w" "$so" || { echo "$so has no ema_alpha_w: build the merged branch first"; exit 2; }
HERE=$SRC/docs/rl_smooth_2026-11-25
BENCH=$SRC/tools/nav_bench
# 지난 판의 구간 기록이 섞이지 않게 이미 돌린 OUT은 받지 않는다
for d in q2e3 main; do
  [ -e "$OUT/$d" ] && { echo "$OUT/$d exists: give a new OUT"; exit 2; }
done
mkdir -p "$OUT"
export BENCH_DOMAIN=${BENCH_DOMAIN:-72} BENCH_OVERRIDES=$HERE/gz_overrides.yaml
# 컨트롤러 목록은 바깥에서 줄일 수 있다(시간이 모자라면 MPPI를 빼는 식)
ALL="FollowPathGoal FollowPathMPPI FollowPathRLShimACD3 FollowPathRLShimACD3W50 FollowPathRLShimACD3W25"
export BENCH_CONTROLLERS=${BENCH_CONTROLLERS:-$ALL}
echo "smooth gz start $(date -Is) out $OUT repeats $REPEATS src $SRC build $B controllers $BENCH_CONTROLLERS" | tee -a "$OUT/gz.txt"

# ---- 멈출 때 run_bench와 그 아래 프로세스를 같이 내린다 ----
CHILD=
marked() { for p in /proc/[0-9]*; do { tr '\0' '\n' < "$p/environ"; } 2>/dev/null | grep -qx "BENCH_RUN=$1" && echo "${p#/proc/}"; done; }
stop_child() {
  [ -n "$CHILD" ] || return
  kill -TERM "$CHILD" 2>/dev/null
  rest=$(marked "$CHILD"); [ -n "$rest" ] && kill -TERM $rest 2>/dev/null
  wait "$CHILD"
}
trap 'stop_child; exit 130' INT TERM
bench() { bash "$BENCH/run_bench.sh" "$@" & CHILD=$!; wait "$CHILD"; local r=$?; CHILD=; return $r; }

# ---- Q2 -> E-3, 그다음 c_moving과 d_robot, 그다음 표 ----
rc=0
BENCH_CONFIG=$SRC/results/2026-11-22/compare_012/q2e3.yaml BENCH_REPEATS=$REPEATS \
  bench "$OUT/q2e3" a_hall || rc=1
sleep 180
BENCH_CONFIG=$BENCH/scenarios.yaml BENCH_REPEATS=$REPEATS \
  bench "$OUT/main" c_moving d_robot || rc=1
echo "gazebo done $(date -Is) rc $rc" | tee -a "$OUT/gz.txt"

python3 "$HERE/gz_results.py" "$OUT" "$OUT" > "$OUT/gz_results.out" 2>&1 || rc=1
echo "smooth gz end $(date -Is) rc $rc" | tee -a "$OUT/gz.txt"
exit $rc
