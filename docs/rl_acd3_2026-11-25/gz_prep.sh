#!/usr/bin/env bash
# acd3 Gazebo 비교 준비(WSL): 소스를 옮기고, 저장소에 없는 onnx 둘을 넣고, 전체를 빌드한다.
#   bash gz_prep.sh <acd2 onnx> <acd3 onnx>   (Windows 경로는 /mnt/c/..., /mnt/d/...로 준다)
set -e
ACD2=${1:?acd2 policy.onnx}
ACD3=${2:?acd3 policy.onnx}
W=pd-rl-acd3
SRC=$HOME/nf_src/$W
B=$HOME/gg_build/$W
rsync -a --delete --exclude .git /mnt/c/Users/pjw/ros2camp/$W/ "$SRC/"
P=$SRC/pode/pode_rl_controller/policies
# 내보내기(export_policy.py)로 spec과 fixture를 worktree에 두고 CMake parity 목록에 acd3를 더한 뒤 돌린다
[ -f "$P/store_arena_acd3/policy_spec.json" ] || { echo "no store_arena_acd3/policy_spec.json; export it first"; exit 2; }
grep -q "store_arena_acd3" "$SRC/pode/pode_rl_controller/CMakeLists.txt" || { echo "add store_arena_acd3 to the parity list in CMakeLists.txt"; exit 2; }
cp "$ACD2" "$P/store_arena_acd2/policy.onnx"
cp "$ACD3" "$P/store_arena_acd3/policy.onnx"
# ---- 빌드(onnxruntime은 ort_probe 묶음) ----
set +e
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_deps_ws/install/setup.bash"
source "$HOME/pinky_deps_ws/install/local_setup.bash"
ort=$HOME/ort_probe/x/opt/ros/jazzy
export CMAKE_PREFIX_PATH=$ort:$CMAKE_PREFIX_PATH AMENT_PREFIX_PATH=$ort:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=$ort/lib:$ort/opt/onnxruntime_vendor/lib:$LD_LIBRARY_PATH
mkdir -p "$B"
cd "$SRC" || exit 1
colcon --log-base "$B/log" build --build-base "$B/build" --install-base "$B/install" \
  > "$B/build.txt" 2>&1 || { tail -30 "$B/build.txt"; exit 1; }
tail -2 "$B/build.txt"
# ---- 셋의 parity(추론 비교 포함): 실패나 추론 건너뜀이 하나라도 있으면 멈춘다 ----
cd "$B/build/pode_rl_controller" || exit 1
ctest --output-on-failure > "$B/ctest.txt" 2>&1; r=$?
grep -E "Test +#|tests passed|Failed" "$B/ctest.txt"
[ $r -eq 0 ] || { echo "ctest failed ($r), see $B/ctest.txt"; exit 1; }
for p in 4b acd2 acd3; do
  ./test_parity_store_arena_$p > "$B/parity_$p.txt" 2>&1; r=$?
  grep -E "SKIPPED|FAILED|\[parity\] onnxruntime" "$B/parity_$p.txt"
  [ $r -eq 0 ] || { echo "parity $p failed ($r)"; exit 1; }
  ! grep -q "SKIPPED" "$B/parity_$p.txt" || { echo "parity $p skipped inference: is policy.onnx in place?"; exit 1; }
done
