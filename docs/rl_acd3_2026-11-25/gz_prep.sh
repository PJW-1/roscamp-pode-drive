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
[ -f "$P/store_arena_acd3/policy_spec.json" ] || { echo "no store_arena_acd3/policy_spec.json; export it first"; exit 2; }
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
# ---- 셋의 parity(추론 비교 포함) ----
cd "$B/build/pode_rl_controller" && ctest --output-on-failure 2>&1 | grep -E "Test +#|tests passed|Failed"
for p in 4b acd2 acd3; do ./test_parity_store_arena_$p 2>&1 | grep -E "SKIPPED|FAILED|\[parity\] onnxruntime"; done
