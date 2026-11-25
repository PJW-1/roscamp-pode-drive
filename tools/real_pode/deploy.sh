#!/usr/bin/env bash
# PC에서 저장소의 커밋된 HEAD를 로봇에 복사한다:  tools/real_pode/deploy.sh pinky@192.168.0.201
# 패키지 -> ROBOT:~/nav2fleet_ws/src/<저장소 경로>/, 실행 묶음(지도 포함) -> ROBOT:~/nav2fleet_real/
# 빌드와 실행은 하지 않는다. 지우기는 각 패키지 폴더 안에서만 한다(logs, params는 남는다).
# RL controller도 보내려면 WITH_RL=1 deploy.sh ROBOT_HOST: 로봇의 onnxruntime_vendor를 먼저 확인하고,
# pode_rl_controller와 시뮬 RL 설정 파일을 보낸 뒤 그 패키지만 로봇에서 빌드한다.
set -euo pipefail
[ $# -eq 1 ] || { echo "usage: deploy.sh ROBOT_HOST"; exit 2; }
host=$1
repo=$(git -C "$(dirname "$(readlink -f "$0")")" rev-parse --show-toplevel)
packages=(interfaces/fleet_interfaces pode/pode_fsm pode/pode_nav_config
  pode/pode_avoidance pode/pode_peer_obstacles pode/pode_scan_filter pode/pode_dock
  pode/pode_plan_keep)
kit=tools/real_pode
# WITH_RL은 1이면 켜고, 비었거나 0이면 끈다. 그 밖의 값은 거절한다
with_rl=${WITH_RL:-0}
case $with_rl in
  0) with_rl= ;;
  1) ;;
  *) echo "bad WITH_RL: use 1 (or unset/0)"; exit 2 ;;
esac
rl_pkg=pode/pode_rl_controller
rl_yaml=sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml

# ---- RL(선택): 로봇을 바꾸기 전에 onnxruntime_vendor를 확인한다 ----
if [ -n "$with_rl" ]; then
  ort=0
  ssh "$host" 'source /opt/ros/jazzy/setup.bash && ros2 pkg prefix onnxruntime_vendor' > /dev/null || ort=$?
  if [ "$ort" -eq 255 ]; then
    echo "ssh $host failed; nothing copied"; exit 1
  elif [ "$ort" -ne 0 ]; then
    echo "로봇에 onnxruntime_vendor가 없다. 아무것도 복사하지 않았다. 로봇에서 설치한 뒤 다시 한다:"
    echo "  sudo apt update && sudo apt install ros-jazzy-onnxruntime-vendor"
    echo "로봇에 인터넷이 없으면 PC에서 arm64 .deb를 받아 옮긴다:"
    echo "  wget http://packages.ros.org/ros2/ubuntu/pool/main/r/ros-jazzy-onnxruntime-vendor/ros-jazzy-onnxruntime-vendor_0.1.1-1noble.20261026.232151_arm64.deb"
    echo "  sha256sum 값 62c7c56c47f898c58f9f40698268259c6ab322d7fa91ecf2fe960c6967ed2906 확인 뒤 scp, 로봇에서 sudo apt install ./<deb>"
    echo "확인: source /opt/ros/jazzy/setup.bash && ros2 pkg prefix onnxruntime_vendor"
    exit 1
  fi
fi

# ---- 커밋된 내용만 꺼낸다 ----
tmp=$(mktemp -d)
trap 'rm -rf -- "$tmp"' EXIT
git -C "$repo" archive HEAD "${packages[@]}" "$kit" | tar -x -C "$tmp"
if [ -n "$(git -C "$repo" status --porcelain -- "${packages[@]}" "$kit")" ]; then
  echo "주의: 커밋하지 않은 변경은 복사하지 않는다(HEAD만 간다)"
fi
if [ -n "$with_rl" ]; then
  git -C "$repo" archive HEAD "$rl_pkg" "$rl_yaml" | tar -x -C "$tmp"
  if [ -n "$(git -C "$repo" status --porcelain -- "$rl_pkg" "$rl_yaml")" ]; then
    echo "주의: RL 쪽도 커밋하지 않은 변경은 복사하지 않는다(HEAD만 간다)"
  fi
fi
echo "HEAD $(git -C "$repo" rev-parse --short HEAD) -> $host"

# ---- 복사 ----
ssh "$host" 'mkdir -p ~/nav2fleet_ws/src/interfaces ~/nav2fleet_ws/src/pode ~/nav2fleet_real'
for pkg in "${packages[@]}"; do
  rsync -a --delete "$tmp/$pkg/" "$host:~/nav2fleet_ws/src/$pkg/"
done
rsync -a "$tmp/$kit/" "$host:~/nav2fleet_real/"

# ---- RL(선택): 패키지와 설정 파일을 보내고 그 패키지만 빌드한다 ----
if [ -n "$with_rl" ]; then
  ssh "$host" "mkdir -p ~/nav2fleet_ws/src/$(dirname "$rl_yaml")"
  rsync -a --delete "$tmp/$rl_pkg/" "$host:~/nav2fleet_ws/src/$rl_pkg/"
  rsync -a "$tmp/$rl_yaml" "$host:~/nav2fleet_ws/src/$rl_yaml"
  echo "RL: 로봇에서 pode_rl_controller만 빌드한다(gtest 없이, -j1)"
  ssh "$host" 'cd ~/nav2fleet_ws && source /opt/ros/jazzy/setup.bash && MAKEFLAGS=-j1 colcon build --packages-select pode_rl_controller --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF'
  echo "RL: 시작은 로봇에서 RL_POLICY=<정책 이름> ~/nav2fleet_real/start.sh ..."
fi

echo "복사 끝. 로봇에서 빌드한다:"
echo "  cd ~/nav2fleet_ws && colcon build"
