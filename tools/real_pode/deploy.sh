#!/usr/bin/env bash
# PC에서 저장소의 커밋된 HEAD를 로봇에 복사한다:  tools/real_pode/deploy.sh pinky@192.168.0.201
# 패키지 -> ROBOT:~/nav2fleet_ws/src/<저장소 경로>/, 실행 묶음(지도 포함) -> ROBOT:~/nav2fleet_real/
# 다른 이름으로 나란히 두기:  DEPLOY_NAME=podedrive deploy.sh ...  (~/podedrive_ws, ~/podedrive_real)
# 실행 묶음에 ws_name(작업공간 이름)을 남긴다. env.sh, make_params.py가 그 작업공간을 읽는다.
# 빌드와 실행은 하지 않는다. 지우기는 각 패키지 폴더 안에서만 한다(logs, params는 남는다).
set -euo pipefail
[ $# -eq 1 ] || { echo "usage: deploy.sh ROBOT_HOST"; exit 2; }
host=$1
name=${DEPLOY_NAME:-nav2fleet}
# 이름은 로봇 홈 아래 폴더 이름이 된다: 소문자로 시작, 소문자·숫자·밑줄만
[[ $name =~ ^[a-z][a-z0-9_]*$ ]] || { echo "bad DEPLOY_NAME: use names like podedrive"; exit 2; }
ws=${name}_ws
real=${name}_real
repo=$(git -C "$(dirname "$(readlink -f "$0")")" rev-parse --show-toplevel)
packages=(interfaces/fleet_interfaces pode/pode_fsm pode/pode_nav_config
  pode/pode_avoidance pode/pode_peer_obstacles pode/pode_scan_filter pode/pode_dock)
kit=tools/real_pode

# ---- 커밋된 내용만 꺼낸다 ----
tmp=$(mktemp -d)
trap 'rm -rf -- "$tmp"' EXIT
git -C "$repo" archive HEAD "${packages[@]}" "$kit" | tar -x -C "$tmp"
if [ -n "$(git -C "$repo" status --porcelain -- "${packages[@]}" "$kit")" ]; then
  echo "주의: 커밋하지 않은 변경은 복사하지 않는다(HEAD만 간다)"
fi
echo "HEAD $(git -C "$repo" rev-parse --short HEAD) -> $host (~/$ws, ~/$real)"
echo "$ws" > "$tmp/$kit/ws_name"

# ---- 복사 ----
ssh "$host" "mkdir -p ~/$ws/src/interfaces ~/$ws/src/pode ~/$real"
for pkg in "${packages[@]}"; do
  rsync -a --delete "$tmp/$pkg/" "$host:~/$ws/src/$pkg/"
done
rsync -a "$tmp/$kit/" "$host:~/$real/"

echo "복사 끝. 로봇에서 빌드한다:"
echo "  cd ~/$ws && colcon build"
echo "띄우기와 내리기: ~/$real/start.sh ..., ~/$real/stop.sh"
