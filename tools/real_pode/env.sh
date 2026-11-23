# 실물 Pode 시험 환경. start.sh가 읽는다. 로봇에서 ros2 명령을 칠 때도 먼저 source한다
# 작업공간은 옆의 ws_name(deploy.sh가 남김)이 정한다. 없으면 예전 자리 ~/nav2fleet_ws
_env_ws=nav2fleet_ws
_env_name="$(dirname "${BASH_SOURCE[0]}")/ws_name"
if [ -f "$_env_name" ]; then
  # 첫 줄을 공백으로 나눠 정확히 한 단어여야 한다(CR은 지운다. make_params.py와 같은 규칙)
  _env_ws= _env_rest=
  read -r _env_ws _env_rest < <(head -n 1 "$_env_name" | tr -d '\r')
  [[ -z $_env_rest && $_env_ws =~ ^[a-z][a-z0-9_]*$ ]] || {
    echo "env.sh: 이상한 ws_name ($_env_name)"; unset _env_ws _env_name _env_rest; return 1; }
  unset _env_rest
fi
for f in "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}" "$HOME/pinky_pro/install/local_setup.bash" \
    "$HOME/$_env_ws/install/local_setup.bash"; do
  [ -f "$f" ] || { echo "env.sh: 없음 $f (${_env_ws}는 colcon build 뒤에 생긴다)"
    unset _env_ws _env_name; return 1; }
  source "$f"
done
unset _env_ws _env_name
# 도메인, RMW, 디스커버리 서버는 net.sh 한곳에 둔다
source "$(dirname "${BASH_SOURCE[0]}")/net.sh"
