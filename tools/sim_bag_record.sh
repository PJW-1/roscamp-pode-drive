#!/bin/bash
# 시뮬용 bag 녹화: 실물 bag_record.sh와 같은 토픽에 참값(sim/truth)과 /clock을 더한다
# 쓰기: ROS_DOMAIN_ID=<시뮬 도메인> tools/sim_bag_record.sh NAMESPACE OUT_DIR   (Ctrl-C로 멈춤)
# 분석: python3 tools/coverage_report.py metrics OUT_DIR --ns NAMESPACE
set -eu

usage() { echo "usage: $0 NAMESPACE OUT_DIR"; exit 2; }
[ $# -eq 2 ] || usage
ns=$1 out=$2
[[ $ns =~ ^[a-z_][a-z0-9_]*$ ]] || { echo "bad NAMESPACE: $ns"; exit 2; }
[ -e "$out" ] && { echo "$out exists; use another OUT_DIR"; exit 1; }
command -v ros2 > /dev/null || { echo "ros2 not found; source /opt/ros/jazzy/setup.bash first"; exit 1; }

# 녹화 시각도 시뮬 시계로 찍는다
exec ros2 bag record -o "$out" -s mcap --use-sim-time \
  -e "^/$ns/(scan|odom|cmd_vel|amcl_pose|particle_cloud|plan|plan_kept|tf|tf_static|state|sim/truth)$|^/clock$"
