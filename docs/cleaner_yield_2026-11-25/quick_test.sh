#!/bin/bash
# 빠른 시험: 소스를 옮긴 뒤 fleet_core의 주어진 시험만 돈다(인자: pytest 인자)
rsync -a --delete --exclude .git /mnt/c/Users/pjw/ros2camp/pd-cleaner-yield/ ~/nf_src/pd-cleaner-yield/
source /opt/ros/jazzy/setup.bash; source ~/pinky_deps_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=41
cd ~/nf_src/pd-cleaner-yield/pc/fleet_core && python3 -m pytest -q -p no:cacheprovider -x "$@"
