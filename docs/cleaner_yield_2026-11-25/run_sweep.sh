#!/bin/bash
# 미룸 표 다시 재기: 재측정 보고서와 같은 도구와 seed 1~5. 인자: 출력 이름, 조건들(기본 default)
NAME=${1:-defer_new}; shift
CONDS=${*:-default}
rsync -a --delete --exclude .git /mnt/c/Users/pjw/ros2camp/pd-cleaner-yield/ ~/nf_src/pd-cleaner-yield/
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=40
D=/mnt/c/Users/pjw/ros2camp/pd-cleaner-yield/docs/cleaner_yield_2026-11-25
cd ~/nf_src/pd-cleaner-yield && PYTHONPATH=pc/fleet_core:sim/fake_robots:$PYTHONPATH /usr/bin/time -f "%e s" \
  python3 tools/cleaner_defer_sweep.py --jobs ${JOBS:-12} -c $CONDS \
  --json $D/$NAME.json --md $D/$NAME.md > /dev/null 2> $D/$NAME.err
echo "$NAME rc=$?"
