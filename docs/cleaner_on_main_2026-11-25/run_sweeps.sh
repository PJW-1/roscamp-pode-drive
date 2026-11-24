#!/bin/bash
# 미룸 비교 표: 옛 코드 + 새 지도, 새 코드(새 지도). 같은 조건 16개, seed 1~5
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=40
D=/mnt/c/Users/pjw/ros2camp/pd-cleaner-sync/docs/cleaner_on_main_2026-11-25
NEWSRC=~/nf_src/pd-cleaner-sync
OLD=~/nf_src/pd-cleaner-old-defer
JOBS=${JOBS:-11}
for which in "$@"; do
  case $which in
    oldcode_newmap)
      cd $OLD && PYTHONPATH=pc/fleet_core:sim/fake_robots:$PYTHONPATH /usr/bin/time -f "%e s" python3 tools/cleaner_defer_sweep.py --jobs $JOBS \
        --map v7=$NEWSRC/pc/fleet_core/fleet_core/data/fleet_map_v7.yaml --map d2=$NEWSRC/pc/fleet_core/fleet_core/data/fleet_map_d2.yaml \
        --json $D/defer_oldcode_newmap.json --md $D/defer_oldcode_newmap.md > /dev/null 2> $D/defer_oldcode_newmap.err ;;
    oldcode_oldmap)
      cd $OLD && PYTHONPATH=pc/fleet_core:sim/fake_robots:$PYTHONPATH /usr/bin/time -f "%e s" python3 tools/cleaner_defer_sweep.py --jobs $JOBS \
        --json $D/defer_oldcode_oldmap.json --md $D/defer_oldcode_oldmap.md > /dev/null 2> $D/defer_oldcode_oldmap.err ;;
    new)
      cd $NEWSRC && PYTHONPATH=pc/fleet_core:sim/fake_robots:$PYTHONPATH /usr/bin/time -f "%e s" python3 tools/cleaner_defer_sweep.py --jobs $JOBS \
        --json $D/defer_new.json --md $D/defer_new.md > /dev/null 2> $D/defer_new.err ;;
  esac
  echo "$which rc=$?"
done
