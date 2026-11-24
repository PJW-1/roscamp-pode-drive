#!/bin/bash
# 옛 코드(feat/cleaner-defer 끝)로 옛 지도와 새 지도에서 measure_extra를 돌린다
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=40
D=/mnt/c/Users/pjw/ros2camp/pd-cleaner-sync/docs/cleaner_on_main_2026-11-25
NEW=/mnt/c/Users/pjw/ros2camp/pd-cleaner-sync/pc/fleet_core/fleet_core/data
OLD=~/nf_src/pd-cleaner-old-defer
python3 $D/measure_extra.py $OLD > $D/extra_oldcode_oldmap.yaml 2> $D/extra_oldcode_oldmap.err
python3 $D/measure_extra.py $OLD --map v7=$NEW/fleet_map_v7.yaml --map d2=$NEW/fleet_map_d2.yaml > $D/extra_oldcode_newmap.yaml 2> $D/extra_oldcode_newmap.err
echo done
