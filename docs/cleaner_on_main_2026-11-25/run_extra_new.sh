#!/bin/bash
# 새 코드(이 브랜치)와 새 지도로 measure_extra를 돌린다
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=40
D=/mnt/c/Users/pjw/ros2camp/pd-cleaner-sync/docs/cleaner_on_main_2026-11-25
python3 $D/measure_extra.py ~/nf_src/pd-cleaner-sync > $D/extra_new.yaml 2> $D/extra_new.err
echo rc=$?
