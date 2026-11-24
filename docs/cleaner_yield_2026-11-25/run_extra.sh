#!/bin/bash
# 이 브랜치 코드로 measure_extra(청소 경로 요약, 경로 최소 거리, 바닥 물체 판)를 돌린다
rsync -a --delete --exclude .git /mnt/c/Users/pjw/ros2camp/pd-cleaner-yield/ ~/nf_src/pd-cleaner-yield/
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=40
D=/mnt/c/Users/pjw/ros2camp/pd-cleaner-yield/docs/cleaner_yield_2026-11-25
python3 $D/measure_extra.py ~/nf_src/pd-cleaner-yield > $D/extra_new.yaml 2> $D/extra_new.err
echo rc=$?
