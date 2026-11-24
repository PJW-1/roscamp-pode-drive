#!/bin/bash
# 새 지도로 coverage data 파일과 그림을 다시 만든다
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/pjw/ros2camp/pd-cleaner-sync/pc/fleet_core
python3 -m fleet_core.coverage --layout v7 --write 2>/dev/null
python3 -m fleet_core.coverage --layout d2 --write 2>/dev/null
cd ../..
PYTHONPATH=pc/fleet_core:$PYTHONPATH python3 tools/coverage_plot.py -h | head -5
