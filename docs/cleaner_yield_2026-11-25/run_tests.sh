#!/bin/bash
rsync -a --delete --exclude .git /mnt/c/Users/pjw/ros2camp/pd-cleaner-yield/ ~/nf_src/pd-cleaner-yield/
# 시험: fleet_core 전체, fake_robots 청소 시험, tools 청소 시험, pode_follow_wall colcon test
source /opt/ros/jazzy/setup.bash; source ~/pinky_deps_ws/install/local_setup.bash
B=~/gg_build/pd-cleaner-yield
source $B/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=41
D=/mnt/c/Users/pjw/ros2camp/pd-cleaner-yield/docs/cleaner_yield_2026-11-25
S=~/nf_src/pd-cleaner-yield
cd $S/pc/fleet_core && python3 -m pytest -q test -p no:cacheprovider > $D/test_fleet_core.log 2>&1; echo "fleet_core rc=$?"
cd $S && PYTHONPATH=pc/fleet_core:sim/fake_robots:$PYTHONPATH python3 -m pytest -q sim/fake_robots/test/test_world_cleaner.py -p no:cacheprovider > $D/test_fake_cleaner.log 2>&1; echo "fake_cleaner rc=$?"
cd $S && PYTHONPATH=pc/fleet_core:sim/fake_robots:tools:$PYTHONPATH python3 -m pytest -q -rs tools/test/test_coverage_report.py tools/test/test_cleaner_defer_sweep.py tools/test/test_script_help.py -p no:cacheprovider > $D/test_tools_cleaner.log 2>&1; echo "tools rc=$?"
cd $S && colcon --log-base $B/log test --packages-select pode_follow_wall --build-base $B/build --install-base $B/install > $D/test_follow_wall.log 2>&1; colcon test-result --test-result-base $B/build/pode_follow_wall --verbose >> $D/test_follow_wall.log 2>&1; echo "follow_wall rc=$?"
