#!/bin/bash
B=~/gg_build/pd-cleaner-yield; mkdir -p $B
rsync -a --delete --exclude .git /mnt/c/Users/pjw/ros2camp/pd-cleaner-yield/ ~/nf_src/pd-cleaner-yield/
source /opt/ros/jazzy/setup.bash; source ~/pinky_deps_ws/install/local_setup.bash
cd ~/nf_src/pd-cleaner-yield
B=~/gg_build/pd-cleaner-yield
colcon --log-base $B/log build --build-base $B/build --install-base $B/install --packages-skip pode_rl_controller --event-handlers console_direct- > $B/../pd-cleaner-yield-build.out 2>&1; rc=$?; tail -30 $B/../pd-cleaner-yield-build.out; exit $rc
