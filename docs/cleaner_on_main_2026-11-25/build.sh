#!/bin/bash
B=~/gg_build/pd-cleaner-sync; mkdir -p $B
rsync -a --delete --exclude .git /mnt/c/Users/pjw/ros2camp/pd-cleaner-sync/ ~/nf_src/pd-cleaner-sync/
source /opt/ros/jazzy/setup.bash; source ~/pinky_deps_ws/install/local_setup.bash
cd ~/nf_src/pd-cleaner-sync
B=~/gg_build/pd-cleaner-sync
colcon --log-base $B/log build --build-base $B/build --install-base $B/install --packages-skip pode_rl_controller --event-handlers console_direct- > $B/../pd-cleaner-sync-build.out 2>&1; rc=$?; tail -30 $B/../pd-cleaner-sync-build.out; exit $rc
