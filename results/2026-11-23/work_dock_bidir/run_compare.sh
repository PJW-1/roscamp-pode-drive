#!/usr/bin/env bash
# WORK 양방향 비교 묶음(2026-11-23): 같은 install, 같은 시나리오로 v7과 v7_bidir를 한 판씩 차례로.
# 사용: run_compare.sh INSTALL_SETUP OUT_DIR   (도메인 64, 화면 없음, nav_max_speed 0.12)
# 시나리오: 11-22 ETA 원인 조사와 같은 rush1~3(gz_v7_rush) + v7a~c(gz_v7)
S=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
BATCH=$S/../../../docs/baseline_sim_2026-11-19/sim_batch.sh
INSTALL=$1; OUT=$2
RUNS="rush1:gz_v7_rush rush2:gz_v7_rush rush3:gz_v7_rush v7a:gz_v7 v7b:gz_v7 v7c:gz_v7"
export SIM_BATCH_INSTALL=$INSTALL SIM_BATCH_DOMAIN=64 SIM_BATCH_PAUSE_S=10
export SIM_BATCH_ARGS="rviz:=none gui:=false nav_max_speed:=0.12"
for layout in v7 v7_bidir; do
  mkdir -p "$OUT/$layout"
  SIM_BATCH_LAYOUT=$layout bash "$BATCH" "$OUT/$layout" - $RUNS 2>&1 | tee "$OUT/$layout/batch.log"
done
echo "compare done"
