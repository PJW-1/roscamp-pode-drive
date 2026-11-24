#!/usr/bin/env bash
# acd3 대기열: acd2에서 고친 v7 지도로 이어 학습하고 평가한다.
# Isaac 작업마다 .heavy_run.lock을 잡고 놓는다(사이 PAUSE_S). .heavy_run.queue/에 파일이 있으면 기다린다.
# 사용: bash docs/rl_acd_2026-11-24/queue.sh <작업 목록 파일>
# 한 줄에 작업 하나: tag|kind|args. kind: train, resume:<tag>(그 tag 마지막 모델에서 이어 학습),
# eval:<tag>(그 tag 마지막 모델로 play.py), eval(args에 --checkpoint).
# 결과 파일(run.json·eval.json)이 있고 queue_status.txt에 exit=0으로 남은 작업만 끝난 것으로 보고 건너뛴다.
set -u
LIST=${1:?job list}
HERE=$(cd "$(dirname "$0")/../.." && pwd)/learning/store_arena
PY=${PY:-/d/isaac/IsaacLab/.venv/Scripts/python.exe}
LOCK=${LOCK:-/c/Users/pjw/ros2camp/.heavy_run.lock}
QDIR=${QDIR:-/c/Users/pjw/ros2camp/.heavy_run.queue}  # 파일이 있으면 다른 트랙 차례: 잠금을 잡지 않음
R=${R:-/d/isaac/runs/store_arena/acd3}
PAUSE_S=${PAUSE_S:-90}
RETRY_S=${RETRY_S:-30}
export PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1
mkdir -p "$R"
cd "${ISAAC_DIR:-/d/isaac/IsaacLab}" || exit 1
release() { [ "$(cut -d' ' -f1-2 "$LOCK/owner" 2>/dev/null)" = "rl-acd3 $1" ] && rm -rf "$LOCK"; }
last_model() { ls -1v "$R/$1"/model_*.pt 2>/dev/null | tail -1; }
# play.py는 eval.json을 다른 결과보다 먼저 쓰므로, 파일만 있고 성공 기록이 없으면 끝난 것이 아니다
done_ok() { { [ -f "$R/$1/run.json" ] || [ -f "$R/$1/eval.json" ]; } && grep -q "^$1 exit=0 " "$R/queue_status.txt" 2>/dev/null; }
first=1
while IFS='|' read -r tag kind rest; do
  [ -z "$tag" ] && continue
  case "$tag" in \#*) continue ;; esac
  OUT=$R/$tag
  if done_ok "$tag"; then echo "$tag skip (done)"; continue; fi
  case "$kind" in train|resume:*|eval:*|eval) ;; *) echo "$tag: unknown kind $kind"; exit 2 ;; esac
  [ $first -eq 1 ] || sleep "$PAUSE_S"
  first=0
  until [ -z "$(ls -A "$QDIR" 2>/dev/null)" ] && mkdir "$LOCK" 2>/dev/null; do
    echo "$tag: waiting (queue: $(ls -A "$QDIR" 2>/dev/null | tr '\n' ' '); lock: $(cat "$LOCK/owner" 2>/dev/null)); retry in $RETRY_S s"
    sleep "$RETRY_S"
  done
  echo "rl-acd3 $tag $(date -Iseconds) (overnight RL acd3, pd-rl-acd3)" > "$LOCK/owner"
  trap "release $tag" EXIT
  # ---- 잠금을 잡은 뒤 다시 본다: 같은 목록의 다른 큐가 이미 끝냈을 수 있고, 이어 쓸 모델도 지금 골라야 한다 ----
  if done_ok "$tag"; then
    trap - EXIT; release "$tag"; echo "$tag skip (done by another queue)"; continue
  fi
  case "$kind" in
    train|eval) ck="" ;;
    *) ck=$(last_model "${kind#*:}"); [ -n "$ck" ] || { echo "$tag: no model in $R/${kind#*:}"; exit 2; } ;;
  esac
  case "$kind" in
    train) script=train.py; extra="" ;;
    resume:*) script=train.py; extra="--resume $ck" ;;
    eval:*) script=play.py; extra="--checkpoint $ck" ;;
    eval) script=play.py; extra="" ;;
  esac
  mkdir -p "$OUT"
  start=$(date +%s)
  # shellcheck disable=SC2086
  "$PY" "$HERE/$script" --out "$OUT" --viz none $extra $rest < /dev/null > "$OUT/stdout.log" 2>&1
  rc=$?
  # 성공 기록을 잠금을 놓기 전에 남긴다: 기다리던 다른 큐가 잠금을 잡자마자 끝났는지 볼 수 있게
  echo "$tag exit=$rc wall_s=$(( $(date +%s) - start )) $(date -Iseconds)" | tee -a "$R/queue_status.txt"
  trap - EXIT
  release "$tag"
  [ $rc -eq 0 ] || { echo "$tag failed; stopping the queue"; exit $rc; }
done < "$LIST"
exit 0
