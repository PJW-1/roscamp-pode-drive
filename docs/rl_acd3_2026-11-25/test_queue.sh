#!/usr/bin/env bash
# queue.sh 시험: Isaac 대신 가짜 python으로 큐 동작만 본다(잠금·결과 폴더는 모두 임시 폴더).
#   bash docs/rl_acd3_2026-11-25/test_queue.sh   (실패하면 0이 아닌 값으로 끝남)
set -u
Q=$(cd "$(dirname "$0")" && pwd)/queue.sh
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
fails=0
check() { if eval "$2"; then echo "ok   $1"; else echo "FAIL $1"; fails=$((fails + 1)); fi; }

# ---- 가짜 python: 부른 기록을 남기고, FAKE_FAIL 작업은 첫 번째에만 eval.json을 쓴 뒤 실패한다 ----
cat > "$T/fakepy.sh" <<'EOF'
#!/usr/bin/env bash
out=; ck=none; script=$(basename "$1")
while [ $# -gt 0 ]; do case "$1" in --out) out=$2; shift ;; --checkpoint|--resume) ck=$2; shift ;; esac; shift; done
tag=$(basename "$out")
echo "$tag $script $(cat "$ck" 2>/dev/null || echo none)" >> "$CALLS"
sleep 2
if [ "$script" = train.py ]; then echo "weights-$tag-$$" > "$out/model_5.pt"; echo '{}' > "$out/run.json"; exit 0; fi
echo '{}' > "$out/eval.json"
if [ "$tag" = "${FAKE_FAIL:-}" ] && [ ! -f "$out/failed_once" ]; then touch "$out/failed_once"; exit 1; fi
exit 0
EOF
chmod +x "$T/fakepy.sh"

setup() {
  export R=$T/$1/runs PY=$T/fakepy.sh LOCK=$T/$1/lock QDIR=$T/$1/q CALLS=$T/$1/calls.txt
  export ISAAC_DIR=$T PAUSE_S=0 RETRY_S=1
  mkdir -p "$R"; : > "$CALLS"
}
count() { grep -c "^$1 " "$CALLS"; }

# ---- 같은 목록을 두 큐가 동시에 돌면 작업마다 한 번씩만, 평가는 그때 있는 모델로 ----
setup two
printf 't1|train|a\ne1|eval:t1|b\ne2|eval:t1|c\n' > "$T/two.txt"
bash "$Q" "$T/two.txt" > "$T/two/q1.log" 2>&1 & a=$!
bash "$Q" "$T/two.txt" > "$T/two/q2.log" 2>&1 & b=$!
wait $a; ra=$?; wait $b; rb=$?
check "two queues both end 0" '[ $ra -eq 0 ] && [ $rb -eq 0 ]'
check "t1 runs once" '[ "$(count t1)" -eq 1 ]'
check "e1 runs once" '[ "$(count e1)" -eq 1 ]'
check "e2 runs once" '[ "$(count e2)" -eq 1 ]'
check "e1 reads the trained model" 'grep -q "^e1 play.py weights-t1-" "$CALLS"'
check "lock released" '[ ! -d "$LOCK" ]'

# ---- eval.json을 쓰고 실패한 평가는 다시 돌릴 때 건너뛰지 않는다 ----
setup fail
printf 'f1|eval|--checkpoint none\n' > "$T/fail.txt"
FAKE_FAIL=f1 bash "$Q" "$T/fail.txt" > "$T/fail/q1.log" 2>&1; r1=$?
FAKE_FAIL=f1 bash "$Q" "$T/fail.txt" > "$T/fail/q2.log" 2>&1; r2=$?
check "failed eval stops the queue" '[ $r1 -ne 0 ]'
check "rerun runs the failed eval again" '[ "$(count f1)" -eq 2 ] && [ $r2 -eq 0 ]'

# ---- 끝남은 그 작업의 마지막 기록으로, 이름은 글자 그대로 본다 ----
setup last
mkdir -p "$R/g1" "$R/a.b"; echo '{}' > "$R/g1/eval.json"; echo '{}' > "$R/a.b/eval.json"
printf 'g1 exit=0 wall_s=1 x\ng1 exit=1 wall_s=1 x\naxb exit=0 wall_s=1 x\n' > "$R/queue_status.txt"
printf 'g1|eval|--checkpoint none\na.b|eval|--checkpoint none\n' > "$T/last.txt"
bash "$Q" "$T/last.txt" > "$T/last/q.log" 2>&1
check "an older exit=0 does not hide a later failure" '[ "$(count g1)" -eq 1 ]'
check "a regex tag does not match another tag" '[ "$(count a.b)" -eq 1 ]'

# ---- 이어 쓸 모델이 없으면 python을 부르지 않고 멈추며 잠금을 놓는다 ----
setup nomodel
printf 'e9|eval:nope|x\n' > "$T/nomodel.txt"
bash "$Q" "$T/nomodel.txt" > "$T/nomodel/q.log" 2>&1; r=$?
check "missing model stops the queue" '[ $r -ne 0 ] && grep -q "no model" "$T/nomodel/q.log"'
check "python not called without a model" '[ "$(count e9)" -eq 0 ]'
check "lock released after the stop" '[ ! -d "$LOCK" ]'

echo "failures: $fails"
exit $((fails > 0))
