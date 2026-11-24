"""v7 운반 2 + 청소 1 한 판에서 청소 로봇의 비키기·집·막힘 사건을 찍고 까닭별로 센다."""
import sys
from collections import Counter

from fake_robots.world.world import ZoneWorld

seed = int(sys.argv[1])
w = ZoneWorld(layout="v7", seed=seed, robots={"pode1": "P-2", "pode2": "P-3", "pode3": "P-1"},
              cleaners=("pode3",))
laps = 0
while w.t < 900 and not (w.orders_done() and laps >= 1):
    w.step()
    laps = sum(e.code == "CLEAN_LAP_DONE" for _, e in w.events)
kinds = Counter()
for at, e in w.events:
    if e.robot == "pode3" and e.code in ("CLEAN_YIELD", "CLEAN_HOME", "CLEAN_STUCK",
                                          "CLEAN_AT_HOME", "CLEAN_LAP_DONE"):
        print(f"{at:6.1f} {e.code} {e.detail}")
        if e.code == "CLEAN_YIELD":
            why = e.detail.split(": ", 1)[-1]
            kinds[why.split(" (")[0].replace("pode1", "X").replace("pode2", "X")] += 1
print("t", w.t, "orders_done", w.orders_done())
for k, v in kinds.most_common():
    print(v, k)
