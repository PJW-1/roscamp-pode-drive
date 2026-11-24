"""막힘 꼴 다시 보기: v7 운반 2 + 청소 1, 주어진 seed를 t초까지 돌리고 상태를 찍는다."""
import sys

from fake_robots.world.world import ZoneWorld

seed, until = int(sys.argv[1]), float(sys.argv[2])
w = ZoneWorld(layout="v7", seed=seed, robots={"pode1": "P-2", "pode2": "P-3", "pode3": "P-1"},
              cleaners=("pode3",))
while w.t < until and not w.orders_done():
    w.step()
t = w.manager.traffic
print("t", w.t, "orders_done", w.orders_done())
for name, m in t._robots.items():
    r = w.robots[name]
    route = m.route
    print(name, f"({r.x:.3f},{r.y:.3f})", m.phase, "goal", m.goal, "node", m.node,
          "route", route and (route.goal, [(e.a, e.b) for e in route.edges], route.done,
                              route.granted),
          "holds", t.grants.held_by(name), t.grants.nodes_of(name), "state", m.report.robot_state)
print("waiting", t.grants.waiting())
job = t._cleaners["pode3"]
print("job", job.stage, job.leg and (job.leg.kind, job.leg.target, job.leg.nodes), job.goal_id)
for at, e in w.events[-40:]:
    print(f"{at:7.1f} {e.robot:6s} {e.code} {e.detail}")
