# WORK_DOCK early release in test_fms_restart (2026-11-23, diagnosis only)

Symptom: `pc/fleet_manager/test/test_fms_restart.py` fails now and then with
`t=401.6 WORK_DOCK released from pode3 while 0.08 m from it` (true distance 0.077 m, monitor
limit = robot radius 0.08 m). No code was changed.

## Cause: fleet (node-pose shortcut + M_E at exactly the release gap)

- `pc/fleet_core/fleet_core/traffic/zone/progress.py:113-115`: when the reported pose is within
  `release_margin_m` (0.05) of `m.node`, the distance to the zone behind is measured from the
  node, not from the report.
- `pc/fleet_core/fleet_core/data/fleet_map_v7.yaml:155`: `M_E` (1.645, 0.55) is exactly 0.135 m
  (= `footprint_radius_m` 0.085 + `release_margin_m` 0.05) from `WORK_DOCK` (x >= 1.78). So any
  report within 0.05 of M_E counts as clear: a report 0.085 m from the dock is enough, and
  with the fake's report noise (Gaussian, sigma 0.03 per axis, 10 Hz) the body can still be in
  the zone. Three consecutive samples (`release_confirm_samples`) do not help much: the
  samples are independent and the robot crawls (about 0.02 m/s sim in this scenario).
- `m.node` becomes M_E early: the fake (like the real Pode) passes a nav_goal segment
  `nav_pass_m` 0.15 m before its end when the next segment is queued
  (`sim/fake_robots/fake_robots/pode_order.py:200-213`), and `_order_progress`
  (`order_answers.py:106-121`) then sets `m.node = M_E`.
- Only pode3 leaves W by W -> M_E (nav) in this scenario; pode1 and pode2 take the express via
  W_X, which the map already moved to 0.2006 m for this exact reason (map comment at
  `fleet_map_v7.yaml:107-116`). The guard test
  `test_layout_v7.py:725 test_v7_pass_node_shortcut_cannot_release_a_dock_early` only checks
  `PASS` nodes on fixed edges, so it skips M_E (`fork`, nav edge).
- Same geometry elsewhere (nodes outside a dock zone closer than 0.185 m to it): Q1 0.135
  (WORK_DOCK), O_IN 0.140 and O_OUT 0.145 (OUT_DOCK).

## Evidence

- Failing run (load: Isaac training): pode3 granted `WORK_DOCK+HALL to O_IN` at 393.6,
  `SEGMENT_DONE seq 1 (nav_goal) to M_E` at 397.4 while its reports were 0.15-0.18 m from M_E,
  WORK_DOCK released at 401.6 with true 0.077 m.
- WORK_DOCK release distances for pode3 (true, latest report) in surviving runs: 0.077/0.141,
  0.119/0.106, 0.111/0.165, 0.149/0.124, 0.161/0.220, 0.153/0.189, 0.111/0.160, 0.119/0.123;
  pode1/pode2 (express) 0.137-0.159. Reports below 0.135 at release show the shortcut firing.
- Scratch unit test (Harness, v7, pode1 W -> M_E with pode2 at O_X and pode3 at O, robot frozen
  0.045 m short of M_E): WORK_DOCK released with the robot 0.090 m from it, both with and
  without the order model.
- Monte Carlo of the rule (straight W -> M_E at 0.02 m/s, sigma 0.03, 10 Hz, 3 samples):
  P(true <= 0.08 m at release) about 7 % with the shortcut, 0.01 % without.
- 18 reruns on main 9cbd413 without the Isaac job (WSL load 3-5): 17 passed, 1 failed
  (run8: `t=398.3 WORK_DOCK released from pode3 while 0.07 m from it`, true 0.073 m). pode3's
  WORK_DOCK release true distance ranged 0.073-0.143 m (0.089 and 0.091 were near misses); the
  latest report was below 0.135 m in 11 of 18 runs. Logs: WSL `~/fake_speed_out/dockcheck/`.
  So it is not a load artefact; load only changes which noise samples line up.

## Proposed fix (not applied)

Use the node pose only when the node clears the zone by the margin as well, per zone:
`site.distance_to_zone(*node, zone) - p.release_margin_m >= gap`; otherwise measure from the
report. (By the triangle inequality a report within the margin of such a node is already
`gap` away, so this amounts to dropping the shortcut; it only ever changed the outcome for
nodes closer than gap + margin.) M_E and Q1 would then release from the report; O_OUT (0.145) too, which undoes the
2026-11-15 O_OUT handover speedup (9d43c78) unless O_OUT moves to >= 0.185 m or that case is
kept by another rule. That trade-off is a user decision.

Test that catches it: widen `test_v7_pass_node_shortcut_cannot_release_a_dock_early` to every
node at the end of any edge (fixed or nav) leaving a dock zone, whatever its kind; M_E fails
today (0.135 - 0.05 = 0.085 < 0.135). Plus a Harness test like the scratch one: robot 0.045 m
short of M_E after seq 1 is done must keep WORK_DOCK.
