# pode_rl_controller

Nav2 controller plugin `pode_rl_controller::FollowPathRL`: runs an RL driving policy (ONNX, ONNX
Runtime CPU) trained in `learning/goal_reach` or `learning/store_arena`. Design, decisions and results:
`docs/worklog_follow_path_rl_2026-11-21.md`.

| file | what it does |
|---|---|
| `src/policy_spec.cpp` | reads `policy_spec.json` (written by `learning/goal_reach/export_policy.py` from `task_spec.py`), format 2 or 3, and refuses an observation layout it does not build (format 3 adds `obs_blocks`: `prev_action`, `lidar_history`, `speed_cap`; `intent` is refused) |
| `src/observation.cpp` | LaserScan -> the 360 training rays, the 41-dim observation, action -> (v, w), rate limit; path progress and lookahead point (`route_spec.follow`) |
| `src/closing_guard.cpp` | closing stop (`ClosingGuard`, no ROS): the forward-only stop for something that comes closer, the C++ port of `task_spec.ClosingGuard` (see the option below) |
| `src/onnx_policy.cpp` | ONNX Runtime session (CPU, 1 thread, sequential, no spinning) |
| `src/follow_path_rl.cpp` | the plugin: scan subscription, goal point from the plan (the spec's goal rule), policy call, speed cap `max_linear_vel` (0 = the policy's v_max; the sim launch sets it to `nav_max_speed` like RPP and MPPI; never above `pode_nav_config/params/nav_speed.yaml`, and a policy whose v_max differs from that file logs an ERROR at configure and every 10 s while driving), reverse guard (no backing up while a lidar ray within `rear_half_width_deg` of straight back is under `rear_stop_range`, default off (0); a candidate value is 0.12 m / 30 deg), turn in place inside the goal checker's xy tolerance, timing log |
| `policies/<name>/` | `policy.onnx` + `policy_spec.json` (goal rule, lookahead, origin); parameter `policy` picks one, default `store_arena_4b`. Every policy folder's onnx is committed, so a fresh clone can run each policy in the sim and on the robot (`tools/test/test_policy_files.py` keeps it so); `goal_reach_base_4096` keeps only its spec, under `test/fixtures` |
| `test/test_parity.cpp` | gtest against `test/fixtures/<policy>` (made by `export_policy.py --fixtures`), built once per policy |
| `test/test_observation.cpp` | gtest: lookahead point and progress against `route_spec.follow`, goal and velocity terms against `task_spec`, lidar with the yaw-pi mount, reverse guard |
| `test/test_closing_guard.cpp` | gtest: the closing stop on the same scenes as `test_closing_guard_*` and `test_static_ranges_*` in `learning/goal_reach/test_task_spec.py`, reference picking by time (10 Hz scans at 20 Hz control, uneven gaps, a scan gap, the clock going back), parameter checks, and decision parity against `test/fixtures/closing/closing_cases.json` (made by `learning/goal_reach/make_closing_cases.py` from `task_spec.ClosingGuard`) |
| `test/test_format3.cpp` | gtest: format 3 spec loading and refusals, `ObsMemory` (prev_action, lidar history, speed_cap), the cap-aware action mapping, when the memory resets, and observation parity against `test/fixtures/format3/format3_cases.json` (made by `tools/rl_fixtures/make_format3_cases.py` from the recipe's `task_spec.py` and `store_arena_env.lidar_history`) |

Format 3 memory (`ObsMemory`): prev_action is the policy's mapped target before the guard, the speed limit
and the rate limit; the lidar history holds earlier sector vectors, newest first. Both return to the episode
start (prev_action 0, every history slot = the current frame) on activate, `reset()`, a plan whose last pose
moved more than 1 mm, and whenever the policy did not run in the previous control step (more than 2 control
periods since its last call: the RotationShim rotated or the plugin turned in place at the goal).

Deployment options, all off by default (off leaves the command bit-identical, `test_parity` checks it):
- `ema_alpha_v`, `ema_alpha_w` (default 1.0 = off, allowed (0, 1], anything else stops configure): low-pass
  of the policy's mapped target, `out = alpha * target + (1 - alpha) * previous out`, applied after the
  policy and before the reverse guard, the speed limit and the rate limit. The prev_action memory keeps the
  unsmoothed target (what the policy was trained to see). The filter resets wherever `ObsMemory` resets and
  its first output after a reset starts from the command being sent then (`cmd_`: the last command, or the
  measured velocity after a gap). It acts on the policy only, not on the turn in place at the goal. Mean
  added delay at 20 Hz: `0.05 * (1 - alpha) / alpha` s (0.05 s at 0.5, 0.15 s at 0.25).
- `scan_mask_deg` (default empty = off): flat list of base-frame angle pairs in degrees
  `[lo, hi, lo, hi, ...]`, `-180 <= lo < hi <= 180` (a range over the back is written as two pairs), with
  decimal points so YAML reads it as doubles. Beams inside a range become no return (inf, i.e. `obs_max`)
  before the 360 training rays are built, so the reverse guard ignores them too. Meant for a real robot whose
  lidar sees its own body (about 150 to 153 deg, `docs/report_2026-11-24_rl_real_prep.md` S1). An odd count
  or a bad pair stops configure. Never mask all ten training rays of a 10 degree policy sector (the rays
  sit at whole degrees: sector 0 is -180..-171, sector 1 is -170..-161, ...; `[140, 149.5]` already masks all
  of 140..149): that sector then reads `obs_max` ("free for 2 m"). In the Isaac evaluation of
  `store_arena_demohist` (worklog sections 29 and 31) a mask over two whole rear sectors cut the success from
  52.8 to 15.8 %, while a mask of more beams that left at least one ray in every sector showed no difference
  from the base (52.2 %) and was better than leaving the tray posts of pode1 unmasked (48.8 %). The narrow
  mask for those posts is `[140.0, 147.0, 149.0, 155.0, -150.5, -147.5, -144.0, -139.0]` (read from 20 scans
  of one robot, not run on the real robot yet). Applied to the 6312 real scans of the pode1 bag of 2026-11-26
  (worklog section 33) this mask left 2 to 8 rays in every post sector, but those rays sit between and beside
  the posts and often have no return: at least one of the four post sectors read `obs_max` in 35 % of the
  scans (12.5 % while the robot was moving). That is the kind of input that broke the policy when whole
  sectors were masked, so this value is not recommended for the real robot as it is. Each training ray takes the nearest scan beam and a ray with
  no beam near it is no return too, so check with the robot's own scan (its angle range and step) that every
  sector keeps at least one ray whose beam is not masked.
- `scan_mask_fill` (default false = off; on without `scan_mask_deg` stops configure): fills a policy sector
  that the mask emptied from its neighbours, the same rule as `lidar_mask_fill` in
  `learning/goal_reach/task_spec.py`. A training ray is masked when the beam it takes (the nearest one) is
  masked; it is usable when its beam is not masked and returned a finite range within
  `[range_min, range_max]` (a valid return beyond `obs_max` counts as usable: the sector reads `obs_max` and
  is not filled). A sector with at least one masked ray and no usable ray is empty. An empty sector reads
  the smaller of two candidates: on each side, the value of the nearest sector that is not empty, looking at
  most 4 sectors away (the first and the last sector are neighbours). A side with no such sector is left out; with
  none on either side the sector reads `obs_max`. Candidates are the values before the fill, so a filled
  value never feeds another sector. Every other sector is unchanged, so a sector the mask does not touch
  still reads `obs_max` when it has no return. The filled values go into the policy observation and the
  lidar history only; the 360 rays that the reverse guard and the closing stop read keep the masked value
  (no return). Checked in the Isaac evaluation and on recorded real scans only (worklog section 34); not run
  on a robot.
- `closing_stop_range` (default 0.0 = off): closing stop, the forward-only stop for something that moves
  toward the robot. The scan from `closing_lookback_s` ago is moved into the pose of now (each old point
  binned by its bearing from the lidar now, the minimum over `+-closing_spread_deg` of neighbouring bins:
  what the robot would see now if everything had stood still). A ray within `closing_half_width_deg` of
  straight ahead that is under `closing_stop_range` and came closer than that by more than `closing_mps`
  (divided by the real time between the two scans) is caught; `closing_min_rays` caught rays side by side
  trigger the stop (the two ends of the ray array are not joined). From the trigger until
  `closing_hold_s` after the last one, a forward target `v` becomes 0; `w` and a backward target are kept.
  Something standing still does not trigger it even while the robot drives up to it. It acts after the
  reverse guard and before the speed limit and the rate limit, on the policy only (not the turn in place),
  and the prev_action memory keeps the unguarded target. Off, it does nothing, not even a TF lookup.
  On, each new scan (a `header.stamp` not seen before) is added with the lidar pose at the scan stamp from
  TF (`scan->header.frame_id` in the local costmap's global frame, usually `odom`). The lookup does not
  wait, so it cannot hold up the control cycle: while the pose at that stamp is not in TF yet, the same
  scan is tried again in the next cycles, and a scan that a newer one replaces before its pose arrived is
  dropped and counted (`tf misses`). The reference is the newest
  stored scan at least `closing_lookback_s` older than the new one; with none, or one older than
  `lookback + max(0.5 * lookback, 0.15)` s (scans stopped for a while), that scan is not judged. Scans
  are stored only in cycles where the policy runs, so after the policy has not run for longer than that
  (the turn in place at the goal, a rotation shim turning first, a wait between goals) nothing is judged
  until `closing_lookback_s` of new scans are stored. A stamp
  equal to the last one is ignored; a stamp before it (the clock went back) clears the history and the
  hold and is not stored itself, and the hold also ends as soon as the clock reads earlier than the last
  trigger. Otherwise the history and the hold are cleared on `cleanup()` only (not on `reset()` or a new plan: the
  robot does not jump, so the time condition is enough). The 1 s log line ends with
  `| closing blocks N, tf misses M` (counts in that period) and the configure summary lists the values,
  both only when it is on. A bad value (nan or inf, a negative width, lookback, spread or hold, a
  non-positive lookback or closing speed, `min_rays < 1`) stops configure.

  | parameter | default | meaning |
  |---|---|---|
  | `closing_stop_range` | 0.0 (off) | m from the lidar centre; 0 or less = off |
  | `closing_half_width_deg` | 60.0 | half width of the front cone, deg, [0, 180] |
  | `closing_lookback_s` | 1.0 | s between the new scan and its reference, > 0 |
  | `closing_spread_deg` | 4.0 | half width of the neighbour minimum in the old scan, deg, [0, 180] |
  | `closing_mps` | 0.08 | m/s of closing beyond the robot's own motion, > 0 |
  | `closing_hold_s` | 0.3 | s the stop holds after the last trigger, >= 0 |
  | `closing_min_rays` | 2 | caught rays needed side by side, >= 1 |

  The defaults are the long-window values of the Isaac evaluation (worklog section 25 of
  `docs/worklog_rl_tracks_2026-11-26.md`): the margin `closing_mps * closing_lookback_s` is 8 cm, four
  times the 0.02 m range noise of the Gazebo lidar, and a person who crosses in front is caught because
  the place they stand in was empty a second ago. The price: something that comes straight at the robot
  slower than `closing_mps` is not caught. The Python `ClosingGuard` keeps the short-window defaults of
  the first evaluation (0.2 s, 8 deg, 0.04 m/s, 1 ray); do not copy those here: in a 2D check with
  0.02 m scan noise they stopped the robot in 28 % of the steps of scenes where nothing moved.

  Two intended differences from the Python `ClosingGuard`: (a) Python never uses the first scan after a
  reset as a reference (the sim teleports the robot on reset); the plugin checks the time condition only.
  (b) Python holds for a number of steps (`round(hold_s / step_dt)`), the plugin for a time. The Python
  `noise_m` (simulated range noise) is not ported. The guard reads the 360 training rays, so a beam with no
  return, outside `[range_min, range_max]` or masked is at `obs_max` (2 m in the current policies) where
  the Isaac evaluation fed raw distances; the judgement can differ only when `closing_stop_range` is above
  `obs_max - closing_mps * 1.5 * closing_lookback_s` (1.88 m with the defaults; the two scans can be up to
  1.5 lookbacks apart). The rule was measured in the Isaac evaluation (worklog section 25) and in Gazebo (section 26, 5 runs per
  scenario): with a person who paces across the aisle without yielding, the robot stopped for the first
  crossing in every run, but no run both avoided contact and arrived: 9 of the 10 runs made contact later
  (6 of them while the robot stood in the person's lane) and the other one timed out. Arrival took 1.8 to
  3.7 times as long and the robot stood long enough to trip the 1 s stall watch of `pode_fsm`. A parked cart
  was passed as before (the rule blocked 3.5 % of the control cycles there). With a person who crosses only
  once (section 30, 5 runs per arm, a weak verdict from one scenario) the robot with the rule on waited and
  then passed without contact in 5 of 5 runs where the one without it made contact in 5 of 5; the rule
  blocked 23 % of the control cycles there. It has not been run on the real robot.

Not loaded by default: `sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml` adds two controllers to
one_pode (`nav_overrides:=<that file>`): `FollowPathRL`, the default RL option, is the policy wrapped in
RotationShim; `FollowPathRLAlone` is the same policy unwrapped. `controller_selector` picks one.

Policies in this repository (the lineage, training settings and results of each are in
`learning/store_arena/POLICIES.md` once that file is merged):

| policy | inputs | spec format | checkpoint | onnx sha256 (first 12) | note |
|---|---|---|---|---|---|
| `store_arena_4b` | 41 | 2 | `c2/model_998.pt` | `fd7519b279cc` | default; first 0.12 m/s policy |
| `store_arena_acd` | 44 | 3 | `t_combo_acd/model_1197.pt` | `a537541fb725` | comparison: first a+c+d recipe run |
| `store_arena_acd2` | 44 | 3 | `t_acd_s3_c2/model_1496.pt` | `d6f98cfc5bf6` | a+c+d, seed 3 of three |
| `store_arena_acd3` | 44 | 3 | `t_wall/model_1745.pt` | `c11b9d29b223` | acd2 continued on the corrected map with the near-wall penalty; the user's real-robot candidate |
| `store_arena_c` | 41 | 2 | `t_c/model_1197.pt` | `ccaf99c15f72` | comparison: asymmetric critic only |
| `store_arena_demohist` | 116 | 3 | `t_demo_hist/model_2044.pt` | `49747b8717d7` | acd3 with two earlier lidar frames added, action std reset to 0.3, continued 300 iterations with the demo conditions mixed in (training code `51b51d39`) |
| `store_arena_std` | 44 | 3 | `t_std/model_2044.pt` | `60c4ac704736` | acd3 with action std reset to 0.3, continued 300 iterations (training code `52719475`) |
| `store_arena_demohist2` | 116 | 3 | `t_demo_hist2/model_2643.pt` | `2b58f8fbb442` | demohist continued 600 more iterations with the same settings (training code `74e82c53`); checked in the sim only, never run on the real robot |

To drive another policy in the sim, pass its overrides file: `nav_overrides:=<share>/nav_overrides/follow_path_rl_acd3.yaml`
runs `store_arena_acd3` under both RL controller names. A new policy needs its folder here (spec + onnx), its
fixtures under `test/fixtures/<name>`, its name in the parity list in `CMakeLists.txt`, and, to be selectable in the
sim, a `follow_path_rl_<suffix>.yaml` that differs from `follow_path_rl.yaml` only in `policy`.

Build needs `onnxruntime_vendor` (apt `ros-jazzy-onnxruntime-vendor`, ORT 1.24.3; amd64 and arm64).
Without sudo, unpack the .deb with `dpkg -x` and put `<dir>/opt/ros/jazzy` on `CMAKE_PREFIX_PATH`;
the library is found through its RPATH at run time.

```bash
colcon build --packages-select pode_rl_controller
colcon test --packages-select pode_rl_controller --event-handlers console_direct+
```
