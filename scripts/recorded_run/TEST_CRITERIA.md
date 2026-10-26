# Recorded validation run - pre-registered numerical criteria (BEFORE the run)

Intended demo: from a fresh clone, launch the hard world; the robot drives the authored
route from spawn to the goal room. Guidance is WORLD-ORACLE waypoints (route file from
known geometry) - not autonomous exploration, not a swept-path safety proof.
These criteria are fixed before execution; no retrofitted pass.

## Config under test
- Repo commit under review (lead integration), world: hard_world.sdf
- Params: nav2_params_recorded_footprint.yaml = repo nav2_params.yaml with
  robot_radius: 0.53 replaced by the derived footprint polygon in BOTH costmaps.
  inflation_radius unchanged at 0.70 - the ACTUAL merged value session B ran with
  (source: launch merged-params tempfile /tmp/tmp8dj1fzna.yaml lines 209/270,
  preserved as sessionB_merged_nav2_params.yaml; my earlier "0.55" report was wrong).
  Inflation is tuning, not geometry-dictated; hardtest 0.60/0.90 profile is dropped.
- Driver: nav_driver5.py (full sec+nanosec sim stamps, per-leg + global timeout, JSONL log)
- Recorder: run_recorder5.py (TF 10 Hz incl. failure events, /odom full rate, /clock 1 Hz,
  gz GT pose stream, rosbag /tf /odom /clock)
- Isolation: ROS_DOMAIN_ID=42, GZ_PARTITION=hardmap, gui:=false, launch_rviz:=false

## Footprint derivation (from mir_ros2.urdf.xacro collision primitives)
chassis box 0.9 x 0.58 @ x=+0.037646; front laser pod 0.16x0.16 @ (0.4288, 0.2358);
back laser pod 0.16x0.16 @ (-0.3548, -0.2352); pods' joint rpy rotates them 45 deg
(true extent 0.591 m); unrotated reading = 0.599 m (lead audit). The polygon is a
CONSERVATIVE convex hull covering BOTH rotation readings (8 vertices, enclosing radius
0.5988 m) - "conservative", not "exact": a convex hull also rounds over concavities.
Repo default 0.53 left pod corners outside; 0.60 encloses. Footprint (base_footprint frame):
[[-0.4679, -0.2352], [-0.4348, -0.3152], [-0.3548, -0.3483], [0.4876, -0.2900],
 [0.5419, 0.2358], [0.5088, 0.3158], [0.4288, 0.3489], [-0.4124, 0.2900]]

## Hard pass/fail (all must hold)
- C1 sequence (explicit denominator): driver.jsonl must contain result events for leg
  indices EXACTLY 1..15 (waypoints[1..15], last = final goal), each status 4, in order.
  The driver refuses to start if its nearest-waypoint resume would begin anywhere but
  index 1 on a fresh spawn (start_index_mismatch, exit 8) - no silent leg skipping.
  Any status != 4, missing index, leg timeout, or global timeout => FAIL.
- C2 physical arrival per leg: gz GT distance to the leg target at result time <= 0.35 m
  (0.25 goal tolerance + 0.10 margin). Catches the frozen-pose instant-success failure
  mode from session B. Any leg above => FAIL.
- C3 final arrival: final GT sample within 0.50 m of true goal (4.31, 0.56). Else FAIL.
- C4 clearance: SWEPT-footprint check - for every pair of consecutive GT samples, the
  convex hull of the footprint at both samples (encloses the translation sweep; rotation
  sweep conservative for small dtheta) must clear all 65 wall boxes parsed from the SDF
  by > 0.0 m. Any swept segment at <= 0 => FAIL. Samples in the 0-0.05 m warn band are
  reported, as is the max inter-sample step (expected ~7.5 mm at 0.45 m/s and 60 Hz
  pose/info; a large step widens the residual-risk gap between samples and is flagged).
  HONEST LABEL: this is geometric no-intersection evidence from ground truth, NOT a
  contact-sensor log - this gz world/model exposes no contact topic, so no direct
  contact proof exists. Clearance > 0 under a conservative hull + dense GT samples is
  the strongest available evidence and implies no physical contact within sampling bounds.
- C5 recovery budget: 0 action-level aborts; BT recovery retries (costmap clear) <= 3.
  Else FAIL.
- Timeouts: 120 sim-s per leg, 900 sim-s global (session B needed ~350 sim-s).

## Report-only metrics (recorded, not pass/fail)
RPP collision-ahead count; follow_path abort count; per-leg durations; final map->odom
translation/rotation; odom-vs-GT heading divergence over time; min-clearance value and
location; map growth timeline.

## Run procedure
1. fresh clone + colcon build (no manual code fixes; user acceptance standard)
2. launch: ros2 launch mir100_navigation slam_navigation.launch.py \
     world:=<pkg>/worlds/hard_world.sdf x:=-4.31 y:=-2.81 z:=0.15 gui:=false launch_rviz:=false
3. copy the launch's merged-params tempfile (/tmp/tmp*.yaml, newest) into the run dir
4. start recorder: run_recorder5.py --world hard_world --model mir100 \
     --out <run>/recorder.jsonl --bag-dir <run>/bag \
     --sha-file <world.sdf> --sha-file nav_driver5.py --sha-file <params.yaml> --sha-file route_waypoints.json
5. start driver: nav_driver5.py --waypoints route_waypoints.json --log <run>/driver.jsonl      --leg-timeout 120 --global-timeout 900 --expect-start-index 1
6. on driver exit: stop stack, run check_clearance.py (world, recorder.jsonl, driver.jsonl,
   footprint polygon, goal 4.31 0.56) -> summary.json; archive everything
