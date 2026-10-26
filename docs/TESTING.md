# Navigation test — 2026-10-26

Gazebo Harmonic + ROS 2 Jazzy completed a `NavigateToPose` goal from the origin to `(6, 0)`. This is the action used by RViz's **Nav2 Goal** tool.

- Action result: **SUCCEEDED** (status 4), error code 0.
- Final feedback position: `(5.756, 0.038)` m; error 0.247 m, inside the configured 0.25 m tolerance.
- Route bent about 1.2 m sideways around the first box.
- Two automatic recoveries/replans occurred before completing the route.
- Test duration: about 121 seconds including startup waiting on this machine.
- The map was saved after the run. It includes both the origin and goal.

The raw progress samples are in `navigation-test.json`.

The chassis collision box now has 0.075 m ground clearance. Previously it touched the floor, allowing wheel odometry to report motion while the body dragged. The lidar uses a separate bridge override so odometry keeps its `odom` frame.

This test exercises the simulated static obstacle course. It does not measure performance with moving people or a physical robot.

## Additional check

A second goal at `(6, 2)` was interrupted by the lifecycle manager after a controller heartbeat timeout while RViz was running. The first obstacle-route result remains successful; sustained GUI load and repeated trips need further testing.

## Fix verification with RViz open

The kernel recorded suspend at 06:57:04 and resume at 06:59:38, matching the heartbeat failure timestamp exactly. The new startup wrapper holds a `sleep:idle` inhibitor only for the running simulation and restricts ROS discovery to localhost. This prevents the observed sleep/network disruption without weakening Nav2's heartbeat watchdog.

`scripts/test_navigation.py` subsequently sent `(6, 0)` then `(6, 2)` in one session with the smaller RViz view open. Both finished with status 4 (**SUCCEEDED**). The first took about 105 seconds with two recoveries; the second took about 75 seconds with zero recoveries. Frames were `odom` and `front_laser_link`, as expected. The detailed report is `repeated-goals-test.json`.

The map, robot, laser points, and route were visually confirmed in RViz. The initial GLSL sampler warning still appears once but did not prevent rendering. The panel's inactive AMCL localization indicator is expected because SLAM supplies the map transform.

A further goal was sent through the actual RViz Nav2 Goal toolbar and a map mouse drag. The server logged a destination near `(6.15, 1.0)` and reported **SUCCEEDED** with zero recoveries. See `rviz-mouse-goal-test.json` and `rviz-navigation.png`.
