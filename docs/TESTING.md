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
