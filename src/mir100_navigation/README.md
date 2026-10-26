# MiR100 Nav2 + SLAM

This package runs the detailed MiR100 in Gazebo Harmonic, builds a live 2D map
from the front lidar, and uses Nav2 to drive to a goal while mapping.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ROS_mir100
colcon build --symlink-install --base-paths src --packages-select mir_description mir100_navigation
source install/setup.bash
```

The ROS setup must include Gazebo Harmonic and `ros_gz_sim` / `ros_gz_bridge`.
Nav2, SLAM Toolbox, and robot_localization must also be installed for ROS Jazzy.

## Run mapping and autonomous navigation

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mir100_navigation slam_navigation.launch.py
```

Gazebo starts with the bounded obstacle world, SLAM Toolbox publishes `/map`,
and RViz shows the live map. Use RViz's **Nav2 Goal** tool to click a reachable
destination; there is no automatic fixed goal. NavFn uses A* for the global
grid route. Regulated pure pursuit follows that route, turns toward the path,
slows for bends, and checks the live lidar costmap for collisions.

Wait until navigation is active and the map appears. Select **Nav2 Goal** in
the RViz toolbar, click a free destination on the map, and drag to choose the
final heading. Release to send the goal. A new goal replaces the current one.

To send a goal automatically for a quick demo, enable the optional sender:

```bash
ros2 launch mir100_navigation slam_navigation.launch.py send_goal:=true goal_x:=5.0 goal_y:=2.5 goal_yaw:=1.57
```

SLAM continues updating the map while the robot moves. NavFn can route through
unmapped cells; the live lidar costmap checks the area ahead. Nav2 keeps a
valid route and replans when it is blocked or the goal changes. For the most predictable route,
click inside the already mapped area.

## Save the map

After the robot has scanned the area you want to keep, run:

```bash
src/mir100_navigation/scripts/save_map.sh
```

The map saver writes `mir100_world.yaml` and `mir100_world.pgm` into this
package's `maps/` directory. The online SLAM map is created at runtime; those
files appear after saving.

## Frames and topics

- Gazebo bridge: `/clock`, `/scan`, `/back_scan`, `/odom`, `/cmd_vel`
- EKF: publishes `odom -> base_footprint` using Gazebo odometry
- SLAM Toolbox: publishes `/map` and `map -> odom`
- Nav2: uses `base_footprint`, a 0.53 m conservative radius, and `/scan`
