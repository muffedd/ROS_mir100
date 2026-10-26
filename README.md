# MiR100 navigation in Gazebo Harmonic

A learning project using ROS 2 Jazzy and the detailed MiR100 model. Click a goal in RViz, plan a grid route with **A***, and follow it using **regulated pure pursuit**. Lidar costmaps and collision checking stop the robot before obstacles; a blocked route is replanned.

## Install and build

Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-robot-localization

git clone https://github.com/muffedd/ROS_mir100.git
cd ROS_mir100
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src --packages-select mir_description mir100_navigation
source install/setup.bash
```

## Click a destination

Run from the `ROS_mir100` folder:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
GZ_IP=127.0.0.1 ros2 launch mir100_navigation slam_navigation.launch.py
```

1. Wait for the map and active navigation in RViz.
2. Select **Nav2 Goal** from the toolbar.
3. Click free space in the room and drag to select the final heading.
4. Release to send the goal. Click another goal to replace it.

The launch creates a live SLAM map. No fixed goal is sent automatically. The bounded room contains three obstacles. A* chooses a route around mapped obstacles. Pure pursuit turns toward the route and slows for bends. The robot keeps a route until a new goal arrives or the controller reports a blocked path, then replans.

Stop the launch with **Ctrl+C** before starting another instance.

## Save a map

In another terminal, from the workspace root:

```bash
src/mir100_navigation/scripts/save_map.sh
```

This writes `src/mir100_navigation/maps/mir100_world.yaml` and `.pgm`.
Set `ROS_SETUP=/path/to/setup.bash` when using a custom ROS installation.

## Source layout

- `src/mir_description`: robot meshes, Gazebo model, world, and launch files.
- `src/mir100_navigation`: Nav2/SLAM settings, navigation launch, and map saver.

The front lidar bridge has its own frame override. Odometry retains `frame_id=odom`, giving the TF chain `map -> odom -> base_footprint -> front_laser_link`.

## Attribution

MiR100 meshes and original description assets are from DFKI's `mir_robot` project, copyright 2018 DFKI GmbH, under the BSD license retained in `src/mir_description/LICENSE`. The repository's existing root license is retained.

## Verified run

The 6 m goal test completed with Nav2 status **SUCCEEDED** after routing around the box. See [test notes](docs/TESTING.md) and [recorded progress](docs/navigation-test.json).
