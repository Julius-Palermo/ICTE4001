# GMapping SLAM with Nav2 on the TurtleBot4 (Simulation)

The `case\_study/` folder of this repository ([Julius-Palermo/ICTE4001](https://github.com/Julius-Palermo/ICTE4001)) runs **GMapping** (a Rao-Blackwellised particle filter SLAM) on a simulated **TurtleBot4** in **ROS 2 Humble**, and uses **Nav2** to drive the robot autonomously to goal poses while the map is being built.

* **GMapping** builds an occupancy grid map from the lidar and odometry, and publishes the robot's corrected pose (the `map → odom` transform).
* **Nav2** uses that map and pose to plan a path to a goal clicked in RViz and drive the robot there.

GMapping replaces the map server and AMCL that Nav2 normally uses, so no saved map or initial pose is needed.

\---

## 1\. Environment

|Item|Version used|
|-|-|
|OS|Ubuntu 22.04|
|ROS 2|Humble|
|Simulator|Ignition Gazebo (TurtleBot4 simulator, `warehouse` world)|
|Robot model|TurtleBot4 standard|
|SLAM package|`slam\_gmapping` (ROS 2 Humble port, `eloquent-devel` branch of [GMHadou/slam\_gmapping\_Humble](https://github.com/GMHadou/slam_gmapping_Humble)), with the modifications listed in Section 6|
|Navigation|Nav2 (`nav2\_bringup` `navigation\_launch.py`, default parameters)|

### 1.1 Install the required packages

These are already installed on the ICTE4001 lab machines. On a fresh Ubuntu 22.04 + ROS 2 Humble install:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator ros-humble-turtlebot4-desktop \\
                 ros-humble-navigation2 ros-humble-nav2-bringup \\
                 ros-humble-teleop-twist-keyboard
```

The TurtleBot4 simulator also needs Ignition Gazebo Fortress (`sudo apt install ignition-fortress`) if it is not already installed.

\---

## 2\. Build the workspace

```bash
# 1. Get the code (clones into \~/ICTE4001)
cd \~
git clone --depth 1 https://github.com/Julius-Palermo/ICTE4001.git

# 2. Go to the GMapping workspace and install any missing dependencies
cd \~/ICTE4001/case\_study/gmapping\_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build

# 4. Source the workspace (do this in EVERY new terminal that runs GMapping)
source install/local\_setup.bash
```

Build only inside `case\_study/gmapping\_ws`. The rest of the repository holds other coursework and is not needed. `--depth 1` downloads only the latest version, which keeps the download smaller. If a folder called `\~/ICTE4001` already exists, clone somewhere else and adjust the paths below to match.

The workspace contains two packages:

|Package|Contents|
|-|-|
|`openslam\_gmapping`|The original OpenSLAM GMapping library (particle filter, scan matcher, grid map)|
|`slam\_gmapping`|The ROS 2 node that connects GMapping to ROS topics and TF|

\---

## 3\. Run GMapping with Nav2 in the simulation

Use **four terminals**. In each one, first run `source /opt/ros/humble/setup.bash`.

### Terminal 1: Simulator

```bash
ros2 launch turtlebot4\_ignition\_bringup turtlebot4\_ignition.launch.py
```

When Gazebo opens, press the **Play** button (bottom left) if the simulation is paused. Do **not** add `slam:=true`, because that starts slam\_toolbox, which would conflict with GMapping (both publish `map → odom`).

### Terminal 2: GMapping

```bash
cd \~/ICTE4001/case\_study/gmapping\_ws
source install/local\_setup.bash
ros2 launch slam\_gmapping slam\_gmapping.launch.py use\_sim\_time:=true
```

Start this straight after the simulator. `use\_sim\_time:=true` makes GMapping use the simulator's clock.

### Terminal 3: RViZ

```bash
ros2 launch turtlebot4\_viz view\_robot.launch.py
```

This should pull up the RViZ window, displaying the robot.

### Terminal 4: Nav2

```bash
ros2 launch nav2\_bringup navigation\_launch.py use\_sim\_time:=true
```

This starts the Nav2 planner, controller and behaviour tree with Nav2's default parameters. `use\_sim\_time:=true` makes Nav2 use the simulator's clock, like GMapping. Once it is running, you can send goals from RViz (see below).

### Check that everything is running (optional)

```bash
ros2 node list | grep -E "slam\_gmapping|bt\_navigator"
```

Both `slam\_gmapping` and `bt\_navigator` should be listed. In RViz (Fixed Frame `map`) you should see the map growing and the robot model on it.

### Build some map, then send a navigation goal

1. **Drive the robot a short distance first** so the map has some free space. If the robot starts on its charging dock, undock it first (`ros2 action send\_goal /undock irobot\_create\_msgs/action/Undock {}`, or select Undock in the Gazebo TurtleBot4 HMI panel). Then drive using the **Teleop** panel in Gazebo, or in a new terminal:

```bash
   ros2 run teleop\_twist\_keyboard teleop\_twist\_keyboard
   ```

2. In RViz, click **Nav2 Goal** in the top toolbar. Click a free (white) spot on the map and drag to set the robot's final heading.
3. The robot plans a path and drives to the goal on its own. The Navigation 2 panel shows the distance remaining, and "Feedback: reached" when it arrives.
4. Repeat with new goals. The map keeps updating as the robot explores.

You do **not** need to set a 2D Pose Estimate: GMapping provides the robot's pose, so the Navigation 2 panel shows "Localization: inactive" (AMCL is not used).

### Save the map

In a new terminal whilst everything else is still running:

```bash
mkdir -p \~/maps
ros2 run nav2\_map\_server map\_saver\_cli -f \~/maps/gmapping\_map
```

This writes `gmapping\_map.pgm` (the image) and `gmapping\_map.yaml` (resolution and origin). An example map of the warehouse made this way is in `case\_study/warehouse\_map/`.

\---

## 4\. Topics and transforms

|Direction|Name|Type|Notes|
|-|-|-|-|
|GMapping in|`/scan`|`sensor\_msgs/LaserScan`|Lidar from the simulator|
|GMapping in|TF `odom → base\_link`|transform|Wheel odometry, read from TF at each scan's timestamp|
|GMapping in|TF `base\_link → lidar`|static transform|Lidar mounting position|
|GMapping out|`/map`|`nav\_msgs/OccupancyGrid`|The map (transient-local); used by Nav2's global costmap|
|GMapping out|TF `map → odom`|transform|Pose correction, published every 0.05 s; gives the robot's position on the map|
|GMapping out|`/map\_metadata`, `/entropy`|`MapMetaData`, `Float64`|Not used by Nav2|
|Nav2 out|`/cmd\_vel`|`geometry\_msgs/Twist`|Velocity commands to the robot|

\---

## 5\. Configuration

The paths in this section and Section 6 are inside `case\_study/gmapping\_ws/src/slam\_gmapping\_Humble-eloquent-devel/`.

|File|Purpose|
|-|-|
|`slam\_gmapping/launch/slam\_gmapping.launch.py`|Starts the `slam\_gmapping\_node`, loads the parameter file, and accepts the `use\_sim\_time` argument|
|`slam\_gmapping/config/slam\_gmapping.yaml`|All GMapping parameters|

Parameters used (values marked \* were changed from the package defaults):

|Parameter|Value|
|-|-|
|`particles`|30|
|`linearUpdate`|0.2 m\* (default 1.0 m)|
|`angularUpdate`|0.25 rad\* (default 0.5 rad)|
|`temporalUpdate`|−1 (disabled)|
|`map\_update\_interval`|5.0 s|
|`delta` (map resolution)|0.05 m|
|`xmin`, `ymin` / `xmax`, `ymax`|−100 m / 100 m (a 4000 × 4000 cell grid)|
|`maxUrange`, `maxRange`|automatic (from the lidar)|
|`srr`, `srt`, `str`, `stt`|0.1, 0.2, 0.1, 0.2|
|`sigma`, `kernelSize`|0.05, 1|
|`lstep`, `astep`, `iterations`|0.05 m, 0.05 rad, 5|
|`lsigma`, `ogain`, `lskip`|0.075, 3.0, 0|
|`minimumScore`|0|
|`resampleThreshold`|0.5|
|`occ\_thresh`|0.25|
|`transform\_publish\_period`|0.05 s|
|`base\_frame` / `odom\_frame` / `map\_frame`|`base\_link` / `odom` / `map`|

Nav2 runs with the default parameter file of `nav2\_bringup` (`nav2\_bringup/params/nav2\_params.yaml`), unmodified.

\---

## 6\. Changes made to the original package

Compared with the `eloquent-devel` branch of GMHadou/slam\_gmapping\_Humble:

1. **Scan QoS** (`slam\_gmapping/src/slam\_gmapping.cpp`): the `scan` subscription was changed from `rclcpp::SystemDefaultsQoS()` to `rclcpp::SensorDataQoS()` (best-effort), so it matches the profile that lidar topics normally publish with.
2. **Parameters** (`slam\_gmapping/config/slam\_gmapping.yaml`): `linearUpdate` 1.0 → 0.2 m and `angularUpdate` 0.5 → 0.25 rad, so that scans are processed more often.

\---

## 7\. Known limitations

* **No recovery from kidnapping.** If the robot is moved without its odometry registering the motion, GMapping cannot relocalise: its particles are predicted from odometry and only searched locally, and it has no global relocalisation. The robot stays lost and the map becomes corrupted.
* **Computational cost.** Every accepted scan is matched for all 30 particles, and each map rebuild replays the whole trajectory into a fixed 200 m × 200 m grid. Navigation with GMapping was visibly less smooth than with slam\_toolbox.
* **2D, static environments only.** Obstacles above or below the lidar plane are not mapped, and moving objects can leave artefacts.
* **Simulation only.** The evaluation was carried out in simulation, where a ground-truth pose is available.

\---

## 8\. Troubleshooting

|Problem|Cause and fix|
|-|-|
|`Package 'slam\_gmapping' not found`|The workspace is not sourced in that terminal. Run `source \~/ICTE4001/case\_study/gmapping\_ws/install/local\_setup.bash`.|
|RViz: "No transform from base\_link to map", or no map appears|GMapping is not running, or the simulation is paused. Press Play in Gazebo and check `ros2 node list` for `slam\_gmapping`. Restarting GMapping after the simulator is running also fixes a missing `map` frame. If the map still does not show, add a Map display on `/map` in RViz and set its Durability Policy to Transient Local.|
|Map jumps around or two maps fight|Another node is also publishing `map → odom` (slam\_toolbox, AMCL or map\_server). Close it and run only GMapping.|
|GMapping receives no scans|Check that `/scan` exists (`ros2 topic list`). The subscription must use `SensorDataQoS` (Section 6).|
|Timestamp / extrapolation warnings|Make sure GMapping and Nav2 were both started with `use\_sim\_time:=true`.|
|Nav2 goal rejected or the robot does not move|Wait until the Navigation 2 panel shows "Navigation: active", and place the goal in already-mapped free (white) space.|
|Global costmap is slow to appear|The default 200 m × 200 m map is large. Reduce `xmin`/`ymin`/`xmax`/`ymax` (e.g. to ±30 m) in `slam\_gmapping.yaml` and restart GMapping.|



