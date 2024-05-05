# ROS2 Localization Demo

This repository contains a ROS2 localization demo. The demo consists of a simple robot that moves in a 2D space and a particle filter that estimates the robot's pose. The robot's pose is published by the robot and the particle filter subscribes to it. The particle filter uses the robot's pose and the robot's movement to estimate the robot's pose.

## Setup the Environment

1. Clone the repository:
```bash
git clone --recursive git@github.com:ikajdan/ros_localization_demo.git
```

2. Build the container:
```bash
./build.sh
```

3. Run the container:
```bash
./run.sh
```

4. Build the workspace:
```bash
cd /root/ws
colcon build --symlink-install
```

## Run the Simulation

1. Source the workspace:
```bash
source ./install/setup.bash
```

2. Run the simulation:
```bash
ros2 launch robot sim.launch.py world:=./src/robot/worlds/main.world
ros2 run rviz2 rviz2 -d ./src/robot/config/view_main.rviz --ros-args -p use_sim_time:=true
```
> [!NOTE]
> If Gazebo fails to start, try to manually spawn the robot:
> ```bash
> ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity robot_name
> ```

3. Launch the localization node:
```bash
ros2 launch robot localization.launch.py map:=./src/robot/maps/main.yaml
```

4. Set the initial pose of the robot in RViz.
> [!NOTE]
> Make sure to set the fixed frame to `map`.

5. (Optional) Run the navigation node:
```bash
ros2 launch robot navigation.launch.py
```

## Control the Robot

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

## View Robot Model

```bash
ros2 launch robot rsp.launch.py
ros2 run joint_state_publisher_gui joint_state_publisher_gui
ros2 run rviz2 rviz2 -d ./src/robot/config/view_robot.rviz --ros-args -p use_sim_time:=true
```

## Launch the SLAM Toolbox

```bash
ros2 launch robot slam.launch.py
ros2 run rviz2 rviz2 -d ./src/robot/config/view_map.rviz --ros-args -p use_sim_time:=true
```

# License

This repository is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
