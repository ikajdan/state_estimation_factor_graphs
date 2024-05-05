# ROS2 Simulation of a Differential Robot

This repository contains a ROS2 localization project. The demo consists of a simple robot that moves in a 2D space and an AMCL localization node that estimates the robot's pose.

The simulation is done in Gazebo, and the localization node is implemented using the `nav2_amcl` package. For portability, the project is containerized using Docker.

## Set up the Environment

1. Clone the repository:

    ```bash
    git clone git@github.com:ikajdan/ros_differential_robot_simulation.git
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
    cd /root/ws && colcon build --symlink-install
    ```

## Run the Simulation

1. Source the workspace:

    ```bash
    source ./install/setup.bash
    ```

2. Launch the simulation:

    ```bash
    ros2 launch robot sim.launch.py world:=./src/robot/worlds/main.world gui:=false
    ```

> [!NOTE]
> The initial startup of the simulation may take a while.

> [!TIP]
> To visualize the Gazebo simulation, omit the `gui:=false` argument.

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

6. Launch the RViz visualization tool:

    ```bash
    ros2 run rviz2 rviz2 -d ./src/robot/config/view_main.rviz --ros-args -p use_sim_time:=true
    ```

## Tools

To control the robot, you can use the `teleop_twist_keyboard` package:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

To create a map of the environment, use the `slam_toolbox` package:

```bash
ros2 launch robot slam.launch.py
ros2 run rviz2 rviz2 -d ./src/robot/config/view_map.rviz --ros-args -p use_sim_time:=true
```

## Acknowledgements

The wast majority of the code is based on the [Building a Mobile Robot](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT) tutorial made by [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics).

## License

This repository is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
