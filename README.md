# ROS2 Simulation of a Differential Robot

This repository contains a ROS2 localization project. The demo consists of a simple robot that moves in a 2D space equipped with sensors for odometry and localization. The robot is controlled using a differential drive system. The simulation is done in Gazebo. For portability, the project is containerized using Docker.

## Project Structure

The project is divided into two packages:

- `robot_simulation`: contains the Gazebo simulation of the robot, the AMCL localization node, and the navigation node.
- `state_estimation`: contains the GTSAM state estimation node.

### Robot Simulation Package

The `robot_simulation` package contains the Gazebo simulation of the robot, the AMCL localization node, and the navigation node. The robot is equipped with a lidar, IMU, and wheel encoders. The package also contains launch files for running the simulation and the localization node.

The description of the robot is defined in the `description` directory. The `launch` directory contains the launch files for the simulation and the localization node. The `maps` directory contains the map of the environment. The `worlds` directory contains the Gazebo world files. Configuration files are stored in the `config` directory.

An EKF is used to fuse the odometry and IMU data to estimate the robot's pose. The `robot_localization` package is used to perform the sensor fusion. The `amcl` package is used for localization.

### State Estimation Package

The `state_estimation` package contains a custom node that uses the GTSAM library to perform robot position estimation. The node subscribes to pose and odometry topics, processes the data using GTSAM's factor graphs and optimization algorithms, and then publishes an optimized pose estimate.

The class `RobotPositionEstimator` is a subclass of `rclcpp::Node` designed to handle state estimation tasks in a ROS 2 environment. It subscribes to the Adaptive Monte Carlo Localization (AMCL) pose topic to get the pose with covariance and to a filtered odometry topic. The node publishes the optimized pose to the `/gtsam_pose` topic.

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
    colcon build --symlink-install
    ```

## Run the Simulation

1. Source the workspace:

    ```bash
    source ./install/setup.bash
    ```

2. Launch the simulation:

    ```bash
    ros2 launch robot_simulation sim.launch.py world:=./src/robot_simulation/worlds/main.world gui:=false
    ```

> [!NOTE]
> The initial startup of the simulation may take a while.

> [!TIP]
> To visualize the Gazebo simulation, omit the `gui:=false` argument.

3. Launch the AMCL localization node:

    ```bash
    ros2 launch robot_simulation localization.launch.py map:=./src/robot_simulation/maps/main.yaml
    ```

4. Launch the GTSAM state estimation node:
    ```bash
    ros2 run state_estimation state_estimation
    ```

5. (Optional) Run the navigation node:

    ```bash
    ros2 launch robot_simulation navigation.launch.py
    ```

6. Launch the RViz visualization tool:

    ```bash
    ros2 run rviz2 rviz2 -d ./src/robot_simulation/config/view_main.rviz --ros-args -p use_sim_time:=true
    ```

7. Set the initial pose of the robot in RViz.

> [!NOTE]
> Make sure to set the fixed frame to `map`.

## Tools

To manually control the robot, you can use the `teleop_twist_keyboard` package:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

To create a map of the environment, use the `slam_toolbox` package:

```bash
ros2 launch robot_simulation slam.launch.py
ros2 run rviz2 rviz2 -d ./src/robot_simulation/config/view_map.rviz --ros-args -p use_sim_time:=true
```

This will launch the `slam_toolbox` node and RViz. Use the `teleop_twist_keyboard` package to move the robot around the environment and create a map.

## Acknowledgements

The wast majority of the robot simulation code was based on the [Building a Mobile Robot](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT) tutorial made by [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics).

The code for the GTSAM state estimation node was based on the following tutorial: [Factor Graphs and GTSAM](https://gtsam.org/tutorials/intro.html).

Other resources used in the development of this project include:
- https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/
- https://classic.gazebosim.org/tutorials?tut=ros_gzplugins
- https://navigation.ros.org/setup_guides/sensors/setup_sensors.html


## License

This repository is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
