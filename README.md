# ROS2 Localization Demo

This repository contains a ROS2 localization demo. The demo consists of a simple robot that moves in a 2D space and a particle filter that estimates the robot's pose. The robot's pose is published by the robot and the particle filter subscribes to it. The particle filter uses the robot's pose and the robot's movement to estimate the robot's pose.

## Setup the Environment

1. Clone the repository:
```
git clone --recursive git@github.com:ikajdan/ros_localization_demo.git
```

2. Build the container:
```
./build.sh
```

3. Run the container:
```
./run.sh
```

4. Build the workspace:
```
cd /root/ws
colcon build --symlink-install
```

# License

This repository is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
