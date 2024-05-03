#!/bin/bash

if ! [ -d "src" ]; then
	echo "Error: directory `src` not found. Please run this script from the root of the f1tenth_docking repository."
	exit 1
fi

docker run \
  -it \
  --volume "${PWD}"/src:/root/ws/src \
  --name=ros_robot_demo \
  ros_robot_demo \
  bash
