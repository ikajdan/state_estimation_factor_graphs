#!/bin/bash

if ! [ -d "src" ]; then
	echo "Error: directory `src` not found. Please run this script from the root of the f1tenth_docking repository."
	exit 1
fi

docker run \
  -it \
  --volume "${PWD}"/src:/root/ws/src \
  --name=f1tenth_docking_ros \
  ros_localization_demo \
  bash
