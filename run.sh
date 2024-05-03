#!/bin/bash

if ! [ -d "src" ]; then
	echo "Error: directory `src` not found. Please run this script from the root of the f1tenth_docking repository."
	exit 1
fi

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch "${XAUTH}"
xauth nlist "${DISPLAY}" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge -
chmod 644 "${XAUTH}"

docker run \
  -it \
  --env=DISPLAY="${DISPLAY}"  \
  --env=QT_X11_NO_MITSHM=1 \
  --env=XDG_RUNTIME_DIR=/tmp \
  --env=WAYLAND_DISPLAY="${WAYLAND_DISPLAY}" \
  --volume="${XDG_RUNTIME_DIR}"/"${WAYLAND_DISPLAY}":/tmp/"${WAYLAND_DISPLAY}" \
  --env=XAUTHORITY="${XAUTH}" \
  --volume="${XAUTH}":"${XAUTH}" \
  --volume="${XSOCK}":"${XSOCK}" \
  --volume ./src:/root/ws/src \
  --name=ros_robot_demo \
  ros_robot_demo \
  bash
