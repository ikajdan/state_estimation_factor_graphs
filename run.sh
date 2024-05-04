#!/bin/bash

NAME=ros_robot_demo
IMAGE=ros_robot_demo
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

if ! [ -d "src" ]; then
	echo 'Error: directory "src" not found. Please run this script from the root of the f1tenth_docking repository.'
	exit 1
fi

if ! [ -f "${XAUTH}" ]; then
  touch "${XAUTH}"
  xauth nlist "${DISPLAY}" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge -
  chmod 644 "${XAUTH}"
fi

if [ "$(docker ps -aq -f status=exited -f name="${NAME}")" ]; then
  # Start the existing container and attach to it
  docker start "${NAME}"
  exec docker exec -it "${NAME}" bash
elif [ "$(docker ps -aq -f status=running -f name="${NAME}")" ]; then
  # Attach to the running container
  exec docker exec -it "${NAME}" bash
else
  # Create a new container and attach to it
  exec docker run \
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
    --name="${NAME}" \
    "${IMAGE}" \
    bash
fi
