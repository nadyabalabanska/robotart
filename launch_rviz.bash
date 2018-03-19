#!/bin/bash

xhost +

DOCKER_COMMAND=docker
if hash nvidia-docker 2>/dev/null; then
    DOCKER_COMMAND=nvidia-docker
fi

$DOCKER_COMMAND exec -it roboteam /ws/rviz.bash

