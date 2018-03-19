#!/bin/bash

set -e

DIR=$(readlink -f `dirname $0`)
IMG_NAME="roboteam"
CONTAINER_NAME="roboteam"

sim=0

if [ -z "$1" ]; then
    sim=0
else
    if [ "$1" == "sim" ]; then
	sim=1
    else
	echo "$1 is not a recognized command"
    fi
fi

# Check if container exists
if [ ! `docker ps -q -f name=$CONTAINER_NAME` ]; then
    if [ `docker ps -a -q -f status=exited -f name=$CONTAINER_NAME` ]; then
        echo "container exited; removing"
        docker rm $CONTAINER_NAME
    fi

    docker build -t $IMG_NAME:latest $DIR
    
    # UDP ports 42120-42122 for Harris RedHawk arm
    # 
    # docker run -p 42120:42120/udp -p 42121:42121/udp -p 42122:42122/udp -dit --name $IMG_NAME $CONTAINER_NAME
    
    DOCKER_COMMAND=docker
    if hash nvidia-docker 2>/dev/null; then
        DOCKER_COMMAND=nvidia-docker 
    fi

    $DOCKER_COMMAND run \
		    -e "REDHAWK_SIMULATION=$sim" \
		    -e "DISPLAY=$DISPLAY" \
		    -e "QT_X11_NO_MITSHM=1" \
		    -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		    --device=/dev/dri:/dev/dri \
		    --privileged \
		    --net=host \
		    -dit \
		    --name $IMG_NAME $CONTAINER_NAME
fi

echo
echo "-------------------------------------------"
echo
echo "To attach to the running container, run:"
echo docker exec -it $CONTAINER_NAME /bin/bash
echo
echo "-------------------------------------------"
echo
echo "Ctrl-C to shutdown"
echo

# Wait for Ctrl-C
ctrl_c() {
    echo
    echo "Shutting down..."
    docker kill $CONTAINER_NAME >/dev/null
    docker rm $CONTAINER_NAME >/dev/null

    echo "Done."
    exit 0
}
trap ctrl_c INT
sleep infinity
