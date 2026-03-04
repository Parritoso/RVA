#!/bin/bash

# Allow docker sockets and root user to access X server
xhost +local:docker
xhost +local:root

# Capture the current working directory
cwd=$(pwd)

# --device=/dev/dri:/dev/dri \
# --group-add video \

docker run -it \
    --name rva_container_generic \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XDG_RUNTIME_DIR=/tmp" \
    --env="XAUTHORITY=/root/.Xauthority" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTHORITY:/root/.Xauthority:rw" \
    --net=host \
    --ipc=host \
    --privileged \
    --mount type=bind,source=$cwd/rva_ws,target=/home/rva_ws \
    rva_container_generic \
    bash
    
docker rm rva_container_generic

