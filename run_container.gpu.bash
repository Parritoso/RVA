
# Allow docker sockets and root user to access X server
xhost +local:docker
xhost +local:root

# Capture the current working directory
cwd=$(pwd)

docker run -it \
    --name rva_container_gpu \
    --gpus all \
    --runtime=nvidia \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    --privileged \
    --mount type=bind,source=$cwd/rva_ws,target=/home/rva_ws \
    rva_container_gpu \
    bash
    
docker rm rva_container_gpu
