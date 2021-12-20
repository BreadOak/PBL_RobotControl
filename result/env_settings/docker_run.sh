#!/bin/bash
xhost +local:root

docker run --rm -it -d \
    --name gpu_ros2 \
    --privileged \
    --volume=/dev:/dev \
    --net=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v $HOME/docker_share:/home/junyoung/colcon_ws/src \
    lgkimjy/ubuntu:ros2_dashing
