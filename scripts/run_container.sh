#!/usr/bin/env bash

# create workspace on host, in home directory
mkdir $HOME/f1tenth_ws

# give docker permission to use X
sudo xhost +si:localuser:root

# run container with privilege mode, host network, display, and mount workspace on host
sudo docker run --runtime nvidia -it --rm --privileged --network host -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix -v /dev:/dev -v $HOME/f1tenth_ws:/f1tenth_ws f1tenth/focal-l4t-foxy:f1tenth-stack