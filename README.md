# F1Tenth remote teleops

This repo is with the assumption you have fully assembled your F1Tenth/RoboRacer vehicle and downloaded ROS2 Humble and the F1Tenth Stack(possible by git cloning this repo) and other related dependancies. If you haven't, follow their instructions to the best of your ability as some of the commands may be outdated: [RoboRacer Set-up instructions](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/intro.html).

## Cloning submodules
If you clone this repository, make sure to clone the submodules as well. You can do this by running:

```bash
git submodule update --init --recursive --remote
```

This will ensure you have all the submodules cloned and updated to the configured branches.

## Launch Instructions
These launch Instructions are under the assumption that your F1Tenth Racer and your cockpit are in the same network.
### Command and Control Downlink
#### Cockpit Side
The attached script in `teleop_cockpit` is with the assumption that you are using an Xbox Controller or a Logitech G923 Steering wheel. You can use a different control mechanism just make sure the axis are mapped correctly. You can find out what output from your controller/wheel maps to what axis using `teleop_cockpit/test_controller.py`

If you are on Windows or Mac you must natively run the cockpit code, not through docker. I recommend you create a virtual environment. You need a python 3.10-3.12 version. Simply run `python transmit_driving_inputs.py`

If you are using linux you can simply do `docker compose up` to launch and `docker compose down` to shut down. 



#### Jetson Side
In one terminal cd into your F1Tenth/Roboracer Workspace then run the following
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# If it's your first time launching or anything in the stack has changed, run “colcon build”
ros2 launch f1tenth_stack bringup_launch.py
```
In another terminal run the following.
```bash
zenoh-bridge-ros2dds client -e tcp/<COCKPIT_IP>:7447
```

### Camera Feed Uplink
First download MediaMTX by running the following in a Jetson terminal.
```bash
# 1. Download the updated release
wget https://github.com/bluenviron/mediamtx/releases/download/v1.15.3/mediamtx_v1.15.3_linux_arm64.tar.gz

# 2. Extract it
tar -zxvf mediamtx_v1.15.3_linux_arm64.tar.gz
```
In a Jetson terminal run
```bash
# 3. Run the server (Leave this terminal open)
./mediamtx
```
Next download gstreamer using these commands.
```bash
sudo apt-get update

sudo apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    gstreamer1.0-nice
```

In another Jetson terminal run 
```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! \
image/jpeg,width=1280,height=720,framerate=30/1 ! \
jpegdec ! \
videoconvert ! \
x264enc speed-preset=ultrafast tune=zerolatency bitrate=2000 ! \
rtspclientsink location=rtsp://localhost:8554/cam
```
The NVIDIA Jetson Orin Nano doesn't come with a H264 ENCODER so this gstreamer pipeline uses CPU H264 Encoding with ultrafast and zerolatency presets. You may have to tinker with the width, height, and framerate as each camera has different accepted formats. If it doesn't work, run the following to find out what those formats are.
```bash
#Install v4l Package
sudo apt-get install v4l-utils
#List Cameras
v4l2-ctl --list-devices
#Pick your device(/dev/video0 as an example)
v4l2-ctl -d dev/video0 --list-formats-ext
```