FROM tiryoh/ros2-desktop-vnc:dashing

LABEL maintainer="Masafumi Harada"

RUN apt-get update -y && \
    apt-get install -y zbar-tools python3 python3-pip git && \
    pip3 install opencv-python zbar && \
    mkdir -p ~/ros2senbay_ws/src && \
    cd ~/ros2senbay_ws/src && \
    git clone https://github.com/haradama/ros2senbay.git