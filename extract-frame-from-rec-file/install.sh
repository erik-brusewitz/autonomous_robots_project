#!/bin/bash

apt-get update
apt-get install -y \
    build-essential \
    cmake \
    software-properties-common \
    libopencv-dev

add-apt-repository -y ppa:chrberger/libcluon
apt-get update
apt-get install -y libcluon

apt-get install -y \
    libopencv-core3.2 \
    libopencv-imgproc3.2 \
    libopencv-video3.2 \
    libopencv-calib3d3.2 \
    libopencv-features2d3.2 \
    libopencv-objdetect3.2 \
    libopencv-highgui3.2 \
    libopencv-videoio3.2 \
    libopencv-flann3.2 \
    python3-opencv

mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/dest ..
make
make install
mv extract-frame ../
cd ..
rm -r build
mkdir ~/extracted_kiwi_imgs
