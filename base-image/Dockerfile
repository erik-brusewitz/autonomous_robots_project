FROM ubuntu:20.04 as builder
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update
RUN apt-get -y upgrade

RUN apt-get update && apt-get install -y build-essential software-properties-common cmake g++ wget unzip git

RUN add-apt-repository -y ppa:chrberger/libcluon
RUN apt-get install -y libcluon

RUN mkdir /opencv
WORKDIR /opencv
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
RUN wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
RUN unzip opencv.zip
RUN unzip opencv_contrib.zip
RUN rm opencv.zip opencv_contrib.zip
RUN mkdir -p build && \
    cd build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x && \
    cmake --build .

RUN /bin/bash -c 'echo "/opencv/build/lib/" >> /etc/ld.so.conf.d/opencv.conf' && ldconfig -v
