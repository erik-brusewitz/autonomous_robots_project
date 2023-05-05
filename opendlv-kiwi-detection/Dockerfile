FROM registry.git.chalmers.se/courses/tme290/2022/group15/base-image:v0.0.2 as builder
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update
RUN apt install -y libcanberra-gtk-module libcanberra-gtk3-module

ADD . /opt/sources
WORKDIR /opt/sources

RUN ls /opt/sources/src

RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/dest .. && \
    make && make install

COPY ./src/model ./build
RUN ls ./build

ENTRYPOINT ["/opt/sources/build/opendlv-kiwi-detection"]

