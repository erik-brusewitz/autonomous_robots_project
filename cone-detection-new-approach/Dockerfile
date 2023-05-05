FROM ubuntu:18.04 as builder
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update
RUN apt-get install -y \
    build-essential \
    cmake \
    software-properties-common \
    libopencv-dev

RUN add-apt-repository -y ppa:chrberger/libcluon
RUN apt-get update
RUN apt-get install -y libcluon

ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp/dest .. && \
    make && make install


FROM ubuntu:18.04
ENV DEBIAN_FRONTEND=noninteractive 

RUN apt-get update
RUN apt-get install -y \
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

WORKDIR /usr/bin
COPY --from=builder /tmp/dest /usr
ENTRYPOINT ["/usr/bin/cone-detection"]

