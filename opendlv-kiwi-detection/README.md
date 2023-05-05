# opendlv-perception-kiwi-detection

Microservice for kiwi detection.

Steps to run:
 - 1- Use ```docker build --no-cache -f Dockerfile -t kiwi-detection .``` to build the docker. It is also possible to pull the image from the gitlab.
 - 2- Run the microservice using the following command:
 
    ``` docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY kiwi-detection --cid=111 --name=img.argb --freq=3 --width=1280 --height=720 --conf=0.8 --nms=0.6 --verbose --model=yolo_fastest --use_kalman```

    after modifying the arguments.
