Guide to connecting to kiwi car:

Connect to the kiwi network
ssh -p 2200 pi@192.168.8.1
password: raspberry

Login:: Gitlab - A microservice - Packages & Registries - 
- Container Registry - CLI Commands (blue button on the top right) - Copy the login command into the kiwi console

Then go to group15 - Packages & Registries - 
- Container Registry - Click on the image you wish to transfer and copy its path (it should end with :v0.0 (or some other version number).
docker pull "image name"

Then: docker-compose up

--------------------------------

Commands for manually running the microservices:

docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY registry.git.chalmers.se/courses/tme290/2022/group15/cone-detection:v0.0 --cid=111 --name=img.argb --width=1280 --height=720 --verbose

docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY registry.git.chalmers.se/courses/tme290/2022/group15/logic:v0.0 --cid=140 --freq=50 --xSpeed=0.13 --nPoints=2 --turnSpeed=0.05 --name=img.argb
