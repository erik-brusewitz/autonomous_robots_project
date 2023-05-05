# cone-detection

Cone detection microservice




```docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY cone-detection --cid=111 --name=img.argb --width=1280 --height=720 --verbose --tune --hLowBlue=100 --sLowBlue=100 --vLowBlue=100 --hHighBlue=120 --sHighBlue=255 --vHighBlue=255 --hLowYellow=15 --sLowYellow=40 --vLowYellow=120 --hHighYellow=30 --sHighYellow=210 --vHighYellow=255 --contrastCoeff=1 --sellectPointingUpCones --maxConePixelSize=4000 --minConePixelSize=70 --maxHorizon=300 --AdaptiveHorizon --hLowHorizon=75 --sLowHorizon=75 --vLowHorizon=100 --hHighHorizon=100 --sHighHorizon=160 --vHighHorizon=200 ```
