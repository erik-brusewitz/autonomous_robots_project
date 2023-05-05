# cone-detection

Cone detection microservice



``` docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY cone-detection --cid=111 --name=img.argb --width=1280 --height=720 --hLowBlue=110 --sLowBlue=90 --vLowBlue=40 --hHighBlue=140 --sHighBlue=255 --vHighBlue=255 --hLowYellow=15 --sLowYellow=100 --vLowYellow=100 --hHighYellow=30 --sHighYellow=255 --vHighYellow=255 --hLowRed=170 --sLowRed=120 --vLowRed=120 --hHighRed=180 --sHighRed=180 --vHighRed=160 --hLowHorizon=90 --sLowHorizon=60 --vLowHorizon=160 --hHighHorizon=80 --sHighHorizon=100 --vHighHorizon=180 --contrastCoeff=1 --morphologyExIter=2 --maxConePixelSize=4000 --minConePixelSize=10 --maxHorizon=350 --adaptiveHorizon  ```
