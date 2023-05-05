# Detecting kiwi cart

## Generate Dataset
To generate dataset in your local directory:

1. Copy all repository files 
2. Download ```kiwi_detection.zip``` from Canvas and put it in the same directory as other files 
3. Run python3 ```generate_dataset.py```


## Roboflow
The dataset is also availabe in Roboflow. Use the following code to download it in your notebook:

```
!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="vMvKMIy9EBzByJMywMqj")
project = rf.workspace("kiwidetection-kdou2").project("kiwi-detection-rgb-images")
dataset = project.version(1).download("yolov5")
```
