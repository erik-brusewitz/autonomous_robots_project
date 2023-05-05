import os
import shutil
import zipfile
import cv2
import numpy as np

if os.path.exists('dataset'):
    shutil.rmtree('dataset')
if os.path.exists('kiwi_detection'):
    shutil.rmtree('kiwi_detection')

GRAY_SCALE = False

ZIP_DATASET_DIR = 'kiwi_detection.zip'
XML_TEMPLATE_PATH = 'xml_annotation-template.xml'
ANNOTATION_FILE_PATH = 'kiwi_detection/pos_all.txt'

with zipfile.ZipFile(ZIP_DATASET_DIR, 'r') as zip_ref:
    zip_ref.extractall('.')

with open(XML_TEMPLATE_PATH) as f:
    xml_template = f.read()

with open('kiwi_detection/pos_all.txt') as f:
    annotation_lines = f.readlines()

os.mkdir('dataset')

for annotation_line in annotation_lines:
    path, _, xmin, ymin, width, height = annotation_line.split(' ')
    directory, img_name = path.split('/')
    xmin = int(xmin)
    ymin = int(ymin)
    width = int(width)
    height = int(height)
    xmax = xmin + width
    ymax = ymin + height
    string = xml_template
    string = string.replace('XMIN', str(xmin))
    string = string.replace('XMAX', str(xmax))
    string = string.replace('YMIN', str(ymin))
    string = string.replace('YMAX', str(ymax))
    string = string.replace('IMG_NAME', str(img_name))
    
    with open('dataset/'+img_name.split('.')[0]+'.xml', "a") as f:
        f.write(string)

    img = cv2.imread('kiwi_detection/'+path)
    if GRAY_SCALE: 
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite('dataset/'+img_name, img)

shutil.rmtree('kiwi_detection')
