import numpy as np
import cv2

from detectSign import analyzePicture

img = cv2.imread('./60_real.png')
analyzePicture(img)
