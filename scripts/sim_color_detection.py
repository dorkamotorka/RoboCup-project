#!/usr/bin/env python2

from naoqi import ALProxy
import numpy as np
import matplotlib.pyplot as plot
import cv2

vision = ALProxy('RobocupVision', 'localhost', 9559)
cameraId = 0 # 0 for
data = vision.getBGR24Image(cameraId)
image = np.fromstring(data, dtype=np.uint8).reshape((480, 640, 3))
rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
plot.imshow(rgb_img)
