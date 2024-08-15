# import pyrealsense2.pyrealsense2 as rs
# pipe = rs.pipeline()
# profile = pipe.start()
# try:
#   for i in range(0, 100):
#     frames = pipe.wait_for_frames()
#     for f in frames:
#       print(f.profile)
# finally:
#     pipe.stop()

# rs-save-to-disk

import cv2
import numpy as np
import os

os.system('rs-save-to-disk')
from PIL import Image
im = Image.open('rs-save-to-disk-output-Color.png')
im.save('rs-save-to-disk-output-Color.jpg', quality=95)

 
 
img = cv2.imread("rs-save-to-disk-output-Color.jpg")
def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        print (xy)
        cv2.circle(img, (x, y), 1, (255, 0, 0), thickness = -1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (0,0,0), thickness = 1)
        cv2.imshow("image", img)
 
 
cv2.namedWindow("image")
cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
cv2.imshow("image", img)
 
 
while(True):
    try:
        cv2.waitKey(100)
    except Exception:
        cv2.destroyWindow("image")
        break
        
cv2.waitKey(0)
cv2.destroyAllWindow()
