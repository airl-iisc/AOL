#!/usr/bin/env python3

import cv2
import numpy as np


def skeletonize(img):
    assert len(img.shape) == 2  #make sure its single channel
    size = np.size(img)
    tenth_size = size/10
    skel = np.zeros(img.shape,np.uint8)

    ret,img = cv2.threshold(img,127,255,0)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    done = 0

    while(done < 20):
        eroded = cv2.erode(img,element)
        temp = cv2.dilate(eroded,element)
        temp = cv2.subtract(img,temp)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
        zeros = size - cv2.countNonZero(img)
        done = done + 1
        if zeros==size:
            done = 5
    return skel

img = cv2.imread('/home/user/volta_ws/mymap.pgm',2)

print(img.shape)
# Naming a window
cv2.namedWindow("Resized_Window", cv2.WINDOW_NORMAL)
  
# Using resizeWindow()
cv2.resizeWindow("Resized_Window", 800, 800)
# Select ROI
# r = cv2.selectROI("Resized_Window", img)
r = [1712, 1862, 2287-1712, 2237-1862]
# Crop image
cropped_image = img[int(r[1]):int(r[1]+r[3]), 
                      int(r[0]):int(r[0]+r[2])]
print(int(r[1]),int(r[1]+r[3]), int(r[0]),int(r[0]+r[2]))

# detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
contours, hierarchy = cv2.findContours(image=cropped_image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
                                     
# draw contours on the original image
image_copy = cropped_image.copy()
fin = cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 0, 0), thickness=10, lineType=cv2.LINE_AA)


cv2.imshow("Resized_Window",fin)
cv2.waitKey(0)
skel = skeletonize(fin)
cv2.imshow("new",skel)

cv2.waitKey(0)
cv2.destroyAllWindows()