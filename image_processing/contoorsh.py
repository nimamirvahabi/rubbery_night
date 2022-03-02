import cv2
import numpy as np

img = cv2.imread('C:/Users/Lenovo/Documents/Image_Processing/codes/wall.jpg', cv2.IMREAD_UNCHANGED)
cv2.imshow("contoors",img)
#convert img to grey
img_grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#set a thresh
thresh = 90
#get threshold image
_,thresh_img = cv2.threshold(img_grey, thresh, 255, cv2.THRESH_BINARY_INV)
#find contours
contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

# draw the contours on the empty image
cv2.drawContours(img, contours, 0, (255,255,0), 3)
cv2.imshow("contoors",img)
cv2.imshow("thresh",thresh_img)
cv2.waitKey(0)