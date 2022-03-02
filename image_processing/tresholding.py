
import cv2

#read image
img_grey = cv2.imread('C:/Users/Lenovo/Documents/Image_Processing/codes/alarm.png', cv2.IMREAD_GRAYSCALE)
cv2.imshow("gray",img_grey)
img_binary = cv2.threshold(img_grey, 200, 255, cv2.THRESH_BINARY)[1]

cv2.imshow("tresh",img_binary)
cv2.waitKey(0)