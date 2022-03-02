import cv2
import numpy as np

source = cv2.imread("C:/Users/Lenovo/Documents/Image_Processing/codes/OriImage.png",cv2.IMREAD_GRAYSCALE)
print(source.shape)

cv2.imshow("picture", source)
cv2.waitKey(0)