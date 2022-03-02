import cv2
import numpy as np

source = cv2.imread("C:/Users/Lenovo/Documents/Image_Processing/codes/OriImage.png")
source2 = cv2.imread("C:/Users/Lenovo/Documents/Image_Processing/codes/Word.png")
print(source.shape)
cv2.imshow("picture", source)
dst = cv2.addWeighted(source, 1, source2, 0.7, 1)
cv2.imshow("blended", dst)
cv2.waitKey(0)
