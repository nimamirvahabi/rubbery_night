import cv2
import numpy as np

source = cv2.imread("C:/Users/Lenovo/Documents/Image_Processing/codes/OriImage.png")
print(source.shape)
new_width = 300

# dsize
dsize = (new_width, source.shape[0])
cv2.imshow("picture", source)
output = cv2.resize(source, dsize, interpolation = cv2.INTER_AREA)
cv2.imshow("e", output)
cv2.waitKey(0)
