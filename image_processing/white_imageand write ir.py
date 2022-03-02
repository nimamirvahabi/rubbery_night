
import cv2
import numpy as np

source = np.full((500, 500, 3), 255, dtype=np.uint8)
print(source.shape)

cv2.imshow("picture", source)
cv2.waitKey(0)
cv2.imwrite("C:/Users/Lenovo/Documents/Image_Processing/codes/white_image.png", source)