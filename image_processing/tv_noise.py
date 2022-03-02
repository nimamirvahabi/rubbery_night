
import cv2
import numpy as np

source = np.random.randint(0, 255, (500, 500, 3), dtype=np.uint8)
print(source.shape)

cv2.imshow("picture", source)
cv2.waitKey(0)
cv2.imwrite("C:/Users/Lenovo/Documents/Image_Processing/codes/tv_image.png", source)