
import cv2
import numpy as np

source = np.full((500, 500, 3), 255, dtype=np.uint8)
print(source.shape)
position = (100,250)

cv2.putText(
     source, #numpy array on which text is written
     "CV2", #text
     position, #position at which writing has to start
     cv2.FONT_HERSHEY_SCRIPT_COMPLEX, #font family
     5, #font size
     (0, 0, 0), #font color
     5) #font stroke
cv2.putText(
     source, #numpy array on which text is written
     "Writed by nima", #text
     (120,300), #position at which writing has to start
     cv2.FONT_HERSHEY_SCRIPT_COMPLEX, #font family
     1, #font size
     (0, 0, 0), #font color
     1) #font stroke
cv2.imshow("picture", source)
cv2.waitKey(0)
cv2.imwrite("C:/Users/Lenovo/Documents/Image_Processing/codes/Write_on_image.png", source)
