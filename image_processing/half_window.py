import cv2
from cv2 import destroyAllWindows
import numpy as np

src = np.full((500,500,3),90,dtype = np.uint8)
for i in range(0,500):
    src[0:i,i,0] +=100
    src[0:i,i,1] +=50
    src[0:i,i,2] +=25
    
cv2.imshow("pic",src)
cv2.waitKey(0)
cv2.destroyAllWindows()