
import cv2
from cv2 import resize

#read image
img_grey = cv2.imread('C:/Users/Lenovo/Documents/Image_Processing/codes/book2.jpg', cv2.IMREAD_GRAYSCALE)
img_grey=resize(img_grey,(int(img_grey.shape[1]/2),int(img_grey.shape[0]/2)))
cv2.imshow("gray",img_grey)
img_binary = cv2.adaptiveThreshold(img_grey,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,1201,0)

cv2.imshow("tresh",img_binary)
cv2.waitKey(0)