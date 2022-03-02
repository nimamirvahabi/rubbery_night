import cv2
import numpy as np
import json

source = cv2.imread("C:/Users/Lenovo/Documents/Image_Processing/codes/OriImage.png")
print(source.shape)

image_json = open('json_file.json','w')
image_json.write(json.dumps(source.tolist(),indent=4))
image_json.close()
cv2.imshow("picture", source)
cv2.waitKey(0)