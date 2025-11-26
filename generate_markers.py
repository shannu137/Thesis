import numpy as np
import cv2
import sys

id = 1
# load the ArUCo dictionary
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

tag = cv2.aruco.generateImageMarker(arucoDict, id, 300)

# write the generated ArUCo tag to disk and then display it to our
# screen
cv2.imwrite("marker_" + str(id) + ".png", tag)
cv2.imshow("ArUCo Tag", tag)
cv2.waitKey(0)