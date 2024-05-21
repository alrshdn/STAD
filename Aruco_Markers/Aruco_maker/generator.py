import cv2 as cv
from cv2 import aruco


marker_size = 400 # pixels

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  #choose dictonary

for id in range(5):
    marker_image = aruco.generateImageMarker(marker_dict, id,marker_size)
    cv.imwrite(f"markers/markder_{id}.png", marker_image)
