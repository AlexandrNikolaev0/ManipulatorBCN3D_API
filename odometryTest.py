import inputs
import sys
import cv2
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import math # Math library
from math import sin,cos,radians
from numpy import dot
import arucoOdometry
import time
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import pandas as pd
from scipy.signal import savgol_filter
import csv
aruco_marker_side_length = 0.0344 # the standart marker size, that we use
aruco_dictionary_name = "DICT_4X4_50" # the simplest aruco dict, that we use. The full list of dictionaries you can find in the class arucoOdometry beginning (file arucoOdometry.py)
# Calibration parameters yaml file
camera_calibration_parameters_filename = 'calibration_chessboardDEXP720.yaml' # here you have to put the filename, that we got as a result of the calibration (it corresponds to cv_file in cameraCalibrate.py file)


# Start the video stream
cap = cv2.VideoCapture(0)  # you have to put the number of your camera instead of the id. Usually, it is 0, 1, 2... Depending on the count of connected to the PC cameras.
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


odom = arucoOdometry.arucoOdometry()
odom.setCameraParams(camera_calibration_parameters_filename)
odom.setArucoLength(aruco_marker_side_length)
odom.setArucoDict(aruco_dictionary_name)

#Defining the markers list, with those markers, which we want to detect
markers = [
{"id":0,"size":aruco_marker_side_length},
{"id":8,"size":aruco_marker_side_length},
{"id":10,"size":0.035}
]

odom.setMarkers(markers)

startTime=time.time() * 1000
time.sleep(1)
while(True):
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = cap.read()
    ret, frame = cap.read()
    frame,x,y,z,a_x,a_y,a_z = odom.updateCameraPoses(frame,time.time()*1000-startTime, 8) # detecting a marker with id: 8
    cv2.imshow("im",frame)
    cv2.waitKey(1)