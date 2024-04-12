""" This script converts the camera calibration values stored in a pickle file to a YAML file. """
import cv2
import os
import pickle

# Define the directory and file names
DIRECTORY = '/home/danoon/shared/aruco-marker-detection/aruco_marker_depthai/scripts/camera_calibration/calibration_values/'
PICKLE_FILE_NAME = 'camera_calibration_pickle_file.pkl'
CV_FILE_NAME = 'calibration_chessboard.yaml'

pickle_file_path = os.path.join(DIRECTORY, PICKLE_FILE_NAME)
cv_file_path = os.path.join(DIRECTORY, CV_FILE_NAME)

pickle_file = open(pickle_file_path, 'rb')
calibration_values = pickle.load(pickle_file)

mtx = calibration_values['mtx']
dist = calibration_values['dist']

# Save parameters to a file
cv_file = cv2.FileStorage(cv_file_path, cv2.FILE_STORAGE_WRITE)
cv_file.write('MTX', mtx)
cv_file.write('DIST', dist)
cv_file.release()