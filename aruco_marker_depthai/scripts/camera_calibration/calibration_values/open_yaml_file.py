import cv2
import os

# Define the directory and file name
DIRECTORY = '/home/danoon/shared/aruco-marker-detection/aruco_marker_depthai/scripts/camera_calibration/calibration_values/'
CV_FILE_NAME = 'calibration_chessboard.yaml'

cv_file_path = os.path.join(DIRECTORY, CV_FILE_NAME)

# Load the parameters from the saved file
cv_file = cv2.FileStorage(cv_file_path, cv2.FILE_STORAGE_READ) 

mtx = cv_file.getNode('MTX').mat()
dist = cv_file.getNode('DIST').mat()

cv_file.release()

# Display key parameter outputs of the camera calibration process
print("Camera matrix:") 
print(mtx) 
print("\n Distortion coefficient:") 
print(dist) 