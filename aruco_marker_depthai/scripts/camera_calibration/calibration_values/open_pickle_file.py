import os
import pickle

# Define the directory and file name
DIRECTORY = '/home/danoon/shared/aruco-marker-detection/aruco_marker_depthai/scripts/camera_calibration_tests/calibration_values/'
PICKLE_FILE_NAME = 'camera_calibration_pickle_file.pkl'

pickle_file_path = os.path.join(DIRECTORY, PICKLE_FILE_NAME)

pickle_file = open(pickle_file_path, 'rb')

calibration_values = pickle.load(pickle_file)

print(calibration_values)