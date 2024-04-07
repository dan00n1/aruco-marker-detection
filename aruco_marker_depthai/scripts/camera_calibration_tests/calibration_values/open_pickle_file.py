import pickle
directory = '/home/danoon/shared/aruco-marker-detection/aruco_marker_depthai/scripts/camera_calibration_tests/calibration_values/camera_calibration_pickle_file.pkl'
pickle_file = open(directory, 'rb')
calibration_values = pickle.load(pickle_file)
print(calibration_values)