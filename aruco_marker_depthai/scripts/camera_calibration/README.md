# Camera calibration
This guide outlines the steps to calculate the calibration values of a camera using OpenCV with an Oak-D camera.

## Prerequisites
Before you begin, make sure you have the following:
- **OAK-D Camera:** Refer to [Luxonis OAK-D Camera](https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK/) for details.
- **Python 3.7 or higher:** Python 3.10.12 was used in this guide.
- **OpenCV library installed:** For more details, see [PyPI](https://pypi.org/project/opencv-python/).
- **A sample chessboard image:** You can find one in the `Media` folder named `opencv_chessboard.png` or download it from the [OpenCV GitHub page](https://github.com/opencv/opencv/blob/master/doc/pattern.png).

## Steps
### 1. Capture images of a chessboard
Capture approximately 15 different pictures of a chessboard using your Oak-D camera.

The script `oakd_camera_capture_script.py` can be used to capture pictures with the Oak-D camera.

### 2. Test corner detection
Use `chessboard_corner_detection.py` to test one of your captured images.

It should display an image of the chessboard with corners drawn on it, similar to the images in the `images_oakd_camera/drawn_corners` folder.

### 3. Calculate camera calibration values
Execute `chessboard_camera_calibration.py` to calculate the calibration values of the camera.

The calibration values will be stored in a pickle file.

Provide an example image to undistort and verify the correctness of the calibration values. Refer to images in `images_oakd_camera/undistorted` for comparison.

### 4. Check pickle file contents
Use `open_pickle_file.py` to verify the contents of the saved pickle file.
The console output should resemble the content structure of `camera_calibration_example.txt`.

### Optional: Undistort images
If necessary, utilize `undistort_camera_images.py` to undistort additional images using the created pickle file.

### 5. Generate YAML File
Execute `pickle_to_yaml.py` to create a `calibration_chessboard.yaml` file. This YAML file is essential for later stages. Confirm successful loading of its contents using `open_yaml_file.py`.