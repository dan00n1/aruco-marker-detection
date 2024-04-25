# ArUco Marker Detection System

This repository hosts a sophisticated system for detecting and calculating ArUco markers in conjunction with a camera.

## Requirements
- Installation of ROS2 Humble (Refer to [ROS2 Humble documentation](https://docs.ros.org/en/humble/index.html))
- DepthAI OAK-d Camera (For more details, see [Luxonis OAK-d Camera](https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK/))

## Prerequisites
Before proceeding, ensure you have the following:
- ArUco markers (4x4 variant): Have at least four distinct ArUco markers of the 4x4 variant. You can either choose from the provided markers in the `Media/ArUco_markers` directory or generate your own using the [ArUco Generator](https://chev.me/arucogen/). 

*Note: It is recommended to use markers between the numbers 1 and 10, as those are the numbers used in this project. In some files, there is code that checks if the markers are in that range. These statements are recognized by their `#FIXME` comments.*

 ## Installation
Execute the following commands within an Ubuntu terminal. Ensure that **ROS2 Humble** is properly installed and that the camera is connected via a USB 3.0 port (identified by a blue-colored USB port).

<details>

<summary> Click here to expand the installation steps. </summary>

### 1. Navigate to the appropriate Workspace
Note: If you clone this repository directly, the folder `aruco_marker_depthai` will be located *inside* the `aruco-marker-detection` folder.
```bash
cd {Your-installation-path-here}/aruco_marker_depthai
# Alternatively:
# cd {Your-download-path}/aruco-marker-detection/aruco_marker_depthai
```

### 2. Source ROS2 Installations and Build the Project
```bash
# Source your ROS2 installation
source /opt/ros/humble/setup.bash
```

### 3. Build the project
```bash
# Build the project (twice if you encounter errors or warnings)
colcon build

# Source the install file of the project
source install/setup.bash
```

Rebuild the project to ensure a proper build and source the setup file again. *NOTE: Rebuild like this after every change you make.*
```bash
colcon build
source install/setup.bash
```
</details>

## Startup Procedures
There are multiple scripts inside this project, each of which is provided with a launch file.

<details>

<summary> Click here to expand the startup procedures steps. </summary>

### 1. Test Marker Orientation
Before implementing any code, it's crucial to ensure that the markers are correctly oriented. Incorrect orientations, such as upside-down markers, can hinder the program's ability to recognize them. To verify marker orientation, you can use the provided launch file listed down below.

```bash
ros2 launch aruco_marker_depthai test_marker_orientation.launch.py
```

<details>
<summary>Click to see the example images </summary>

A wrong orientation: 

![upperleft wrong](<Media/README_images/upperleft wrong.png>)

A correct orientation:

![upperleft correct](<Media/README_images/upperleft correct.png>)

</details>

### 2. Calibrate camera

To accurately determine the camera's position, it needs to be calibrated. While the OAK-d camera comes pre-calibrated, additional calibration values are required for the methods employed in this system. Detailed instructions for camera calibration can be found in the separate README file, accessible here: [camera calibration README](aruco_marker_depthai/scripts/camera_calibration/README.md)

### 3. Marker Detectors
Several scripts are available for detecting markers and their location and pose. You can explore the functionality of these scripts to verify if they accurately identify markers and their positions relative to the camera. The marker pose script provides information about the pose of individual markers, while the marker location script indicates the marker's position with respect to the camera.


```bash
# Launch each of them seperately
ros2 launch aruco_marker_depthai stereo_marker_detector.launch.py
ros2 launch aruco_marker_depthai stereo_marker_pose.launch.py
ros2 launch aruco_marker_depthai stereo_marker_location.launch.py
```

<details>
<summary>Click here to see the example images</summary>

Marker pose:

![marker_pose](<Media/README_images/marker_pose.png>)

Marker location:

![marker_location](<Media/README_images/marker_location.png>)
</details>

### 4. Camera localization
You can determine the camera's location based on one or multiple markers. To obtain the location based on a single marker, execute the following command:

```bash
ros2 launch aruco_marker_depthai camera_location_singular.launch.py
```

To calculate the location based on multiple markers, use the following command:
```bash
ros2 launch aruco_marker_depthai camera_location_multiple.launch.py
```
<details>
<summary>Click to see the example images</summary>

Camera location based on one marker:

![camera_location](<Media/README_images/camera_location.png>)

Camera location based on multiple markers:

![camera_location_multi](<Media/README_images/camera_location_multi.png>)
</details>

### 5. Optional: Testing with Multiple Cameras
When evaluating the camera location based on multiple markers, a script has been developed to configure the camera based on each individual marker and display all of those camera frames.

```bash
ros2 launch aruco_marker_depthai test_multiple_camera_locations_multiple_markers.launch.py
```

<details>
<summary>Click to see an example image</summary>

![all_cameras](<Media/README_images/all_cameras.png>)
</details>

</details>


## Possible errors
While building or running the project, there is a possibility that you may encounter errors. Some of these errors have already been addressed, so I provide the solutions for you here:

### Build error
When initially building your project, you may encounter a build failure. If this occurs, try building again. The second attempt is likely to succeed. Therefore, it is recommended to build the project twice.

### Different end-of-line characters
The following error may occur when trying to run the detector:
![Python error; no such file or directory](<Media/README_images/Python error no such file or dir.png>)

If that is the case, you need to ensure that the scripts, sources, and/or launch files are in the correct line-ending character format. Use the tool `dos2unix` to convert the files to the Unix-style line endings that Ubuntu requires. ([Source of solution](https://askubuntu.com/questions/896860/usr-bin-env-python3-r-no-such-file-or-directory))
```bash
# Install dos2unix:
sudo apt install dos2unix

# Convert the files to Unix-style line endings:
dos2unix {file-path}/{file-name}
# output: dos2unix: converting file {file-name} to Unix format ...

## For example:
## dos2unix scripts/marker_detector.py
```
Return to step 2 to rebuild the project again.

## License
[MIT License](https://github.com/dan00n1/aruco-marker-detection/blob/0d4f772d2e9c2635cd3b6ded379c2c12e2cb8b23/LICENSE)
