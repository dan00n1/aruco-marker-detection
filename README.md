# ArUco Marker Detection System

This repository hosts a sophisticated system for detecting and calculating ArUco markers in conjunction with a camera.

## Requirements
- Installation of ROS2 Humble (Refer to [ROS2 Humble documentation](https://docs.ros.org/en/humble/index.html))
- DepthAI OAK-d Camera (For more details, see [Luxonis OAK-d Camera](https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK/))

## Installation and Startup Procedures
Execute the following commands within an Ubuntu terminal. Ensure that **ROS2 Humble** is properly installed and that the camera is connected via a USB 3.0 port (identified by a blue-colored USB port).

### 1. Navigate to the Appropriate Workspace
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

Rebuild the project to ensure accuracy and source the setup file again. *Note: `--symlink-install` can be appended to prevent frequent rebuilding of the project after minor changes in Python files.*
```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch the Marker Detector
```bash
ros2 launch aruco_marker_depthai marker_detector_own_pipeline.launch.py
```

### Possible errors
While building or running the project, there is a possibility that you may encounter errors. Some of these errors have already been addressed, so I provide the solutions for you here:
#### Different end-of-line characters
The following error may occur when trying to run the detector:
![Python error; no such file or directory](<Info images/Python error no such file or dir.png>)

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
