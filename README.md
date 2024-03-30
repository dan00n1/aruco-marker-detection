# aruco-marker-detection
An repository for the detection and calculation regarding Aruco markers and it's camera

## Requires
- ROS2 Humble installation (See: [ROS2 Humble documentation](https://docs.ros.org/en/humble/index.html))
- DepthAI OAK-d Camera (See: [Luxonis OAK-d Camera](https://docs.luxonis.com/projects/hardware/en/latest/pages/BW1098OAK/))

## Start-up (Ubuntu terminal commands)
Navigate to the correct workspace. Note: if you copy this repository directly, the folder `aruco_marker_depthai` will be located *inside* the `aruco-marker-detection` folder
```bash
cd {Your-installation-path-here}/aruco_marker_depthai
# Alternatively: 
# cd {Your-download-path}/aruco-marker-detection/aruco_marker_depthai  
```

Source the ROS2 installations and build the project
```bash 
#Source your ROS2 installation
source /opt/ros/humble/setup.bash

# Build the project
colcon build

#Source the install file of the project
source install/setup.bash
```

Build the project again, just to be sure, and source the setup file again. Note: `--symlink-install` can be added to prevent the constant building of the project after minor changes inside python files.
```bash
colcon build --symlink-install
source install/setup.bash
```

Run the marker detector.
```bash
ros2 launch aruco_marker_depthai stereo_marker_detector.launch.py
```

## License
[MIT](https://github.com/dan00n1/aruco-marker-detection/blob/0d4f772d2e9c2635cd3b6ded379c2c12e2cb8b23/LICENSE)
