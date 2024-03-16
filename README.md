# aruco-marker-detection
An repo for the detection and calculation regarding Aruco markers

## Requires
- ROS2 Humble installation
- DepthAI OAK-d Camera

## Start-up (Ubuntu terminal commands)


Navigate to the correct workspace. (I had the workspace located at `Home/Depthai-tester-folder/aruco_marker_depthai`)  
```bash
cd {Your-installation-path-here}/aruco_marker_depthai
```

Source your ROS2 installation
```bash
source install/setup.bash
source /opt/ros/humble/setup.bash
```

Build the project. Maybe do it twice just to be sure. `--symlink-install` can be added to prevent the constant building of the project after minor changes inside python files.
```bash
colcon build
```

Run the marker detector.
```bash
ros2 launch aruco_marker_depthai stereo_marker_detector.launch.py
```

## License
[MIT](https://github.com/dan00n1/aruco-marker-detection/blob/0d4f772d2e9c2635cd3b6ded379c2c12e2cb8b23/LICENSE)
