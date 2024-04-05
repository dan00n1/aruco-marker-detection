from launch import LaunchDescription
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    # Only the marker detector node script
    marker_detector_node = launch_ros.actions.Node(package='aruco_marker_depthai', executable='marker_detector.py', output='screen')

    ld = LaunchDescription()
    ld.add_action(marker_detector_node)
    return ld