from launch import LaunchDescription
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    # Has its own pipeline to start the oak-d camera, so no stereo node is needed
    # Use 'q' to quit the program
    marker_orientation_tester = launch_ros.actions.Node(package='aruco_marker_depthai', executable='marker_orientation_tester.py', output='screen')
    
    ld = LaunchDescription()
    ld.add_action(marker_orientation_tester)
    
    return ld