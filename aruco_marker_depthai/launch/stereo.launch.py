import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('aruco_marker_depthai'), 'rviz', 'stereoInputOnly.rviz')

    tf_prefix = LaunchConfiguration('tf_prefix', default = 'oak')
    mode = LaunchConfiguration('mode', default = 'depth')
    lrcheck = LaunchConfiguration('lrcheck', default = True)
    extended = LaunchConfiguration('extended', default = False)
    subpixel = LaunchConfiguration('subpixel', default = True)
    confidence = LaunchConfiguration('confidence', default = 200)
    LRchecktresh = LaunchConfiguration('LRchecktresh', default = 5)
    monoResolution = LaunchConfiguration('monoResolution',  default = '720p')

    stereo_node = launch_ros.actions.Node(
        package='aruco_marker_depthai', 
        executable='stereo_node',
        output='screen',
        parameters=[{'tf_prefix': tf_prefix},
                    {'mode': mode},
                    {'lrcheck': lrcheck},
                    {'extended': extended},
                    {'subpixel': subpixel},
                    {'confidence': confidence},
                    {'LRchecktresh': LRchecktresh},
                    {'monoResolution': monoResolution}])
    
    rviz_node = launch_ros.actions.Node(package='rviz2', executable='rviz2', output='screen', arguments=['--display-config', default_rviz])

    ld = LaunchDescription()
    ld.add_action(stereo_node)
    ld.add_action(rviz_node)

    return ld