import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    depthai_examples_path = get_package_share_directory('my_depthai_ros2')

    default_resources_path = os.path.join(depthai_examples_path,
                                'resources')
    print('Default resources path..............')
    print(default_resources_path)


    nnConfig             = LaunchConfiguration('nnConfig', default = "SimpleFruitsv1iyolov5pytorch.json")
    resourceBaseFolder = LaunchConfiguration('resourceBaseFolder', default = default_resources_path)

    declare_nnConfig_cmd = DeclareLaunchArgument(
        'nnConfig',
        default_value=nnConfig,
        description='Path to the object detection blob-configuration needed for detection')
    
    declare_resourceBaseFolder_cmd = DeclareLaunchArgument(
        'resourceBaseFolder',
        default_value=resourceBaseFolder,
        description='Path to the resources folder which contains the default blobs for the network')
    
  
    publisch_tf_node = launch_ros.actions.Node(
            package='my_depthai_ros2', executable='publisch_tf.py',
            output='screen',
            parameters=[{'nnConfig': nnConfig},
                        {'resourceBaseFolder': resourceBaseFolder}])

    ld = LaunchDescription()
    ld.add_action(declare_nnConfig_cmd)
    ld.add_action(declare_resourceBaseFolder_cmd)
    ld.add_action(publisch_tf_node)
    return ld

