import os
# from classes import Camera
# from classes.enums import Positions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    depthai_examples_path = get_package_share_directory('my_depthai_ros2')
    default_rviz = os.path.join(get_package_share_directory('my_depthai_ros2'), 'rviz', 'spatialDetections.rviz')
    
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')
    default_resources_path = os.path.join(depthai_examples_path, 'resources')
    # print('Default resources path..............')
    # print(default_resources_path)

    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix    = LaunchConfiguration('tf_prefix',     default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    spatial_camera = LaunchConfiguration('spatial_camera',  default = True)
    # print(spatial_camera)

    cam_pos_x = LaunchConfiguration('cam_pos_x',     default = '0.25')
    cam_pos_y = LaunchConfiguration('cam_pos_y',     default = '0.0') #camera model
    cam_pos_z = LaunchConfiguration('cam_pos_z',     default = '0.5')
    cam_roll  = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw   = LaunchConfiguration('cam_yaw',       default = '0.0')

    camera_param_uri   = LaunchConfiguration('camera_param_uri',  default = 'package://my_depthai_ros2/params/camera')
    sync_nn            = LaunchConfiguration('sync_nn',           default = True)
    subpixel           = LaunchConfiguration('subpixel',          default = True)

    nnName              = LaunchConfiguration('nnName', default = "SimpleFruitsv1iyolov5pytorch_openvino_2021.4_6shave.blob")
    nnConfig            = LaunchConfiguration('nnConfig', default = "SimpleFruitsv1iyolov5pytorch.json")
    resourceBaseFolder  = LaunchConfiguration('resourceBaseFolder', default = default_resources_path)
    confidence          = LaunchConfiguration('confidence',        default = 200)
    lrcheck             = LaunchConfiguration('lrcheck', default = True)
    extended            = LaunchConfiguration('extended', default = False)
    LRchecktresh        = LaunchConfiguration('LRchecktresh',      default = 5)
    monoResolution      = LaunchConfiguration('monoResolution',  default = '400p')
    publish_grayscale_image  = LaunchConfiguration('publish_grayscale_image', default = True)
    publish_depth_image      = LaunchConfiguration('publish_depth_image', default = True)

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Set spatial_camera:=False for `OAK-1, OAK-1-LITE, OAK-1-MAX`.')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

    declare_camera_param_uri_cmd = DeclareLaunchArgument(
        'camera_param_uri',
        default_value=camera_param_uri,
        description='Sending camera yaml path')

    declare_sync_nn_cmd = DeclareLaunchArgument(
        'sync_nn',
        default_value=sync_nn,
        description='Syncs the image output with the Detection.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='Enables subpixel stereo detection.')

    declare_nnName_cmd = DeclareLaunchArgument(
        'nnName',
        default_value=nnName,
        description='Path to the object detection blob needed for detection')
 
    declare_nnConfig_cmd = DeclareLaunchArgument(
        'nnConfig',
        default_value=nnConfig,
        description='Path to the object detection blob-configuration needed for detection')
    
    declare_resourceBaseFolder_cmd = DeclareLaunchArgument(
        'resourceBaseFolder',
        default_value=resourceBaseFolder,
        description='Path to the resources folder which contains the default blobs for the network')
    
    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='Confidence that the disparity from the feature matching was good. 0-255. 255 being the lowest confidence.')
    
    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='LR Threshold is the threshod of how much off the disparity on the l->r and r->l  ')

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_monoResolution_cmd = DeclareLaunchArgument(
        'monoResolution',
        default_value=monoResolution,
        description='Contains the resolution of the Mono Cameras. Available resolutions are 800p, 720p & 400p for OAK-D & 480p for OAK-D-Lite.')

    declare_publish_depth_image_cmd = DeclareLaunchArgument(
        'publish_depth_image',
        default_value=publish_depth_image,
        description='Specifies using depth_image publishing')

    declare_publish_grayscale_image_cmd = DeclareLaunchArgument(
        'publish_grayscale_image',
        default_value=publish_grayscale_image,
        description='Specifies using gray-scale_image publishing')


    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'tf_prefix'   : tf_prefix,
                                              'camera_model': camera_model,
                                              'base_frame'  : base_frame,
                                              'parent_frame': parent_frame,
                                              'cam_pos_x'   : cam_pos_x,
                                              'cam_pos_y'   : cam_pos_y,
                                              'cam_pos_z'   : cam_pos_z,
                                              'cam_roll'    : cam_roll,
                                              'cam_pitch'   : cam_pitch,
                                              'cam_yaw'     : cam_yaw}.items())
    
    yolo_spatial_detector_node = launch_ros.actions.Node(
            package='my_depthai_ros2', executable='yolo_spatial_detector_node',
            output='screen',
            parameters=[{'tf_prefix': tf_prefix},
                        {'camera_param_uri': camera_param_uri},
                        {'nnName': nnName},
                        {'nnConfig': nnConfig},
                        {'resourceBaseFolder': resourceBaseFolder},
                        {'sync_nn': sync_nn},
                        {'monoResolution': monoResolution},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel},
                        {'confidence': confidence},
                        {'LRchecktresh': LRchecktresh},                        
                        {'spatial_camera': spatial_camera},
                        {'publish_grayscale_image': publish_grayscale_image},
                        {'publish_depth_image': publish_depth_image}])

    metric_converter_node = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    remappings=[('image_raw', '/stereo/depth'),
                                ('camera_info', '/stereo/camera_info'),
                                ('image', '/stereo/converted_depth')]
                ),
            ],
            output='screen',)

    color_pointcloud = True


    if not color_pointcloud:
        point_cloud_node = launch_ros.actions.ComposableNodeContainer(
                name='container2',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    # Driver itself
                    launch_ros.descriptions.ComposableNode(
                        package='depth_image_proc',
                        plugin='depth_image_proc::PointCloudXyziNode',
                        name='point_cloud_xyzi',

                        remappings=[('depth/image_rect', '/stereo/converted_depth'),
                                    ('intensity/image_rect', '/right/image_rect'),
                                    ('intensity/camera_info', '/right/camera_info'),
                                    ('points', '/stereo/points')]
                    ),
                ],
                output='screen',)
    else:
        point_cloud_node = launch_ros.actions.ComposableNodeContainer(
                name='container2',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    # Driver itself
                    launch_ros.descriptions.ComposableNode(
                        package='depth_image_proc',
                        plugin='depth_image_proc::PointCloudXyzrgbNode',
                        name='point_cloud_xyzrgb',

                        remappings=[('depth_registered/image_rect', '/stereo/converted_depth'),
                                    ('rgb/image_rect_color', '/color/image_rect'),
                                    ('rgb/camera_info', '/color/camera_info'),
                                    ('points', '/stereo/points')]
                    ),
                ],
                output='screen',)


    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz])

    bounding_boxes_node = launch_ros.actions.Node(
            package='my_depthai_ros2', executable='publisch_bouding_boxes.py',
            output='screen',
            parameters=[{'nnConfig': nnConfig},
                        {'resourceBaseFolder': resourceBaseFolder}])

    tf_node = launch_ros.actions.Node(
            package='my_depthai_ros2', executable='publisch_tf.py',
            output='screen',
            parameters=[{'nnConfig': nnConfig},
                        {'resourceBaseFolder': resourceBaseFolder}])

    ld = LaunchDescription()
    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(declare_camera_model_cmd)
    
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)
    
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    
    ld.add_action(declare_camera_param_uri_cmd)
    ld.add_action(declare_nnName_cmd)
    ld.add_action(declare_nnConfig_cmd)
    ld.add_action(declare_resourceBaseFolder_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_sync_nn_cmd)
    ld.add_action(urdf_launch)

    # ld.add_action(declare_lrcheck_cmd)
    # ld.add_action(declare_extended_cmd)
    # ld.add_action(declare_subpixel_cmd)
    # ld.add_action(declare_LRchecktresh_cmd)
    # ld.add_action(declare_monoResolution_cmd)

    #ld.add_action(declare_mode_cmd)
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)
    ld.add_action(declare_monoResolution_cmd)
    ld.add_action(declare_publish_depth_image_cmd)    
    ld.add_action(declare_publish_grayscale_image_cmd)

    ld.add_action(yolo_spatial_detector_node)

    ld.add_action(metric_converter_node)
    ld.add_action(point_cloud_node)

    ld.add_action(rviz_node)
    ld.add_action(bounding_boxes_node)
    ld.add_action(tf_node)
    return ld

