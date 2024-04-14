#!/usr/bin/env python3
import cv2
import os
import rclpy
import numpy as np 
from rclpy.node import Node 
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image 
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

import depthai as dai # TODO: remove when not needed anymore

# The different ArUco dictionaries built into the OpenCV library. 
# This list is used to check if the user has selected a valid ArUco dictionary.
ARUCO_DICT = {
"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# ArUco dictionary name; change to the used dictionary
ARUCO_DICTIONARY_NAME = "DICT_4X4_1000"

# ArUco marker side length in meters; change to the correct side length
ARUCO_MARKER_SIDE_LENGTH = 0.030 # 28mm

# File path to the input image; change to the correct directory path and file name
DIRECTORY = "/home/danoon/shared/aruco-marker-detection/aruco_marker_depthai/scripts/camera_calibration/calibration_values/"
CAMERA_CALIBRATION_FILE_NAME = "calibration_chessboard.yaml"
CAMERA_CALIBRATION_FILE_PATH = os.path.join(DIRECTORY, CAMERA_CALIBRATION_FILE_NAME)

# Check if node values are the same as the saved values inside the YAML file.
MTX_NODE_NAME = "MTX"
DIST_NODE_NAME = "DIST"

class ArucoLocation(Node):
    def __init__(self):
        super().__init__('aruco_location')
    
        self.bridge = CvBridge()

        # FIXME: Remove and use constants shown above
        self.declare_parameter("aruco_dictionary_name", "DICT_4X4_1000")
        self.declare_parameter("aruco_marker_side_length", 0.030)
        self.declare_parameter("camera_calibration_parameters_filename", "/calibration_chessboard.yaml")
        self.declare_parameter("image_topic", "/video_frames")
        self.declare_parameter("aruco_marker_name", "aruco_marker")
        
        # Read parameters
        aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
        self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value
        self.camera_calibration_parameters_filename = self.get_parameter("camera_calibration_parameters_filename").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value
    
        # Check that we have a valid ArUco marker
        if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
            self.get_logger().info("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
            
        # Load the camera parameters from the saved file
        cv_file = cv2.FileStorage(self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
        self.mtx = cv_file.getNode('K').mat()
        self.dst = cv_file.getNode('D').mat()
        cv_file.release()
        
        # Load the ArUco dictionary
        self.get_logger().info("[INFO] detecting '{}' markers...".format(aruco_dictionary_name))
        self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        
        # Create the subscriber. This subscriber will receive an Image from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(Image, image_topic, self.listener_callback, 10)
        
        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)
        
    
    def listener_callback(self, data):
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data)
        
        # Detect ArUco markers in the video frame
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters, cameraMatrix=self.mtx, distCoeff=self.dst)
    
       # Check that at least one ArUco marker was detected
        if marker_ids is not None:
            # Draw a square around detected markers in the video frame
            cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
        
            # Get the rotation and translation vectors
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_side_length, self.mtx, self.dst)
                
            # The pose of the marker is with respect to the camera lens frame.
            # Imagine you are looking through the camera viewfinder, the camera lens frame's:
            # x-axis points to the right
            # y-axis points straight down towards your toes
            # z-axis points straight ahead away from your eye, out of the camera
            for i, marker_id in enumerate(marker_ids):  
                # Create the coordinate transform
                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = self.get_clock().now().to_msg()
                transform_stamped.header.frame_id = 'camera_depth_frame'
                transform_stamped.child_frame_id = self.aruco_marker_name
            
                # Store the translation (i.e. position) information
                transform_stamped.transform.translation.x = tvecs[i][0][0]
                transform_stamped.transform.translation.y = tvecs[i][0][1]
                transform_stamped.transform.translation.z = tvecs[i][0][2]
        
                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()   
                
                # Quaternion format     
                transform_stamped.transform.rotation.x = quat[0] 
                transform_stamped.transform.rotation.y = quat[1] 
                transform_stamped.transform.rotation.z = quat[2] 
                transform_stamped.transform.rotation.w = quat[3] 
        
                # Send the transform
                self.tfbroadcaster.sendTransform(transform_stamped)    
                        
                # Draw the axes on the marker
                cv2.aruco.drawAxis(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)        
                    
            cv2.imshow("camera", current_frame)
            cv2.waitKey(1)
   
def main(args=None):
    rclpy.init(args=args)
    aruco_location = ArucoLocation()
    rclpy.spin(aruco_location)
    aruco_location.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()