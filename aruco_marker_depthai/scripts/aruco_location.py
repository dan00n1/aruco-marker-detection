#!/usr/bin/env python3
import cv2
import os
import rclpy
import sys
import numpy as np 
from rclpy.node import Node 
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image 
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

"""
This script calculates the location of an ArUco marker in the camera frame.
Uses rviz to visualize the marker location.

The pose of the marker is with respect to the camera lens frame.
Imagine you are looking through the camera viewfinder, the camera lens frame's:
 - x-axis points to the right
 - y-axis points straight down towards your toes
 - z-axis points straight ahead away from your eye, out of the camera
"""

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
        self.tfbroadcaster = TransformBroadcaster(self)

        self.aruco_dictionary, self.aruco_parameters = self.get_aruco_dictionary()
        self.mtx, self.dst = self.get_camera_calibration_values()    

        self.pubDetectedMarkerLocation = self.create_publisher(Image, 'color/detected_marker_location', 10)
        self.subImage = self.create_subscription(Image, 'color/image_rect', self.calculate_marker_location, 10)
        
    def get_camera_calibration_values(self):
        """ 
        Get the camera calibration values from the saved file. 
        
        Returns:
            mtx: The camera matrix
            dst: The distortion coefficients
            
        Raises:
            IOError: If the camera calibration values are not found
        """
        try:
            cv_file = cv2.FileStorage(CAMERA_CALIBRATION_FILE_PATH, cv2.FILE_STORAGE_READ)
            mtx = cv_file.getNode(MTX_NODE_NAME).mat()
            dst = cv_file.getNode(DIST_NODE_NAME).mat()
            cv_file.release()

            # Check that the camera calibration values were found
            if mtx is None or dst is None:
                raise IOError(f"[ERROR] Unable to find camera calibration values in the file: {CAMERA_CALIBRATION_FILE_PATH}\n"
                              f"[INFO] Please ensure that the file exists and contains the correct calibration data.")

            return mtx, dst

        except Exception as e:
            print(f"{e}")
            sys.exit(0)

    def get_aruco_dictionary(self):
        """
        Get the ArUco dictionary
        
        Returns:
            aruco_dictionary: The ArUco dictionary
            aruco_parameters: The ArUco parameters
            
        Raises:
            IOError: If the ArUco dictionary is not supported
        """
        # Check that we have a valid ArUco marker
        if ARUCO_DICT.get(ARUCO_DICTIONARY_NAME, None) is None:
            print(f"[ERROR] ArUCo tag of '{ARUCO_DICTIONARY_NAME}' is not supported")
            sys.exit(0)

        # Load the ArUco dictionary
        aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[ARUCO_DICTIONARY_NAME])
        aruco_parameters = cv2.aruco.DetectorParameters_create()
        print(f"[INFO] Detecting '{ARUCO_DICTIONARY_NAME}' markers")

        return aruco_dictionary, aruco_parameters
  
    def calculate_marker_location(self, image_msg):
        """
        Calculate the location of the ArUco marker in the image
        
        Parameters:
            image_msg: The image message   
        """
        self.frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')

        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(self.frame, self.aruco_dictionary, parameters=self.aruco_parameters, cameraMatrix=self.mtx, distCoeff=self.dst)

        # Check that at least one ArUco marker was detected
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(self.frame, corners, marker_ids)

            # Get the rotation and translation vectors
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, ARUCO_MARKER_SIDE_LENGTH, self.mtx, self.dst)

            for i, marker_id in enumerate(marker_ids):  
                transform_stamped = self.create_transform_stamped() # Create a TransformStamped message
                transform_stamped = self.update_transform_stamped(transform_stamped, i, tvecs, rvecs) # Update the TransformStamped message
                
                # Send the transform
                self.tfbroadcaster.sendTransform(transform_stamped)    
                        
                # Draw the axes on the marker
                cv2.aruco.drawAxis(self.frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)        
                    
        self.publish_image(self.frame)

    def publish_image(self, frame):
        """
        Publish the image with the detected marker location
        
        Parameters:
            frame: The image with the detected marker location
        """
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        self.pubDetectedMarkerLocation.publish(image_message)

    def create_transform_stamped(self):
        """ 
        Create a TransformStamped message
        
        Returns:
            transform_stamped: The TransformStamped message
        """
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'camera_depth_frame'
        transform_stamped.child_frame_id = 'aruco_marker'
        return transform_stamped

    def update_transform_stamped(self, transform_stamped, i, tvecs, rvecs):
        """
        Update the TransformStamped message with the translation and rotation data
        
        Parameters:
            transform_stamped: The TransformStamped message
            i: The index of the marker
            tvecs: The translation vectors
            rvecs: The rotation vectors
        
        Returns:
            transform_stamped: The updated TransformStamped message"""
        transform_stamped = self.set_translation_data(transform_stamped, i, tvecs)
        quaternion = self.set_rotation_data(i, rvecs)
        transform_stamped = self.set_transform_rotation_data(transform_stamped, quaternion)
        return transform_stamped

    def set_translation_data(self, transform_stamped, i, tvecs):
        """
        Set the translation data for the TransformStamped message
        
        Parameters:
            transform_stamped: The TransformStamped message
            i: The index of the marker
            tvecs: The translation vectors
        
        Returns:
            transform_stamped: The updated TransformStamped message
        """
        transform_stamped.transform.translation.x = tvecs[i][0][0]
        transform_stamped.transform.translation.y = tvecs[i][0][1]
        transform_stamped.transform.translation.z = tvecs[i][0][2]
        return transform_stamped

    def set_rotation_data(self, i, rvecs):
        """
        Set the rotation data for the TransformStamped message
        
        Parameters:
            i: The index of the marker
            rvecs: The rotation vectors
        
        Returns:
            quaternion: The rotation data as a quaternion
        """
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quaternion = r.as_quat()   

        return quaternion

    def set_transform_rotation_data(self, transform_stamped, quaternion):
        """ 
        Set the rotation data for the TransformStamped message
        
        Parameters:
            transform_stamped: The TransformStamped message
            quaternion: The rotation data as a quaternion
        
        Returns:
            transform_stamped: The updated TransformStamped message
        """
        transform_stamped.transform.rotation.x = quaternion[0] 
        transform_stamped.transform.rotation.y = quaternion[1] 
        transform_stamped.transform.rotation.z = quaternion[2] 
        transform_stamped.transform.rotation.w = quaternion[3] 

        return transform_stamped

def main(args=None):
    rclpy.init(args=args)
    aruco_location = ArucoLocation()
    rclpy.spin(aruco_location)
    aruco_location.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()