#!/usr/bin/env python3
import rclpy
import os
import cv2
import sys
import math
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from scipy.spatial.transform import Rotation as R

"""
This script calculates the pose of an ArUco marker in the camera frame.

The pose of the marker is with respect to the camera lens frame.
Imagine you are looking through the camera viewfinder, the camera lens frame's:
 - x-axis points to the right
 - y-axis points straight down towards your toes
 - z-axis points straight ahead away from your eye, out of the camera
"""

# Set to True to print the transformation information
DEBUG_PRINT_VALUES = False

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

# ArUco dictionary name; change to the used dictionary; if you used the generator provided in the README, this should be the same
ARUCO_DICTIONARY_NAME = "DICT_4X4_1000"

# ArUco marker side length in meters; change to the correct side length
ARUCO_MARKER_SIDE_LENGTH = 0.040 #Example: 0.030 meters = 30 mm

# File path to the yaml file; change to the correct directory path and file name
DIRECTORY = "/home/danoon/shared/aruco-marker-detection/aruco_marker_depthai/scripts/camera_calibration/calibration_values/"
CAMERA_CALIBRATION_FILE_NAME = "calibration_chessboard.yaml"
CAMERA_CALIBRATION_FILE_PATH = os.path.join(DIRECTORY, CAMERA_CALIBRATION_FILE_NAME)

# Check if node values are the same as the saved values inside the YAML file.
MTX_NODE_NAME = "MTX"
DIST_NODE_NAME = "DIST"

class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__('aruco_pose_node')
        self.bridge = CvBridge()
       
        self.mtx, self.dst = self.get_camera_calibration_values()
        self.aruco_dictionary, self.aruco_parameters = self.get_aruco_dictionary()

        self.pubDetectedMarkerPose = self.create_publisher(Image, 'color/detected_marker_pose', 10)
        self.subImage = self.create_subscription(Image, 'color/image_rect', self.determine_pose, 10)
        

    def determine_pose(self, image_msg):
        """
        Determine the pose of the ArUco marker in the image

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
                transform_translation_x, transform_translation_y, transform_translation_z = self.get_transform_translations(i, tvecs)

                quaternion = self.store_rotation_data(i, rvecs)

                transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w = self.get_rotation_data(quaternion)

                roll_x, pitch_y, yaw_z = self.euler_from_quaternion(transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w)
                
                roll_x = self.calculate_degrees_from_radians(roll_x)
                pitch_y = self.calculate_degrees_from_radians(pitch_y)
                yaw_z = self.calculate_degrees_from_radians(yaw_z)

                if DEBUG_PRINT_VALUES:
                    self.print_transform_info(transform_translation_x, transform_translation_y, transform_translation_z, roll_x, pitch_y, yaw_z)
                
                cv2.aruco.drawAxis(self.frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05) # Draw the axes on the marker

        self.publish_image(self.frame)

    def publish_image(self, frame):
        """ Publish the image with the detected ArUco marker pose 
        
        Parameters:
            frame: The image with the detected ArUco marker pose
        """
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        self.pubDetectedMarkerPose.publish(image_message)

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

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        - roll is rotation around x in radians (counterclockwise)
        - pitch is rotation around y in radians (counterclockwise)
        - yaw is rotation around z in radians (counterclockwise)

        Parameters:
            x (float): The x value of the quaternion
            y (float): The y value of the quaternion
            z (float): The z value of the quaternion
            w (float): The w value of the quaternion

        Returns:
            roll_x (float): The roll angle in radians
            pitch_y (float): The pitch angle in radians
            yaw_z (float): The yaw angle in radians
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
            
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
            
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
            
        return roll_x, pitch_y, yaw_z # in radians

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

    def get_transform_translations(self, i, tvecs):
        """
        Get the translation (i.e. position) information
        
        Parameters:
            i (int): The index of the ArUco marker
            tvecs (list): The translation vectors
        """
        # Store the translation (i.e. position) information
        transform_translation_x = tvecs[i][0][0]
        transform_translation_y = tvecs[i][0][1]
        transform_translation_z = tvecs[i][0][2]

        return transform_translation_x, transform_translation_y, transform_translation_z

    def store_rotation_data(self, i, rvecs):
        """
        Store the rotation information
        
        Parameters:
            i (int): The index of the ArUco marker
            rvecs (list): The rotation vectors
            
        Returns:
            quaternion: The quaternion representation of the rotation
        """
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quaternion = r.as_quat()

        return quaternion

    def get_rotation_data(self, quaternion):
        """
        Get the rotation information
        
        Parameters:
            quaternion (list): The quaternion representation of the rotation  
            
        Returns:
            transform_rotation_x: The x value of the quaternion
            transform_rotation_y: The y value of the quaternion
            transform_rotation_z: The z value of the quaternion
            transform_rotation_w: The w value of the quaternion
        """
        transform_rotation_x = quaternion[0] 
        transform_rotation_y = quaternion[1] 
        transform_rotation_z = quaternion[2] 
        transform_rotation_w = quaternion[3]

        return transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w

    def calculate_degrees_from_radians(self, radians):
        """ Convert radians to degrees """
        return math.degrees(radians)

    def print_transform_info(self, transform_translation_x, transform_translation_y, transform_translation_z, roll_x, pitch_y, yaw_z):
        """ Print the transformation information """
        print("\ntransform_translation_x: {}".format(transform_translation_x))
        print("transform_translation_y: {}".format(transform_translation_y))
        print("transform_translation_z: {}".format(transform_translation_z))
        print("roll_x: {}".format(roll_x))
        print("pitch_y: {}".format(pitch_y))
        print("yaw_z: {}".format(yaw_z))

def main(args=None):
    rclpy.init(args=args)
    aruco_pose_node = ArucoPoseNode()
    rclpy.spin(aruco_pose_node)
    aruco_pose_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    