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

# Set to True to print the camera pose in console
DEBUG_PRINT_CAMERA_POSE = False

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
ARUCO_MARKER_SIDE_LENGTH = 0.038 #Example: 0.030 meters = 30 mm

# File path to the yaml file; change to the correct directory path and file name
DIRECTORY = "/home/danoon/shared/aruco-marker-detection/aruco_marker_depthai/scripts/camera_calibration/calibration_values/"
CAMERA_CALIBRATION_FILE_NAME = "calibration_chessboard.yaml"
CAMERA_CALIBRATION_FILE_PATH = os.path.join(DIRECTORY, CAMERA_CALIBRATION_FILE_NAME)

# Check if node values are the same as the saved values inside the YAML file.
MTX_NODE_NAME = "MTX"
DIST_NODE_NAME = "DIST"

class CameraLocationMultipleMarkers(Node):
    def __init__(self):
        super().__init__('camera_location_multiple_markers')
        self.bridge = CvBridge()
        self.tfbroadcaster = TransformBroadcaster(self)

        self.aruco_dictionary, self.aruco_parameters = self.get_aruco_dictionary()
        self.mtx, self.dst = self.get_camera_calibration_values()    

        self.pubDetectedMarkerLocation = self.create_publisher(Image, 'color/detected_camera_location', 10)
        self.subImage = self.create_subscription(Image, 'color/image_rect', self.calculate_camera_pose, 10)

        self.pubDetectedCameraLocationTransforms = self.create_publisher(TransformStamped, 'detected_camera_location_transforms', 10)
        
    def get_camera_calibration_values(self):
        """ 
        Get the camera calibration values from the yaml file
        
        Returns:
            mtx: The camera matrix
            dst: The distortion coefficients

        Raises:
            IOError: If the camera calibration values are not found in the file
        """
        try:
            cv_file = cv2.FileStorage(CAMERA_CALIBRATION_FILE_PATH, cv2.FILE_STORAGE_READ)
            mtx = cv_file.getNode(MTX_NODE_NAME).mat()
            dst = cv_file.getNode(DIST_NODE_NAME).mat()
            cv_file.release()

            if mtx is None or dst is None:
                raise IOError(f"[ERROR] Unable to find camera calibration values in the file: {CAMERA_CALIBRATION_FILE_PATH}\n"
                              f"[INFO] Please ensure that the file exists and contains the correct calibration data.")

            return mtx, dst

        except Exception as e:
            print(f"{e}")
            sys.exit(0)

    def get_aruco_dictionary(self):
        """ 
        Get the ArUco dictionary and parameters
        
        Returns:
            aruco_dictionary: The ArUco dictionary
            aruco_parameters: The ArUco parameters
            
        Raises:
            SystemExit: If the ArUco dictionary is not supported
        """
        if ARUCO_DICT.get(ARUCO_DICTIONARY_NAME, None) is None:
            print(f"[ERROR] ArUCo tag of '{ARUCO_DICTIONARY_NAME}' is not supported")
            sys.exit(0)

        aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[ARUCO_DICTIONARY_NAME])
        aruco_parameters = cv2.aruco.DetectorParameters_create()
        print(f"[INFO] Detecting '{ARUCO_DICTIONARY_NAME}' markers")

        return aruco_dictionary, aruco_parameters
  
    def calculate_camera_pose(self, image_msg):
        """ 
        Calculate the camera pose
        
        Parameters:
            image_msg: The image message
        """
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')

        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(frame, self.aruco_dictionary, parameters=self.aruco_parameters, cameraMatrix=self.mtx, distCoeff=self.dst)
        
        # self.average_camera_position_calculation(frame, corners, marker_ids)

        if marker_ids is not None:
            self.draw_markers_on_frame(frame, corners, marker_ids)
            rvecs_four, tvecs_four, obj_points_four = cv2.aruco.estimatePoseSingleMarkers(corners, ARUCO_MARKER_SIDE_LENGTH, self.mtx, self.dst)

            all_tvecs = []
            for i in range(len(marker_ids)):
                rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers([corners[i]], ARUCO_MARKER_SIDE_LENGTH, self.mtx, self.dst)

                marker_id = marker_ids[i][0]
                if 0 <= marker_id <= 10: #FIXME: to fix the range of marker_id
                    # Store the rvecs and tvecs for all markers
                    rvec = rvecs[0]
                    tvec = tvecs[0]
                    marker_transform = self.create_marker_transform_stamped(marker_id, rvec, tvec)
                    self.tfbroadcaster.sendTransform(marker_transform)

                    all_tvecs.append(tvecs_four[i])

                    inverse_rvecs = [-r for r in rvecs]
                    inverse_tvecs = [-t for t in tvecs]
                    inverse_quaternion = self.set_rotation_data(0, inverse_rvecs)
                    camera_pose = (inverse_quaternion, inverse_tvecs)

                    # Publish the camera pose
                    self.publish_camera_transform(camera_pose, marker_id)
                

            if len(rvecs) > 0 and len(all_tvecs) > 0:
                avg_tvec = np.mean(all_tvecs, axis=0)
                inverse_tvec = -avg_tvec
                inverse_rvecs = [-r for r in rvecs]
                
                inverse_quaternion = self.set_rotation_data(0, inverse_rvecs)
                camera_pose = (inverse_quaternion, inverse_tvec)

                # Publish the camera pose
                self.publish_avg_camera_transform(camera_pose)

        self.publish_image(frame)

    def create_marker_transform_stamped(self, marker_id, rvec, tvec):
        """ 
        Create a TransformStamped message for the marker
        
        Parameters:
            marker_id: The ID of the marker
            rvec: The rotation vector
            tvec: The translation vector
        
        Returns:
            transform_stamped: The TransformStamped message
        """
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = f"world"  # Assuming the markers are fixed on the ground
        transform_stamped.child_frame_id = f"ar_{marker_id}"
        transform_stamped.transform.translation.x = float(tvec[0][0])
        transform_stamped.transform.translation.y = float(tvec[0][1])
        transform_stamped.transform.translation.z = float(0.0)  # Assuming the markers are on the ground
        # transform_stamped.transform.translation.z = float(tvec[0][2])
        
        # Calculate quaternion from rotation vector
        rotation_matrix = np.eye(3)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        r = R.from_matrix(rotation_matrix)
        quaternion = r.as_quat()

        transform_stamped.transform.rotation.x = float(quaternion[0])
        transform_stamped.transform.rotation.y = float(quaternion[1])
        transform_stamped.transform.rotation.z = float(quaternion[2])
        transform_stamped.transform.rotation.w = float(quaternion[3])
        
        return transform_stamped
    
    def publish_camera_transform(self, camera_pose, marker_id):
        """
        Publish the camera pose
        
        Parameters:
            camera_pose: A tuple containing the quaternion representing rotation and the translation vector representing position
        """
        quaternion, translation = camera_pose
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()

        transform_stamped.header.frame_id = f"world" 
        transform_stamped.child_frame_id = f"cam_{marker_id}"
        # Position the camera frame above the markers
        transform_stamped.transform.translation.x = float(translation[0][0][0])
        transform_stamped.transform.translation.y = float(translation[0][0][1])
        transform_stamped.transform.translation.z = -(float(translation[0][0][2]))

        transform_stamped.transform.rotation.x = float(quaternion[0])
        transform_stamped.transform.rotation.y = float(quaternion[1])
        transform_stamped.transform.rotation.z = float(quaternion[2])
        transform_stamped.transform.rotation.w = float(quaternion[3])

        self.tfbroadcaster.sendTransform(transform_stamped)
        
        # self.pubDetectedCameraLocationTransforms.publish(transform_stamped)

    
    def publish_avg_camera_transform(self, camera_pose):
        """
        Publish the camera pose
        
        Parameters:
            camera_pose: A tuple containing the quaternion representing rotation and the translation vector representing position
        """
        quaternion, translation = camera_pose
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()

        transform_stamped.header.frame_id = f"world" 
        transform_stamped.child_frame_id = f"cam_avg"
        # Position the camera frame above the markers
        transform_stamped.transform.translation.x = float(translation[0][0])
        transform_stamped.transform.translation.y = float(translation[0][1])
        transform_stamped.transform.translation.z = -(float(translation[0][2]))

        transform_stamped.transform.rotation.x = float(quaternion[0])
        transform_stamped.transform.rotation.y = float(quaternion[1])
        transform_stamped.transform.rotation.z = float(quaternion[2])
        transform_stamped.transform.rotation.w = float(quaternion[3])

        self.tfbroadcaster.sendTransform(transform_stamped)
        
        # self.pubDetectedCameraLocationTransforms.publish(transform_stamped)

    def draw_markers_on_frame(self, frame, corners, marker_ids):
        """
        Draw the markers on the frame
        
        Parameters:
            frame: The image with detected markers
            corners: The corners of the detected markers
            marker_ids: The IDs of the detected markers
        """
        cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
        for corner in corners:
            corner = corner.reshape(-1, 2).astype(int)
            cv2.polylines(frame, [corner], True, (0, 255, 0), thickness=3)  # Adjust thickness as needed

    def print_camera_pose(self, camera_pose):
        """
        Print the camera pose
        
        Parameters:
            camera_pose: A tuple containing the quaternion representing rotation and the translation vector representing position
        """
        quaternion, translation = camera_pose
        print(f"Translation: {translation}")
        print(f"Quaternion: {quaternion}")

    def set_rotation_data(self, i, rvecs):
        """
        Set the rotation data
        
        Parameters:
            i: The index of the rotation vector
            rvecs: The rotation vectors
        
        Returns:
            quaternion: The quaternion representing the rotation
        """
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quaternion = r.as_quat()   
        return quaternion
        
    def publish_image(self, frame):
        """
        Publish the image with detected markers
        
        Parameters:
            frame: The image with detected markers
        """
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        self.pubDetectedMarkerLocation.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    camera_location_multiple_markers = CameraLocationMultipleMarkers()
    rclpy.spin(camera_location_multiple_markers)
    camera_location_multiple_markers.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
