#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import numpy as np
import depthai as dai
from visualization_msgs.msg import Marker
import ros2_numpy
import cv2
from cv_bridge import CvBridge
import tf2_ros
import geometry_msgs.msg
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MarkerDetector(Node):
    FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.5
    RED = (0, 0, 255)
    BLUE = (255, 0, 0)
    PINK = (255, 0, 255)
    AVERAGE_DEVIATION = 20

    def __init__(self):
        super().__init__('marker_detector')

        # self.subImage = self.create_subscription(Image, 'color/image_rect', self.image_callback, 10)
        self.pubDetectedMarkerImage = self.create_publisher(Image, 'color/detected_markers', 10)
        self.pubTextMarker = self.create_publisher(Marker, 'color/ObjectText', 10)

        self.bridge = CvBridge()

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        # Create self.pipeline
        self.pipeline = dai.Pipeline()

        # Define source and outputs
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.xoutVideo = self.pipeline.create(dai.node.XLinkOut)
        self.xoutPreview = self.pipeline.create(dai.node.XLinkOut)

        # Set stream names
        self.xoutVideo.setStreamName("video")
        self.xoutPreview.setStreamName("preview")

        # Set camera properties
        self.set_camera_properties(self.camRgb)
        self.set_link_camera(self.camRgb, self.xoutVideo, self.xoutPreview)

        
        # Define the ArUco parameters
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.image_callback()

    def image_callback(self):
        # self.frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        self.device = dai.Device(self.pipeline)

        # Connect to device and start self.pipeline
        video = self.device.getOutputQueue('video')
        preview = self.device.getOutputQueue('preview')

        while rclpy.ok():
            videoFrame = video.get()
            previewFrame = preview.get()
            self.frame = previewFrame.getCvFrame()

            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(self.frame, self.aruco_dict, parameters=self.aruco_params)
            ids_found = self.is_object_visible(ids)
            
            # If markers are detected, draw them on the frame and print their coordinates
            if ids_found:
                list_of_markers_found = []

                for i, marker_id in enumerate(ids):
                    marker_corner_points = [tuple(map(int, coords)) for coords in corners[i][0]]
                    list_of_markers_found.append(marker_corner_points)

                self.draw_detected_markers_on_screen(self.frame, corners, ids)

                list_of_corners = self.find_corners(list_of_markers_found)
                four_corners_are_found, list_of_four_corners = self.get_list_of_four_corners(list_of_corners)
                
                if four_corners_are_found:
                    self.draw_rectangle_around_four_corner_markers(self.frame, list_of_four_corners)
                    # print(self.calculate_rectangle_position(list_of_four_corners))
                
                self.draw_markers_on_screen(self.frame, list_of_markers_found, list_of_corners)
                
            cv2.imshow("ArUco Detection", self.frame)
            cv2.waitKey(1)

            self.publish_image(self.frame)

    def publish_image(self, frame):
        image_message= self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        self.pubDetectedMarkerImage.publish(image_message)

    def is_object_visible(self, ids):
        return ids is not None

    def get_top_left_coordinate_of_marker(self, marker):
        return marker[0]

    def find_corners(self, list_of_coordinates):
        top_left_marker, top_right_marker, bottom_right_marker, bottom_left_marker = None, None, None, None
        
        for coordinates in list_of_coordinates:
            top_left_coordinate = self.get_top_left_coordinate_of_marker(coordinates)
            
            if top_left_marker is None or top_left_coordinate[0] < top_left_marker[0][0] or (top_left_coordinate[0] <= top_left_marker[0][0] + self.AVERAGE_DEVIATION and top_left_coordinate[1] < top_left_marker[0][1]):
                top_left_marker = coordinates

            if top_right_marker is None or top_left_coordinate[1] < top_right_marker[0][1] or (top_left_coordinate[0] >= top_right_marker[0][0] + self.AVERAGE_DEVIATION and top_left_coordinate[1] < top_right_marker[0][1]):
                top_right_marker = coordinates

            if bottom_right_marker is None or top_left_coordinate[0] > bottom_right_marker[0][0] or (top_left_coordinate[0] >= bottom_right_marker[0][0] + self.AVERAGE_DEVIATION and top_left_coordinate[1] > bottom_right_marker[0][1]):
                bottom_right_marker = coordinates
        
            if bottom_left_marker is None or top_left_coordinate[1] > bottom_left_marker[0][1] or (top_left_coordinate[0] <= bottom_left_marker[0][0] + self.AVERAGE_DEVIATION and top_left_coordinate[1] > bottom_left_marker[0][1]):
                bottom_left_marker = coordinates

        return [top_left_marker, top_right_marker, bottom_right_marker, bottom_left_marker]

    def is_corner_list_unique(self, corners_found):
        unique_sublists = set(tuple(sorted(sublist)) for sublist in corners_found)
        return len(corners_found) == len(unique_sublists)

    def get_list_of_four_corners(self, corners_found):
        if self.is_corner_list_unique(corners_found) and len(corners_found) == 4:
            return True, corners_found
        else:
            return False, []

    def draw_detected_markers_on_screen(self, frame, corners, ids):
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    def sort_corner_coordinates(self, marker):
        return sorted(marker, key=lambda corner: corner[0] + corner[1])

    def get_marker(self, list_of_markers, position):
        return list_of_markers[position]

    def get_visual_corner_coordinates_of_marker(self, marker_list, marker_nr, corner_position):
        marker = self.get_marker(marker_list, marker_nr)
        marker_corners_sorted = self.sort_corner_coordinates(marker)
        return marker_corners_sorted[corner_position]

    def get_four_outer_corner_coordinates(self, corners_found):
        # Extract the sorted corners
        top_left_marker = self.get_visual_corner_coordinates_of_marker(corners_found, 0, 0)
        top_right_marker = self.get_visual_corner_coordinates_of_marker(corners_found, 1, 1)
        bottom_right_marker = self.get_visual_corner_coordinates_of_marker(corners_found, 2, 3)
        bottom_left_marker = self.get_visual_corner_coordinates_of_marker(corners_found, 3, 2)

        return top_left_marker, top_right_marker, bottom_right_marker, bottom_left_marker

    def draw_rectangle_around_four_corner_markers(self, frame, corners_found):
        top_left_marker, top_right_marker, bottom_right_marker, bottom_left_marker = self.get_four_outer_corner_coordinates(corners_found)
        # Draw lines
        cv2.line(frame, top_left_marker, top_right_marker, self.RED, 2)
        cv2.line(frame, top_right_marker, bottom_right_marker, self.BLUE, 2)
        cv2.line(frame, bottom_right_marker, bottom_left_marker, self.BLUE, 2)
        cv2.line(frame, bottom_left_marker, top_left_marker, self.BLUE, 2)

    def calculate_rectangle_position(self, corners_found):
        top_left_marker, top_right_marker, bottom_right_marker, bottom_left_marker = self.get_four_outer_corner_coordinates(corners_found)

        # Calculate the rectangle position
        min_x = min(top_left_marker[0], top_right_marker[0], bottom_right_marker[0], bottom_left_marker[0])
        max_x = max(top_left_marker[0], top_right_marker[0], bottom_right_marker[0], bottom_left_marker[0])
        min_y = min(top_left_marker[1], top_right_marker[1], bottom_right_marker[1], bottom_left_marker[1])
        max_y = max(top_left_marker[1], top_right_marker[1], bottom_right_marker[1], bottom_left_marker[1])

        return (min_x, min_y), (max_x, max_y)

    def draw_markers_on_screen(self, frame, list_of_coordinates, corners_found):
        for coordinates in list_of_coordinates:
            left_upper_coordinate = coordinates[0]
            
            text = f"{left_upper_coordinate}"
            if coordinates in corners_found:
                cv2.putText(frame, text, (left_upper_coordinate[0], left_upper_coordinate[1] - 10), self.FONT_FACE, self.FONT_SCALE, self.PINK, 2)
            else:
                cv2.putText(frame, text, (left_upper_coordinate[0], left_upper_coordinate[1] - 10), self.FONT_FACE, self.FONT_SCALE, self.RED, 2)

    def set_camera_properties(self, camRgb):
        camRgb.setPreviewSize(500, 500)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(True)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    def set_link_camera(self, camRgb, xoutVideo, xoutPreview):
        camRgb.video.link(xoutVideo.input)
        camRgb.preview.link(xoutPreview.input)

def main(args=None):
    rclpy.init(args=args)
    marker_detector = MarkerDetector()
    rclpy.spin(marker_detector)
    marker_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()