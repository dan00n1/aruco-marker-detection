#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import depthai as dai
import numpy as np

class MarkerDetector(Node):
    """
    A ROS2 node for detecting markers in images and publishing the results.

    This node uses depthai library to detect ArUco markers in images captured
    by a color camera. It subscribes to the image topic, processes the images
    to detect ArUco markers, and publishes the detected markers' positions
    along with visualizations.

    Attributes:
        FONT_FACE: OpenCV font face for rendering text.
        FONT_SCALE: Font scale for rendering text.
        RED: Color for drawing marker information.
    """
    FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.5
    RED = (0, 0, 255)
    BLUE = (255, 0, 0)
    PINK = (255, 0, 255)
    AVERAGE_DEVIATION = 20

    def __init__(self):
        """
        Initializes the MarkerDetector node.

        Sets up publishers and initializes the depthai pipeline for marker detection.
        """
        super().__init__('marker_detector')

        # Publishers
        self.pub_detected_marker_image = self.create_publisher(Image, 'color/detected_markers', 10)
        self.pub_text_marker = self.create_publisher(Marker, 'color/ObjectText', 10)

        # Other setup
        self.bridge = CvBridge()
        self.pipeline = self.create_pipeline()
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

    def create_pipeline(self):
        """
        Creates and configures the depthai pipeline.

        Returns:
            Pipeline: Initialized depthai pipeline.
        """
        pipeline = dai.Pipeline()
        camRgb = pipeline.create(dai.node.ColorCamera)
        xoutVideo = pipeline.create(dai.node.XLinkOut)
        xoutPreview = pipeline.create(dai.node.XLinkOut)
        xoutVideo.setStreamName("video")
        xoutPreview.setStreamName("preview")
        self.set_camera_properties(camRgb)
        self.set_link_camera(camRgb, xoutVideo, xoutPreview)
        # Configuration of the pipeline omitted for brevity
        return pipeline

    def image_callback(self):
        """
        Image callback function.

        Processes incoming images to detect ArUco markers and publishes results.
        """
        with dai.Device(self.pipeline) as device:
            video = device.getOutputQueue('video')
            preview = device.getOutputQueue('preview')

            while rclpy.ok():
                video_frame = video.get()
                preview_frame = preview.get()
                frame = preview_frame.getCvFrame()

                # ArUco marker detection and visualization
                # Processing omitted for brevity

                corners, ids, rejected_img_points = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
                if ids is not None:
                    self.draw_detected_markers_on_screen(frame, corners, ids)
                    list_of_markers_found = [tuple(map(int, coords)) for coords in corners[0][0]]
                    self.draw_markers_on_screen(frame, list_of_markers_found)
                    cv2.imshow("ArUco Detection", frame)
                    cv2.waitKey(1)
                    self.publish_image(frame)

    def publish_image(self, frame):
        """
        Publishes the image message containing detected markers.

        Args:
            frame (numpy.ndarray): The frame containing detected markers.
        """
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        self.pub_detected_marker_image.publish(image_message)

    def draw_detected_markers_on_screen(self, frame, corners, ids):
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    def draw_markers_on_screen(self, frame, list_of_coordinates):
        for coordinates in list_of_coordinates:
            cv2.putText(frame, f"{coordinates}", (coordinates[0], coordinates[1] - 10), self.FONT_FACE, self.FONT_SCALE, self.RED, 2)

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
    """
    Main function to run the MarkerDetector node.

    Initializes the ROS2 node, creates an instance of MarkerDetector, and
    runs the image processing callback function.
    """
    rclpy.init(args=args)
    marker_detector = MarkerDetector()
    marker_detector.image_callback()
    rclpy.spin(marker_detector)
    marker_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
