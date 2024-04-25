#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import depthai as dai
import cv2
import sys

# For the cv2.putText() function
FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.5
RED = (0, 0, 255)
ORANGE = (0, 128, 255)

class MarkerOrientationTester(Node):
    def __init__(self):
        super().__init__('marker_orientation_tester')
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

        self.check_marker_orientation()

    def check_marker_orientation(self):
        """ Check if the ArUco markers are detected in the frame. If detected, draw the markers and the top_left corner of each marker on the screen. """
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

            cv2.imshow("ArUco Detection", self.frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Press 'q' to quit
                sys.exit(0)

    def is_object_visible(self, ids):
        """ 
        Check if the object is visible in the frame 
        
        Parameters:
            ids: The IDs of the detected markers
        
        Returns:
            True if the object is visible, False otherwise
        """

        return ids is not None

    def draw_detected_markers_on_screen(self, frame, corners, ids):
        """ 
        Draw the detected markers and the top_left corner of each marker on the screen 
        
        Parameters:
            frame: The frame with the detected markers
            corners: The corners of the detected markers
            ids: The IDs of the detected markers
        """
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, corner in enumerate(corners):
            marker = corner[0][0]
            cv2.putText(frame, "x", (int(marker[0]), int(marker[1])+10), FONT_FACE, FONT_SCALE+0.5, RED, 4)
            cv2.putText(frame, f"Upper left", (int(marker[0])+20, int(marker[1])+10), FONT_FACE, FONT_SCALE, ORANGE, 2)

    def set_camera_properties(self, camRgb):
        """
        Set the camera properties 
        
        Parameters:
            camRgb: The ColorCamera node
        """
        camRgb.setPreviewSize(500, 500)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(True)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    def set_link_camera(self, camRgb, xoutVideo, xoutPreview):
        """
        Link the camera to the video and preview outputs
        
        Parameters:
            camRgb: The ColorCamera node
            xoutVideo: The XLinkOut node for video
            xoutPreview: The XLinkOut node for preview
        """
        camRgb.video.link(xoutVideo.input)
        camRgb.preview.link(xoutPreview.input)

def main(args=None):
    rclpy.init(args=args)
    marker_orientation_tester = MarkerOrientationTester()
    rclpy.spin(marker_orientation_tester)
    marker_orientation_tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()