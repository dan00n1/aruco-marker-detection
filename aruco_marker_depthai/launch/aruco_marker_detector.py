#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define source and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
xoutVideo = pipeline.create(dai.node.XLinkOut)
xoutPreview = pipeline.create(dai.node.XLinkOut)

xoutVideo.setStreamName("video")
xoutPreview.setStreamName("preview")

# Properties
# camRgb.setPreviewSize(300, 300)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(True)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Linking
camRgb.video.link(xoutVideo.input)
camRgb.preview.link(xoutPreview.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    video = device.getOutputQueue('video')
    preview = device.getOutputQueue('preview')

    # Define the ArUco parameters
    aruco_params = cv2.aruco.DetectorParameters_create()
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

    while True:
        videoFrame = video.get()
        previewFrame = preview.get()

        # Get BGR frame from NV12 encoded video frame
        frame = previewFrame.getCvFrame()

        # Detect ArUco markers in the frame
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

        # If markers are detected, draw them on the frame and print their coordinates
        if ids is not None:
            for i, marker_id in enumerate(ids):
                # Draw marker
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Convert corner coordinates to integers
                marker_coords_int = tuple(map(int, corners[i][0][0]))

                # Print marker ID and its corner coordinates
                marker_text = f"ID: {marker_id}, Coords: {marker_coords_int}"
                cv2.putText(frame, marker_text, (marker_coords_int[0], marker_coords_int[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the frame with detected markers
        cv2.imshow("ArUco Detection", frame)
        cv2.imshow("preview", previewFrame.getFrame())

        if cv2.waitKey(1) == ord('q'):
            break
