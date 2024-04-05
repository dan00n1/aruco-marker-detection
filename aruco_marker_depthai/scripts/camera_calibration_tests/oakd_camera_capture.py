#!/usr/bin/env python3
import time
from pathlib import Path
import cv2
import depthai as dai
import os

image_counter = 0
 # Make sure the destination path is present before starting to store the examples
DIRECTORY_PATH = "/home/danoon/shared/aruco-marker-detection/aruco_marker_depthai/scripts/camera_calibration_tests/images_oakd_camera"

def setup_pipeline():
    """
    Create the pipeline and configure the camera
    
    Returns:
        pipeline: Pipeline object
    """
    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)

    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    camRgb.video.link(xoutRgb.input)

    xin = pipeline.create(dai.node.XLinkIn)
    xin.setStreamName("control")
    xin.out.link(camRgb.inputControl)

    # Properties
    videoEnc = pipeline.create(dai.node.VideoEncoder)
    videoEnc.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
    camRgb.still.link(videoEnc.input)

    # Linking
    xoutStill = pipeline.create(dai.node.XLinkOut)
    xoutStill.setStreamName("still")
    videoEnc.bitstream.link(xoutStill.input)

    return pipeline

def start_pipeline(pipeline):
    """
    Start the pipeline and get the output queues

    Args:
        pipeline: Pipeline to start

    Returns:
        rgb_output_queue: Output queue for the rgb frames
        still_output_queue: Output queue for the still frames
        control_output_queue: Input queue for the control commands
    """	
    with dai.Device(pipeline) as device:
        # Output queue will be used to get the rgb frames from the output defined above
        rgb_output_queue = device.getOutputQueue(name="rgb", maxSize=30, blocking=False)
        still_output_queue = device.getOutputQueue(name="still", maxSize=30, blocking=True)
        control_output_queue = device.getInputQueue(name="control")

        os.makedirs(DIRECTORY_PATH, exist_ok=True)

        return rgb_output_queue, still_output_queue, control_output_queue

def capture_images(rgb_output_queue, still_output_queue, control_output_queue):
    """
    Capture images from the camera and display them on the screen.
    Press 'c' to capture an image and 'q' to quit the program.

    Args:
        rgb_output_queue: Output queue for the rgb frames
        still_output_queue: Output queue for the still frames
        control_output_queue: Input queue for the control commands
    """
    while True:
        handle_rgb_frame(rgb_output_queue)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('c'):
            send_still_to_control_output(control_output_queue)
        
        if still_output_queue.has():
            save_image(still_output_queue)


def handle_rgb_frame(rgb_output_queue):
    """
    Handle the rgb frame output from the camera
    
    Args:
        rgb_output_queue: Output queue for the rgb frames
    """

    inRgb = rgb_output_queue.tryGet()
    if inRgb is not None:
        show_rgb_frame(inRgb)

def show_rgb_frame(inRgb):
    """
    Display the rgb frame on the screen

    Args:
        inRgb: RGB frame from the camera
    """
    frame = inRgb.getCvFrame()
    # 4k / 4
    frame = cv2.pyrDown(frame)
    frame = cv2.pyrDown(frame)
    cv2.imshow("rgb", frame)

def send_still_to_control_output(control_output_queue):
    """
    Send the 'still' event to the camera
    
    Args:
        control_output_queue: Input queue for the control commands
    """
    ctrl = dai.CameraControl()
    ctrl.setCaptureStill(True)
    control_output_queue.send(ctrl)
    print("Sent 'still' event to the camera!")

def save_image(still_output_queue):
    """
    Save the image to the disk
    
    Args:
        still_output_queue: Output queue for the still frames
    """
    image_file_path = f"{DIRECTORY_PATH}/chessboard_oakd_{image_counter}.jpeg"
    with open(image_file_path, "wb") as output_file:
        output_file.write(still_output_queue.get().getData())
        print('Image saved to', image_file_path)
        image_counter+=1

def start_camera_capture():
    """ Start the camera capture process """
    pipeline = setup_pipeline()
    rgb_output_queue, still_output_queue, control_output_queue = start_pipeline(pipeline)
    capture_images(rgb_output_queue, still_output_queue, control_output_queue)

start_camera_capture()