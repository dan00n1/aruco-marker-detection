"""
This file is used to capture images from the OAK-D camera using the DepthAI library.

The images are saved to the Media/images_oakd_camera directory.
The script captures images when the 'c' key is pressed. The images are saved in JPEG format.

The image is only saved correctly when the message 'Image saved to ...' is printed in the console.

Keep in mind that the images that are being saved, are around 2MB in size.
I advise you to resize them before using them, so that they don't take up too much space.

Important note:
If the message is not printed, you may need to press the 'c' key again.
Sometimes even multiple times, as the camera may not be ready to capture the image.
Maybe waiting for a longer time before capturing again could help or try to restart the script.

"""

#!/usr/bin/env python3
import time
import datetime
import cv2
import depthai as dai
import os

DIRECTORY_NAME = "/home/danoon/shared/aruco-marker-detection/Media/images_oakd_camera"


def get_date_time():
    current_time_ms = int(time.time() * 1000)

    # Convert milliseconds to a datetime object
    current_datetime = datetime.datetime.fromtimestamp(current_time_ms / 1000.0)

    # Format the datetime object as a string for the filename
    date_time_string = current_datetime.strftime("%d-%m_%H-%M-%S")
    return date_time_string

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

videoEnc = pipeline.create(dai.node.VideoEncoder)
videoEnc.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
camRgb.still.link(videoEnc.input)

xoutStill = pipeline.create(dai.node.XLinkOut)
xoutStill.setStreamName("still")
videoEnc.bitstream.link(xoutStill.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    qRgb = device.getOutputQueue(name="rgb", maxSize=30, blocking=False)
    qStill = device.getOutputQueue(name="still")
    qControl = device.getInputQueue(name="control")

    # Create the directory if it does not exist
    os.makedirs(DIRECTORY_NAME, exist_ok=True)

    capturing = False # Flag to indicate if we are currently capturing an image

    while True:
        inRgb = qRgb.tryGet()  # Non-blocking call, will return a new data that has arrived or None otherwise
        if inRgb is not None:
            frame = inRgb.getCvFrame()
            # 4k / 4
            frame = cv2.pyrDown(frame) # Downscale twice
            frame = cv2.pyrDown(frame) 
            cv2.imshow("rgb", frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('c'):
            ctrl = dai.CameraControl()
            ctrl.setCaptureStill(True) # Capture a still image
            qControl.send(ctrl) 
            capturing = True 
            print("Sent 'still' event to the camera!")
            time.sleep(1.5)  # Wait for a short time before continuing

        if capturing and qStill.has():
            directory_name = f"{DIRECTORY_NAME}/chessboard_oakd_{get_date_time()}.jpeg"
            with open(directory_name, "wb") as new_file:
                new_file.write(qStill.get().getData()) 
                print('Image saved to', directory_name)
            capturing = False

cv2.destroyAllWindows()