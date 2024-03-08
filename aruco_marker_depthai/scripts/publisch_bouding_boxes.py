#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from depthai_ros_msgs.msg import SpatialDetectionArray
from sensor_msgs.msg import Image

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String

import cv2
from cv_bridge import CvBridge

import json
import random

class PublischBoundingBoxes(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        self.get_logger().info("PublischBoundingBoxes started")

        self.bridge = CvBridge()
        self.image = None

        self.declare_parameter("resourceBaseFolder", "")
        path = self.get_parameter("resourceBaseFolder").get_parameter_value().string_value
        # self.get_logger().info(path)
        self.declare_parameter("nnConfig", "")
        nnConfig = self.get_parameter("nnConfig").get_parameter_value().string_value
        # self.get_logger().info(nnConfig)
        nnConfigPath = path + '/' + nnConfig
        # self.get_logger().info(nnConfigPath)

        self.colors = []
        colors = []
        with open(nnConfigPath, 'r') as f:
            self.model_objects = json.loads(f.read())
            try:
                self.class_names = self.model_objects["class_names"]
                self.get_logger().info(str(self.class_names))
                colors = self.model_objects["colors"]
                for color in colors:
                    self.colors.append(colors[color])
            except:
                self.class_names = self.model_objects["mappings"]["labels"]
                self.get_logger().info("e: "+ str(self.class_names))

                for name in self.class_names:
                    self.colors.append("#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)]))


        self.subSpatialDetection = self.create_subscription(SpatialDetectionArray, 'color/yolov4_Spatial_detections', self.spatial_dections_callback, 10)
        # self.subSpatialDetection  # prevent unused variable warning

        self.subImage = self.create_subscription(Image, 'color/detections', self.image_callback, 10)
        # self.subSpatialDetection  # prevent unused variable warning

        self.pubImageBoudingBoxes = self.create_publisher(Image, 'color/image_w_bouding_boxes', 10)
        # self.pubTextMarkers = rospy.Publisher("spatialDetectionTextMarkers", ImageMarkerArray, queue_size=1)

    def spatial_dections_callback(self, spatial_detection_array_msg):
        number_of_bounding_boxes = 0
        if(self.image is not None):
            bouding_box_image = self.image
            for detection in spatial_detection_array_msg.detections:

                detectionID = None
                score = -1.0
                for result in detection.results:
                    if result.score > score:
                        detectionID = result.class_id
                        score = result.score

                #detectionID = detection.results[0].class_id
                #score = detection.results[0].score

                if(detectionID is not None):
                    x1 = detection.bbox.center.position.x - (detection.bbox.size_x / 2)
                    y1 = detection.bbox.center.position.y - (detection.bbox.size_y / 2)
                    x2 = detection.bbox.center.position.x + (detection.bbox.size_x / 2)
                    y2 = detection.bbox.center.position.y + (detection.bbox.size_y / 2)
                    #spatial detections
                    x = detection.position.x
                    y = detection.position.y
                    z = detection.position.z


                    class_index = int(detection.results[0].class_id)
                    score = detection.results[0].score
                    class_color = self.colors[int(class_index)].strip("#")
                    class_color = tuple(int(class_color[i:i + 2], 16) for i in (0, 2, 4))

                    cv2.rectangle(self.image, (int(x1), int(y1)), (int(x2), int(y2)), class_color, 2)

                    text = f'{self.class_names[class_index]}, : {round(score * 100,3)}'
                    #print(text)
                    image = cv2.putText(self.image, text, (int(x1)+5, int(y2)-5), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color, 2, cv2.LINE_AA)
                    text = 'x: %.3f m' % (x)
                    image = cv2.putText(self.image, text, (int(x1)+5, int(y2)+20), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color, 2, cv2.LINE_AA)
                    text = 'y: %.3f m' % (y)
                    image = cv2.putText(self.image, text, (int(x1)+5, int(y2)+45), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color, 2, cv2.LINE_AA)
                    text = 'z: %.3f m' % (z)
                    image = cv2.putText(self.image, text, (int(x1)+5, int(y2)+70), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color, 2, cv2.LINE_AA)
                    number_of_bounding_boxes = number_of_bounding_boxes + 1

            if number_of_bounding_boxes:
                image_message= self.bridge.cv2_to_imgmsg(bouding_box_image, encoding="passthrough")

                self.pubImageBoudingBoxes.publish(image_message)
                pass
      

    def image_callback(self, image_msg):

        self.image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
 
        pass

def main(args=None):
    rclpy.init(args=args)

    publich_bounding_boxes = PublischBoundingBoxes()

    rclpy.spin(publich_bounding_boxes)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publich_bounding_boxes.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
