#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"


std::tuple<dai::Pipeline, int, int> createPipeline(bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution) {
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto monoLeftCam = pipeline.create<dai::node::MonoCamera>();
    auto monoRightCam = pipeline.create<dai::node::MonoCamera>();
    auto stereoDepth = pipeline.create<dai::node::StereoDepth>();

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();

    auto xoutColor = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutColor->setStreamName("color");
    xoutDepth->setStreamName("depth");

    int width, height;
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeftCam->setResolution(monoResolution);
    monoLeftCam->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRightCam->setResolution(monoResolution);
    monoRightCam->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    int rgbScaleNumerator = 2;
    int rgbScaleDinominator = 3;
    colorCam->setIspScale(rgbScaleNumerator, rgbScaleDinominator);
    colorCam->setPreviewSize(width, height);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // StereoDepth
    stereoDepth->initialConfig.setConfidenceThreshold(confidence);
    stereoDepth->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereoDepth->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereoDepth->setLeftRightCheck(lrcheck);
    stereoDepth->setExtendedDisparity(extended);
    stereoDepth->setSubpixel(subpixel);
    stereoDepth->setDepthAlign(dai::CameraBoardSocket::CAM_A); //added by gerard

    // Link plugins CAM -> STEREO -> XLINK
    monoLeftCam->out.link(stereoDepth->left);
    monoRightCam->out.link(stereoDepth->right);

    stereoDepth->rectifiedLeft.link(xoutLeft->input);
    stereoDepth->rectifiedRight.link(xoutRight->input);
    colorCam->preview.link(xoutColor->input);
    stereoDepth->depth.link(xoutDepth->input);


    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_node");

    std::string tfPrefix, mode, monoResolution;
    bool lrcheck, extended, subpixel, enableDepth;
    int confidence, LRchecktresh;
    int monoWidth, monoHeight;
    dai::Pipeline pipeline;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("mode", "depth");
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("confidence", 200);
    node->declare_parameter("LRchecktresh", 5);
    node->declare_parameter("monoResolution", "720p");

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("mode", mode);
    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("extended", extended);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);

    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution);
    
    dai::Device device(pipeline);
    auto leftQueue = device.getOutputQueue("left", 30, false);
    auto rightQueue = device.getOutputQueue("right", 30, false);
    auto colorQueue = device.getOutputQueue("color", 30, false);
    auto stereoQueue = device.getOutputQueue("depth", 30, false);

    auto calibrationHandler = device.readCalibration();
    
    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    dai::rosBridge::ImageConverter leftconverter(tfPrefix + "_left_camera_optical_frame", true);
    auto leftCameraInfo = leftconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
        leftQueue,
        node,
        std::string("left/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &leftconverter, std::placeholders::_1, std::placeholders::_2),
        30,
        leftCameraInfo,
        "left");
    leftPublish.addPublisherCallback();

    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
        rightQueue,
        node,
        std::string("right/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rightCameraInfo,
        "right");
    rightPublish.addPublisherCallback();

    dai::rosBridge::ImageConverter color_converter(tfPrefix + "_color_camera_optical_frame", true);
    auto colorCameraInfo = color_converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> colorPublish(
        colorQueue,
        node,
        std::string("color/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &color_converter, std::placeholders::_1, std::placeholders::_2),
        30,
        colorCameraInfo,
        "color");
    colorPublish.addPublisherCallback();

    auto depthconverter = rightconverter;//color_converter;
    auto depthCameraInfo = rightCameraInfo;//depthconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        stereoQueue,
        node,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                    &depthconverter,  // since the converter has the same frame name
                                    // and image type is also same we can reuse it
                    std::placeholders::_1,
                    std::placeholders::_2),
        30,
        depthCameraInfo,
        "stereo");
    depthPublish.addPublisherCallback();
    
    rclcpp::spin(node);

    return 0;
}
