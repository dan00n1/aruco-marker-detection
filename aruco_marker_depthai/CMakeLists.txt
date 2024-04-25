cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
project(aruco_marker_depthai VERSION 2.8.1 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()



find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

message(STATUS "------------------------------------------")
message(STATUS "Depthai Bridge is being built using AMENT.")
message(STATUS "------------------------------------------")

find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(rclpy REQUIRED)


set(dependencies
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  depthai_bridge
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)



macro(dai_add_node_ros1 node_name node_src1)
  add_executable("${node_name}" "${node_src1}")

  target_link_libraries("${node_name}"
                        depthai::core
                        opencv_imgproc
                        opencv_highgui
                        jsoncpp)

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

macro(dai_add_node_ros2 node_name node_src1 node_src2)
  add_executable("${node_name}" "${node_src1}" "${node_src2}")

  target_link_libraries("${node_name}"
                        depthai::core
                        opencv_imgproc
                        opencv_highgui
                        jsoncpp)

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

dai_add_node_ros1(stereo_node src/stereo_publisher.cpp)

if($ENV{ROS_DISTRO} STREQUAL "galactic")
  target_compile_definitions(rgb_stereo_node PRIVATE IS_GALACTIC)
endif()


install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
# Install Python modules
#ament_python_install_package(${PROJECT_NAME})
#install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts)
#FILES_MATCHING PATTERN "*.py")

install(TARGETS
        stereo_node
        DESTINATION lib/${PROJECT_NAME})



install(PROGRAMS
        scripts/marker_detector.py
        scripts/aruco_pose_node.py
        scripts/aruco_location.py
        scripts/marker_orientation_tester.py
        scripts/camera_location_singular_marker.py
        scripts/camera_location_multiple_markers.py
        scripts/camera_location_multiple_markers_multi_cameras.py
        DESTINATION lib/${PROJECT_NAME})


ament_package()