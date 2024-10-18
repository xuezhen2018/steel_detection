cmake_minimum_required(VERSION 3.0.2)
project(steel_detection)

SET (ONNXRUNTIME_DIR /home/lq/onnxruntime)
SET (cv_bridge_DIR /usr/local/bin/cmake)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  cv_bridge
)

add_message_files(
  FILES
  steeldetect.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES steel_detection
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${ONNXRUNTIME_DIR}/include
)


add_executable(steel_detect include/steel_detection/yolov8_onnx.h include/steel_detection/yolov8_utils.h src/steel_detection.cpp src/yolov8_onnx.cpp src/yolov8_utils.cpp)
target_link_libraries(steel_detect ${catkin_LIBRARIES} ${OpenCV_LIBS} "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")