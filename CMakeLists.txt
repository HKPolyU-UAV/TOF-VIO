cmake_minimum_required(VERSION 2.8.3)
project(tof_vio)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

find_package (OpenCV 3 REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    nodelet
    roscpp
    rostime
    sensor_msgs
    std_msgs
    geometry_msgs
    pcl_conversions
    pcl_ros
    cv_bridge
    message_generation
    image_transport
    tf
    )


## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    cv_bridge
    roscpp
    std_msgs
    image_transport
    )


include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/include/"
    )

##################################################################################
add_library(nodelet_icp
    src/vo/node_vo.cpp
    src/vo/salientpts.cpp
    src/vo/icp.cpp
    src/vo/tof_frame.cpp
    )
target_link_libraries(nodelet_icp
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
##################################################################################
add_executable(ekf
    src/ekf/node_ekf.cpp
    src/ekf/ekf_model.cpp
    )
target_link_libraries(ekf
    ${catkin_LIBRARIES} )
##################################################################################
add_executable(eskf
    src/eskf/node_eskf.cpp
    src/eskf/eskf_imu.cpp
    )
target_link_libraries(eskf
    ${catkin_LIBRARIES} )
##################################################################################
add_executable(gt_publisher
    src/tools/node_gt_publisher.cpp
    )
target_link_libraries(gt_publisher
    ${catkin_LIBRARIES})
##################################################################################
add_executable(topic2file
    src/tools/node_odomtopic2file.cpp
    )
target_link_libraries(topic2file
    ${catkin_LIBRARIES})
##################################################################################
add_executable(odom2posestamp
    src/tools/node_odom2posestamp.cpp
    )
target_link_libraries(odom2posestamp
    ${catkin_LIBRARIES})
##################################################################################



