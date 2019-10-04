cmake_minimum_required(VERSION 2.8.3)
project(vio)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(OpenCV REQUIRED)

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
    )
find_package(OpenCV 3 REQUIRED)

#add_message_files(
#    FILES
#    )

generate_messages(
    DEPENDENCIES
    std_msgs
    )

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
    )



include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/include/"
    )


add_library(nodelet_icp
    src/node_vo.cpp
    src/salientpts.cpp
    src/icp.cpp
    src/tof_frame.cpp)
add_executable(gt_publisher
    src/node_gt_publisher.cpp
    )
add_executable(topic2file
    src/node_odomtopic2file.cpp
    )
add_executable(ekf
    src/node_ekf.cpp
    src/ekf_model.cpp
    )
add_executable(eskf
    src/node_eskf.cpp
    src/eskf_imu.cpp
    )

target_link_libraries(nodelet_icp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(gt_publisher ${catkin_LIBRARIES})
target_link_libraries(ekf ${catkin_LIBRARIES} )
target_link_libraries(eskf ${catkin_LIBRARIES} )
target_link_libraries(topic2file ${catkin_LIBRARIES})
