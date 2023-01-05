cmake_minimum_required(VERSION 2.8.11)
add_compile_options(-std=c++11)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    cv_bridge
    image_transport
    )
find_package(OpenCV REQUIRED)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/ 
    /opt/MVS/include/  
    include
    )
link_directories(
    /usr/lib/
    /opt/MVS/lib/64/
    )

add_executable(MvCameraControl src/Camera.cpp)   
target_link_libraries(
    MvCameraControl 
    /opt/MVS/lib/64/libMvCameraControl.so 
    pthread 
    ${OpenCV_LIBS} 
    ${catkin_LIBRARIES})