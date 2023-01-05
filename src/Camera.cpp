#include "Camera.hpp"
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;



int main(int argc,char *argv[]){
    cv::Mat src;
    ros::init(argc,argv,"hik_cam");
    ros::NodeHandle nh;
    camera::Camera MVS_cap(nh); 
    image_transport::ImageTransport cam_image(nh);
    image_transport::CameraPublisher img_pub = cam_image.advertiseCamera("/hik_cam/image",1000);
    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;    
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::RGB8;
    ros::Rate loop_rate(1);
    
    while(nh.ok()){
        loop_rate.sleep();
        ros::spinOnce();
        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            cout << "empty"<< endl;
            continue;
        }
        cv_ptr->image = src;
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();
        image_msg.header.frame_id = "hik_cam";
        camera_info_msg.header.frame_id = image_msg.header.frame_id;
	    camera_info_msg.header.stamp = image_msg.header.stamp;
        img_pub.publish(image_msg, camera_info_msg);
        
    }


    return 0;
}