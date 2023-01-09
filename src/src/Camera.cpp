#include "Camera.hpp"
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char *argv[])
{
    cv::Mat src, resize_src;
    ros::init(argc, argv, "hik_cam_capture");
    ros::NodeHandle nh("~");
    camera::Camera MVS_cap(nh);
    int pub_img_height;
    int pub_img_width;   
    string image_topic;
    string frame_id;

    nh.getParam("image_topic",image_topic);
    nh.getParam("frame_id",frame_id);
    nh.getParam("frame_id",frame_id);
    nh.getParam("pub_img_height",pub_img_height);
    nh.getParam("pub_img_width",pub_img_width);
    
    image_transport::ImageTransport cam_image(nh);
    image_transport::CameraPublisher img_pub = cam_image.advertiseCamera(image_topic, 1);
    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::RGB8;
    ros::Rate loop_rate(30);
    int empty = 0;
    int count = 0;
    while (nh.ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        MVS_cap.ReadImg(src);
    	cv::resize(src, resize_src, cv::Size(pub_img_width, pub_img_height), 0, 0, cv::INTER_AREA);
         cv::imwrite("/home/srr/calib_file/images/"+to_string(count)+".jpg", resize_src);
        if (resize_src.empty())
        {
            cout << "empty" << endl;
            continue;
        }
        cv_ptr->image = resize_src;
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();
        image_msg.header.frame_id = frame_id;
        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp = image_msg.header.stamp;
        img_pub.publish(image_msg, camera_info_msg);
	count++;
    }

    return 0;
}
