#include "test_depth.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>



#include <iostream>
#include <fstream>


using namespace std;
using namespace cv;




test_depth::test_depth(ros::NodeHandle nh)
        : _nh(nh),
        counter(true)
{

}

bool test_depth::init()
{
        _sub_cameraInfo = _nh.subscribe("/camera/rgb/camera_info", 1, &test_depth::disparityImageCallBack, this);
        _sub_disparity = _nh.subscribe("/gazebo_sim/disparity", 1, &test_depth::realDisparityCallBack, this);
        _pub_camInfo = _nh.advertise<sensor_msgs::CameraInfo>("/gazebo_sim/camera_info", 10);
        return true;
}


void test_depth::disparityImageCallBack(const sensor_msgs::CameraInfo& msg)
{
        sensor_msgs::CameraInfo temp = msg;

        float baseLine = 0.2;
        float Tx = 0.0;
        float Fx = temp.P[0];
        Tx = -1 * Fx * baseLine;
        temp.P[3] = Tx;
        _pub_camInfo.publish(temp);
}

void test_depth::realDisparityCallBack(const stereo_msgs::DisparityImage::ConstPtr& msg)
{
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
                boost::shared_ptr<void const> tracked_object;

                cv::Mat image = cv_bridge::toCvShare(msg->image, tracked_object, "32FC1")->image;
                // cv::Mat temp;
                // image.convertTo(temp, CV_8UC1, 1);
                //
                // imshow("received disp msg", temp);



                waitKey(10);
        } catch( ...) {
                ROS_INFO("hello");
        }

}
