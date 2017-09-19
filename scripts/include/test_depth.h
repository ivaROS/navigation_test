#ifndef TEST_DEPTH_H
#define TEST_DEPTH_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/static_transform_broadcaster.h>



class test_depth
{

        ros::NodeHandle _nh;
        ros::Subscriber _sub_cameraInfo;
        ros::Subscriber _sub_disparity;
        ros::Publisher _pub_camInfo;



public:
        test_depth(ros::NodeHandle nh);
        void disparityImageCallBack(const sensor_msgs::CameraInfo& msg);
        void realDisparityCallBack(const stereo_msgs::DisparityImage::ConstPtr& msg);

        bool init();
        bool counter;
protected:

};

#endif /*TEST_H */
