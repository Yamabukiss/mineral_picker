#pragma once

#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "algorithm"
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "dynamic_reconfigure/server.h"
#include "mineral_picker/dynamicConfig.h"
#include"iostream"
#include"fstream"
#include "thread"
#include "mutex"
#include "future"
#include "chrono"
#include "ctime"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

class Picker
{
public:
    void onInit();
    void receiveFromCam(const sensor_msgs::ImageConstPtr &image);
    void dynamicCallback(mineral_picker::dynamicConfig& config);
    void featureTracker();
    void imgProcess();
    cv::Point2f pixel2cam(const cv::Point2f &p);
    void pose_estimation_2d2d();
    void triangulation(const cv::Mat &R,const cv::Mat &t);

    ros::NodeHandle nh_;
    cv_bridge::CvImagePtr cv_image_;
    ros::Subscriber img_subscriber_;
    ros::Publisher binary_publisher_;
    ros::Publisher prev_publisher_;
    ros::Publisher cur_publisher_;
    ros::Publisher camera_pose_publisher_;
    dynamic_reconfigure::Server<mineral_picker::dynamicConfig> server_;
    dynamic_reconfigure::Server<mineral_picker::dynamicConfig>::CallbackType callback_;

    int morph_type_;
    int morph_iterations_;
    int lower_hsv_h_;
    int lower_hsv_s_;
    int lower_hsv_v_;
    int upper_hsv_h_;
    int upper_hsv_s_;
    int upper_hsv_v_;
    int min_features_;
    int min_matches_;
    cv::Mat K_;
    double min_area_thresh_;
    bool initial_;
    cv::Mat prev_img_;
    cv::Mat cur_img_;
    cv::Mat prev_descriptor_;
    cv::Mat cur_descriptor_;
    std::vector<cv::KeyPoint> prev_keypoint_vec_;
    std::vector<cv::KeyPoint> cur_keypoint_vec_;
    std::vector<cv::DMatch> matches_;
};