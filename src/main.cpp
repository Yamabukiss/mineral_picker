#include "mineral_picker/header.h"

void Picker::onInit()
{
//    img_subscriber_= nh_.subscribe("/hk_camera/image_raw", 1, &Picker::receiveFromCam,this);
    img_subscriber_= nh_.subscribe("/image_rect", 1, &Picker::receiveFromCam,this);
    binary_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_binary_publisher", 1);
    segmentation_publisher_ = nh_.advertise<sensor_msgs::Image>("segmentation_publisher", 1);
    camera_pose_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("camera_pose_publisher", 1);

    callback_ = boost::bind(&Picker::dynamicCallback, this, _1);
    server_.setCallback(callback_);
    K_ = (cv::Mat_<float>(3, 3) << 3556.40883, 0, 752.97577, 0, 3555.52262, 580.95345, 0, 0, 1);
}

void Picker::dynamicCallback(mineral_picker::dynamicConfig &config)
{
    morph_type_ = config.morph_type;
    morph_iterations_ = config.morph_iterations;
    min_area_thresh_=config.min_area_thresh;
    lower_hsv_h_=config.lower_hsv_h;
    lower_hsv_s_=config.lower_hsv_s;
    lower_hsv_v_=config.lower_hsv_v;
    upper_hsv_h_=config.upper_hsv_h;
    upper_hsv_s_=config.upper_hsv_s;
    upper_hsv_v_=config.upper_hsv_v;
}


void Picker::receiveFromCam(const sensor_msgs::ImageConstPtr& image)
{
    cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
    imgProcess();
    segmentation_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),cv_image_->encoding , cv_image_->image).toImageMsg());
}

void Picker::imgProcess()
{
    //segementation
    cv::Mat hsv_img,binary_img,mor_img;
    cv::cvtColor(cv_image_->image,hsv_img,cv::COLOR_BGR2HSV);
    cv::inRange(hsv_img,cv::Scalar(lower_hsv_h_,lower_hsv_s_,lower_hsv_v_),cv::Scalar(upper_hsv_h_,upper_hsv_s_,upper_hsv_v_),binary_img);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(binary_img,mor_img,morph_type_,kernel,cv::Point(-1,-1),morph_iterations_);
    binary_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , mor_img).toImageMsg());

    //contours process
    std::vector< std::vector< cv::Point> > contours;
    std::vector< std::vector< cv::Point> > mask_contours;
    cv::findContours(mor_img,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); i++)
    {
        std::vector<cv::Point2i> hull;
        cv::convexHull(contours[i], hull, true);
        if (cv::contourArea(hull) > min_area_thresh_)
        {
            auto moment = cv::moments(hull);
            double hu_moment[7];
            cv::HuMoments(moment, hu_moment);
            int cx = int(moment.m10 / moment.m00);
            int cy = int(moment.m01 / moment.m00);
            cv::Point2i middle_point(cx, cy);
            cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 199, 140), 7);
            cv::circle(cv_image_->image, middle_point, 3, cv::Scalar(0, 199, 140), 7);
        }
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "mineral_picker_node");
    Picker picker;
    picker.onInit();
    while (ros::ok())
    {
        ros::spinOnce();
    }

}
