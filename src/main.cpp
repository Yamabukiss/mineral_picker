#include "mineral_picker/header.h"

void Picker::onInit()
{
    img_subscriber_= nh_.subscribe("/hk_camera/image_raw", 1, &Picker::receiveFromCam,this);
//    img_subscriber_= nh_.subscribe("/image_rect", 1, &Picker::receiveFromCam,this);
    hsv_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_hsv_publisher", 1);
    masked_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_masked_publisher", 1);
    segmentation_publisher_ = nh_.advertise<sensor_msgs::Image>("segmentation_publisher", 1);
    camera_pose_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("camera_pose_publisher", 1);

    callback_ = boost::bind(&Picker::dynamicCallback, this, _1);
    server_.setCallback(callback_);
    K_ = (cv::Mat_<float>(3, 3) << 3556.40883, 0, 752.97577, 0, 3555.52262, 580.95345, 0, 0, 1);
    cv::Mat temp_triangle_=cv::imread("/home/yamabuki/detect_ws/src/mineral_picker/temp_triangle.png",cv::IMREAD_GRAYSCALE);
    cv::Mat temp_rectangle_=cv::imread("/home/yamabuki/detect_ws/src/mineral_picker/temp_rectangle.png",cv::IMREAD_GRAYSCALE);

    cv::Mat binary_1,binary_2;

    cv::threshold(temp_triangle_,binary_1,0, 255, cv::THRESH_BINARY_INV + cv::THRESH_OTSU);
    cv::threshold(temp_rectangle_,binary_2,0, 255, cv::THRESH_BINARY_INV + cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> rect_temp_contour;
    std::vector<std::vector<cv::Point>> tri_temp_contour;
    std::vector<cv::Point> rect_hull;
    std::vector<cv::Point> tri_hull;

    cv::findContours(binary_1,tri_temp_contour,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    cv::convexHull(tri_temp_contour[0],tri_hull, true);
    temp_triangle_hull_=tri_hull;

    cv::findContours(binary_2,rect_temp_contour,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    std::sort(rect_temp_contour.begin(),rect_temp_contour.end(),[](const std::vector<cv::Point>& contour1,const std::vector<cv::Point>& contour2){return contour1.size()>contour2.size();});
    cv::convexHull(rect_temp_contour[0],rect_hull, true);
    temp_rectangle_hull_=rect_hull;
    std::cout<<"temp init finished"<<std::endl;
}

void Picker::dynamicCallback(mineral_picker::dynamicConfig &config)
{
    morph_type_ = config.morph_type;
    morph_iterations_ = config.morph_iterations;
    lower_hsv_h_=config.lower_hsv_h;
    lower_hsv_s_=config.lower_hsv_s;
    lower_hsv_v_=config.lower_hsv_v;
    upper_hsv_h_=config.upper_hsv_h;
    upper_hsv_s_=config.upper_hsv_s;
    upper_hsv_v_=config.upper_hsv_v;
    morph_size_=config.morph_size;
    save_on_=config.save_on;
    moment_bias_=config.moment_bias;
    approx_epsilon_=config.approx_epsilon;
}


void Picker::receiveFromCam(const sensor_msgs::ImageConstPtr& image)
{
    cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
    imgProcess();
    segmentation_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),cv_image_->encoding , cv_image_->image).toImageMsg());
//    getTemplateImg();
}

void Picker::getTemplateImg()
{
        cv::Mat gray_img;
        cv::cvtColor(cv_image_->image,gray_img,CV_BGR2GRAY);
        segmentation_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , gray_img).toImageMsg());
    if (save_on_)
    {
        cv::imwrite("/home/yamabuki/detect_ws/src/mineral_picker/temp_img.jpg",gray_img);
    }
}

void Picker::imgProcess()
{
    //segementation
    auto * mor_ptr=new cv::Mat();
    auto * hsv_ptr=new cv::Mat();
    auto * binary_ptr=new cv::Mat();
    auto * mask_ptr=new cv::Mat();
    cv::cvtColor(cv_image_->image,*hsv_ptr,cv::COLOR_BGR2HSV);
    cv::inRange(*hsv_ptr,cv::Scalar(lower_hsv_h_,lower_hsv_s_,lower_hsv_v_),cv::Scalar(upper_hsv_h_,upper_hsv_s_,upper_hsv_v_),*binary_ptr);
    delete hsv_ptr;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1+2*morph_size_, 1+2*morph_size_), cv::Point(-1, -1));
    cv::morphologyEx(*binary_ptr,*mor_ptr,morph_type_,kernel,cv::Point(-1,-1),morph_iterations_);
    delete binary_ptr;
    hsv_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , *mor_ptr).toImageMsg());

    // hsv contours process
    auto * contours_ptr = new std::vector< std::vector< cv::Point> >();
    cv::findContours(*mor_ptr,*contours_ptr,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    auto * blank_mask_ptr= new cv::Mat();
    *blank_mask_ptr=cv::Mat::zeros(cv_image_->image.rows,cv_image_->image.cols,CV_8UC1);
    for (auto &contours : *contours_ptr)
    {
        std::vector<cv::Point2i> hull;
        cv::convexHull(contours, hull, true);
        cv::fillConvexPoly(*blank_mask_ptr,hull,cv::Scalar(255));
    }
    delete contours_ptr;

    cv::bitwise_xor(*mor_ptr,*blank_mask_ptr,*mask_ptr);
    masked_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , *mask_ptr).toImageMsg());
    delete mor_ptr;
    delete blank_mask_ptr;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(*mask_ptr,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    delete mask_ptr;
    for (auto &contour : contours)
    {
        std::vector<cv::Point2i> hull;
        cv::convexHull(contour, hull, true);
        auto moment = cv::moments(hull);
        double hu_moment[7];
        cv::HuMoments(moment, hu_moment);
        if (cv::matchShapes(hull,temp_triangle_hull_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_ || cv::matchShapes(hull,temp_rectangle_hull_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
        {
            std::vector<cv::Point2i> approx_points;
            cv::approxPolyDP(hull,approx_points, approx_epsilon_,true);
            if (approx_points.size()==3 || approx_points.size()==4)
            {

                for (auto &applox_point : approx_points) cv::circle(cv_image_->image,applox_point,8,cv::Scalar(0,0,255),3);
                int cx = int(moment.m10 / moment.m00);
                int cy = int(moment.m01 / moment.m00);
                cv::Point2f centroid(cx, cy);

                // centroid and polylines green
                cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
                cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
            }
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
