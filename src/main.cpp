#include "mineral_picker/header.h"

void Picker::onInit()
{
    img_subscriber_= nh_.subscribe("/hk_camera/image_raw", 1, &Picker::receiveFromCam,this);
//    img_subscriber_= nh_.subscribe("/image_rect", 1, &Picker::receiveFromCam,this);
    binary_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_binary_publisher", 1);
    segmentation_publisher_ = nh_.advertise<sensor_msgs::Image>("segmentation_publisher", 1);
    histed_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_histed_publisher", 1);
    camera_pose_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("camera_pose_publisher", 1);

    callback_ = boost::bind(&Picker::dynamicCallback, this, _1);
    server_.setCallback(callback_);
    K_ = (cv::Mat_<float>(3, 3) << 3556.40883, 0, 752.97577, 0, 3555.52262, 580.95345, 0, 0, 1);
}

void Picker::dynamicCallback(mineral_picker::dynamicConfig &config)
{
    clip_limit_ = config.clip_limit;
    morph_type_ = config.morph_type;
    morph_iterations_ = config.morph_iterations;
    min_area_thresh_=config.min_area_thresh;
    max_area_thresh_=config.max_area_thresh;
    lower_hsv_h_=config.lower_hsv_h;
    lower_hsv_s_=config.lower_hsv_s;
    lower_hsv_v_=config.lower_hsv_v;
    upper_hsv_h_=config.upper_hsv_h;
    upper_hsv_s_=config.upper_hsv_s;
    upper_hsv_v_=config.upper_hsv_v;
    morph_size_=config.morph_size;
    app_epsilon_=config.app_epsilon;
    angle_bias_=config.angle_bias;
    length_bias_=config.length_bias;
}


void Picker::receiveFromCam(const sensor_msgs::ImageConstPtr& image)
{
    cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
    imgProcess();
    segmentation_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),cv_image_->encoding , cv_image_->image).toImageMsg());
}

void Picker::claheProcess(cv::Mat * &hist_ptr)
{
    std::vector<cv::Mat> bgr_vec;
    cv::split(cv_image_->image,bgr_vec);
    auto clahe=cv::createCLAHE(clip_limit_);
    clahe->apply(bgr_vec[0],bgr_vec[0]);
    clahe->apply(bgr_vec[1],bgr_vec[1]);
    clahe->apply(bgr_vec[2],bgr_vec[2]);
    cv::merge(bgr_vec,*hist_ptr);
}

bool Picker::selectPoints(const std::vector<cv::Point2f> &centroid_vec,std::vector<cv::Point2f> &plane_points)
{
    auto distance_lam = [](const cv::Point2f &p1,const cv::Point2f &p2){return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));};
//    auto angle_lam = [](const cv::Point2f &p1,const cv::Point2f &p2,const cv::Point2f &p3,int angle_bias)
//            {
//                double dx1 = (p2.x - p1.x);
//                double dy1 = (p2.y - p1.y);
//                double dx2 = (p3.x - p1.x);
//                double dy2 = (p3.y - p1.y);
//                double angle_line = (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1 * dy1)*(dx2*dx2 + dy2 * dy2) + 1e-10);
//                double a = acos(angle_line) * 180 / 3.141592653;
//                return a>=90-angle_bias && a<=90+angle_bias;
//            };
    std::vector<std::pair<std::pair<cv::Point2f,cv::Point2f>,float>> points_distance_vec;
    for (int i=0;i<centroid_vec.size()-1;i++)
    {
        for (int j=i+1;j<centroid_vec.size();j++)
        {
            double distance=distance_lam(centroid_vec[i],centroid_vec[j]);
            points_distance_vec.emplace_back(std::make_pair(std::make_pair(centroid_vec[i],centroid_vec[j]),distance));
        }
        std::sort(points_distance_vec.begin(),points_distance_vec.end(),
                  [&](std::pair<std::pair<cv::Point2f,cv::Point2f>,float> &a,std::pair<std::pair<cv::Point2f,cv::Point2f>,float> &b){return a.second < b.second;});
//        if (int(points_distance_vec[1].second-points_distance_vec[0].second)<=length_bias_ &&
//        angle_lam(points_distance_vec[0].first.first,points_distance_vec[0].first.second,points_distance_vec[1].first.second,angle_bias_))
        if (int(points_distance_vec[1].second-points_distance_vec[0].second)<=length_bias_)
        {
            plane_points.emplace_back(points_distance_vec[0].first.first);
            plane_points.emplace_back(points_distance_vec[0].first.second);
            plane_points.emplace_back(points_distance_vec[1].first.second);

            cv::circle(cv_image_->image, points_distance_vec[0].first.first, 8, cv::Scalar(255, 255, 0), 4);
            cv::circle(cv_image_->image, points_distance_vec[0].first.second, 8, cv::Scalar(255, 255, 0), 4);
            cv::circle(cv_image_->image, points_distance_vec[1].first.second, 8, cv::Scalar(255, 255, 0), 4);
            break;
        }
    }
}

void Picker::imgProcess()
{
    //segementation
    cv::Mat mor_img;
    auto * hist_ptr=new cv::Mat ();
    auto * hsv_ptr=new cv::Mat();
    auto * binary_ptr=new cv::Mat();
    claheProcess(hist_ptr);
    cv::cvtColor(*hist_ptr,*hsv_ptr,cv::COLOR_BGR2HSV);
    histed_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),cv_image_->encoding , *hist_ptr).toImageMsg());
    delete hist_ptr;
    cv::inRange(*hsv_ptr,cv::Scalar(lower_hsv_h_,lower_hsv_s_,lower_hsv_v_),cv::Scalar(upper_hsv_h_,upper_hsv_s_,upper_hsv_v_),*binary_ptr);
    delete hsv_ptr;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1+2*morph_size_, 1+2*morph_size_), cv::Point(-1, -1));
    cv::morphologyEx(*binary_ptr,mor_img,morph_type_,kernel,cv::Point(-1,-1),morph_iterations_);
    delete binary_ptr;
    binary_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , mor_img).toImageMsg());

    //contours process
    std::vector< std::vector< cv::Point> > contours;
    cv::findContours(mor_img,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point2f> centroid_vec;
    for (int i = 0; i < contours.size(); i++)
    {
        std::vector<cv::Point2i> hull;
        cv::convexHull(contours[i], hull, true);
        if (cv::contourArea(hull) >= min_area_thresh_ && cv::contourArea(hull) <= max_area_thresh_)
        {
            // rotate rectangule blue
//            auto rotate_rect = cv::minAreaRect(hull);
//            auto* touchVertices = new cv::Point2f[4];
//            rotate_rect.points(touchVertices);
//            for (int j = 0; j < 4; ++j)
//            {
//                cv::line(cv_image_->image, touchVertices[j % 4], touchVertices[(j + 1) % 4], cv::Scalar(255,0,0), 2);
//            }
            std::vector<cv::Point2f> app_points_vec;
            cv::approxPolyDP(hull,app_points_vec,app_epsilon_,true);

            if (app_points_vec.size() == 3 || app_points_vec.size() == 4) // approx poly points should be 3 or 4
            {
                // approx shape points red

                for (auto &point : app_points_vec)
                {
                    cv::circle(cv_image_->image,point,8,cv::Scalar(0,0,255),2);
                }

                auto moment = cv::moments(hull);
                double hu_moment[7];
                cv::HuMoments(moment, hu_moment);
                int cx = int(moment.m10 / moment.m00);
                int cy = int(moment.m01 / moment.m00);
                cv::Point2f centroid(cx, cy);
                centroid_vec.emplace_back(centroid);

                // centroid and polylines green
                cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
//                cv::circle(cv_image_->image, middle_point, 2, cv::Scalar(0, 255, 0), 2);
            }
            else continue;
        }
    }
    // now select the plane 4 points
    std::vector<cv::Point2f> plane_points;
    if (!centroid_vec.empty()) selectPoints(centroid_vec,plane_points);
    else std::cout<<"can not detect target"<<std::endl;
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
