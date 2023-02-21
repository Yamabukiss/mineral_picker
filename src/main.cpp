#include "mineral_picker/header.h"

void Picker::onInit()
{
    img_subscriber_= nh_.subscribe("/hk_camera/image_raw", 1, &Picker::receiveFromCam,this);
    binary_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_binary_publisher", 1);
    prev_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_prev_publisher", 1);
    cur_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_cur_publisher", 1);
    camera_pose_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("camera_pose_publisher", 1);

    callback_ = boost::bind(&Picker::dynamicCallback, this, _1);
    server_.setCallback(callback_);
    initial_= true;
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
    min_features_=config.min_features;
    min_matches_=config.min_matches;
}


void Picker::receiveFromCam(const sensor_msgs::ImageConstPtr& image)
{
    cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
    cur_img_=cv_image_->image.clone();
    imgProcess();
}

void Picker::featureTracker()
{

    std::vector<cv::Point2f> corners_vec;
    auto orb=cv::ORB::create(min_features_);
    auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch> matches_vec;
    orb->detectAndCompute(prev_img_,cv::Mat(),prev_keypoint_vec_,prev_descriptor_);
    orb->detectAndCompute(cur_img_,cv::Mat(),cur_keypoint_vec_,cur_descriptor_);
    cv::drawKeypoints(prev_img_,prev_keypoint_vec_,prev_img_);
    cv::drawKeypoints(cur_img_,cur_keypoint_vec_,cur_img_);
    if (cur_descriptor_.cols==prev_descriptor_.cols)
    {
        matcher->match(prev_descriptor_, cur_descriptor_, matches_vec);
        // sift process
        double min_hamm_dist = 10000, max_hamm_dist = 0;

        // find min and max hamming distance between two descriptors
        for (int i = 0; i < prev_descriptor_.rows; i++)
        {
            double dist = matches_vec[i].distance;
            if (dist < min_hamm_dist) min_hamm_dist = dist;
            if (dist > max_hamm_dist) max_hamm_dist = dist;
        }

        // sift the match successed points 30 is selected by experience
        for (int i = 0; i < prev_descriptor_.rows; i++)
        {
            if (matches_vec[i].distance <= std::max(2 * min_hamm_dist, 30.0))
            {
                matches_.push_back(matches_vec[i]);
            }
        }
        if (matches_.size()>=min_matches_)
        {
            pose_estimation_2d2d();
        }
        else std::cout<<"too few matching points"<<std::endl;

    }

    else std::cout<<"track fail,there is not enough feature points in current frame"<<std::endl;
}

cv::Point2f Picker::pixel2cam(const cv::Point2f &p)
{
    return {(p.x - K_.at<float>(0, 2)) / K_.at<float>(0, 0),(p.y - K_.at<float>(1, 2)) / K_.at<float>(1, 1)};
}

void Picker::triangulation(const cv::Mat &R, const cv::Mat &t)
{
    cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    std::vector<cv::Point2f> pts_1, pts_2;
    for (auto m:matches_) {
        pts_1.push_back(pixel2cam(prev_keypoint_vec_[m.queryIdx].pt));
        pts_2.push_back(pixel2cam(cur_keypoint_vec_[m.trainIdx].pt));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
    std::vector<cv::Point3d> points_3d;
    for (int i = 0; i < pts_4d.cols; i++) {
        cv::Mat x = pts_4d.col(i);
        cv::Point3d p(
                x.at<float>(0, 0),
                x.at<float>(1, 0),
                x.at<float>(2, 0)
        );
        points_3d.push_back(p);
    }
}



void Picker::pose_estimation_2d2d()
{

    cv::Mat R,t;
    //相机内参矩阵
//    cv::Mat K=(cv::Mat_<double>(3,3)<<520.9,0,325.1,0,521,0,249.7,0,0,1);

    //把匹配点转换为vector<point2f>的形式
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    for(int i=0;i<(int)matches_.size();i++)
    {
        points1.push_back(prev_keypoint_vec_[matches_[i].queryIdx].pt);
        points2.push_back(cur_keypoint_vec_[matches_[i].queryIdx].pt);
    }

    //计算基础矩阵F
//    cv::Mat fundamental_matrix;
//    fundamental_matrix=cv::findFundamentalMat(points1,points2,cv::FM_8POINT);
//    std::cout << "FUNDAMENTAL_MATRIX" <<std::endl<<fundamental_matrix<<std::endl;

    //计算本质矩阵
    cv::Point2d principal_point(753,581);//相机光心 标定值
    double focal_lengh=3556 ;//相机焦距
    cv::Mat essential_matrix;
    essential_matrix=cv::findEssentialMat(points1,points2,focal_lengh,principal_point);
//    std::cout<<"essential_matrix is"<<std::endl<<essential_matrix<<std::endl;

    //计算单应性矩阵
    //本列非平面
//    cv::Mat homography_matrix;
//    homography_matrix=cv::findHomography(points1,points2,cv::RANSAC,3);
//    std::cout<<"homography_matrix is"<<std::endl<<homography_matrix<<std::endl;

    //从本质矩阵恢复旋转，平移
    cv::recoverPose(essential_matrix,points1,points2,R,t,focal_lengh,principal_point);
    std::cout<<"R :"<<R<<std::endl;
    std::cout<<"t is"<<t<<std::endl;
    tf::Matrix3x3 tf_rotate_matrix(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                                   R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                                   R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    tf::Vector3 tf_tvec(t.at<double>(0,0), t.at<double>(0,1), t.at<double>(0,2));


//    triangulation(R,t);
    tf::Quaternion quaternion;
    double r;
    double p;
    double y;
    static geometry_msgs::TwistStamped  test;

    test.twist.linear.x=t.at<double>(0,0);
    test.twist.linear.y=t.at<double>(0,1);
    test.twist.linear.z=t.at<double>(0,2);
    tf_rotate_matrix.getRPY(r, p, y);
    quaternion.setRPY(y, p, r);
    test.twist.angular.x=r;
    test.twist.angular.y=p;
    test.twist.angular.z=y;
    camera_pose_publisher_.publish(test);
    tf::Transform transform;
    transform.setRotation(quaternion);
    transform.setOrigin(tf_tvec);
    tf::StampedTransform stamped_Transfor(transform, ros::Time::now(), "camera_optional_frame","camera");
    static tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(stamped_Transfor);
}


void Picker::imgProcess()
{
    //segementation
    cv::Mat hsv_img,binary_img,mor_img;
    cv::cvtColor(cur_img_,hsv_img,cv::COLOR_BGR2HSV);
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
            cv::polylines(cur_img_, hull, true, cv::Scalar(0, 199, 140), 7);
            cv::circle(cur_img_, middle_point, 3, cv::Scalar(0, 199, 140), 7);
        }
    }
    if (!initial_)
    {
        featureTracker();
        if (!matches_.empty()) matches_.clear();
        prev_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),cv_image_->encoding , prev_img_).toImageMsg());
        cur_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),cv_image_->encoding , cur_img_).toImageMsg());
        prev_img_=cur_img_;
        prev_keypoint_vec_=cur_keypoint_vec_;
        prev_descriptor_=cur_descriptor_;
    }

    else
    {
        prev_img_=cur_img_;
        prev_keypoint_vec_=cur_keypoint_vec_;
        prev_descriptor_=cur_descriptor_;
        initial_= false;
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
