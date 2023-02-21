#include "mineral_picker/header.h"

std::queue<std::vector<cv::DMatch>> g_matches_buff;
std::queue<std::vector<cv::KeyPoint>> g_prev_keypoints_buff;
std::queue<std::vector<cv::KeyPoint>> g_cur_keypoints_buff;
//std::queue<cv::Mat> g_R_buff;
//std::queue<cv::Mat> g_t_buff;
//std::queue<std::vector<cv::Point2f>> g_points_vec_buff;
std::mutex mutex_buff;

int g_thread_num=0;

void Picker::onInit()
{
    img_subscriber_= nh_.subscribe("/image_rect", 1, &Picker::receiveFromCam,this);
    binary_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_binary_publisher", 1);
    prev_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_prev_publisher", 1);
    cur_publisher_ = nh_.advertise<sensor_msgs::Image>("picker_cur_publisher", 1);
    callback_ = boost::bind(&Picker::dynamicCallback, this, _1);
    server_.setCallback(callback_);
    initial_= true;
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

    time_t start,end;
    start=clock();
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
        end=clock();
        std::cout<<"orb time: "<<(end-start)/1000<<" ms"<<std::endl;
        if (matches_.size()>=min_matches_)
        {
            mutex_buff.lock();
            if (g_matches_buff.size()>=5)
            {
                std::cout<<"queue larger than 5,pop now"<<std::endl;
                g_prev_keypoints_buff.pop();
                g_cur_keypoints_buff.pop();
                g_matches_buff.pop();

                g_prev_keypoints_buff.push(prev_keypoint_vec_);
                g_cur_keypoints_buff.push(cur_keypoint_vec_);
                g_matches_buff.push(matches_);
                std::cout<<"buff restructure finish"<<std::endl;
            }
            else
            {
                g_prev_keypoints_buff.push(prev_keypoint_vec_);
                g_cur_keypoints_buff.push(cur_keypoint_vec_);
                g_matches_buff.push(matches_);
                std::cout<<"queue size now:"<<g_matches_buff.size()<<std::endl;
            }
            mutex_buff.unlock();
        }
        else std::cout<<"too few matching points"<<std::endl;
    }
    else std::cout<<"track fail,there is not enough feature points in current frame"<<std::endl;
}

//cv::Point2f pixel2cam(const cv::Point2f &p, const cv::Mat &K)
//{
//    return {(p.x - K.at<float>(0, 2)) / K.at<float>(0, 0),(p.y - K.at<float>(1, 2)) / K.at<float>(1, 1)};
//}


//void Picker::pose_estimation_2d2d()
//{
//
//    cv::Mat R,t;
//    //相机内参矩阵
////    cv::Mat K=(cv::Mat_<double>(3,3)<<520.9,0,325.1,0,521,0,249.7,0,0,1);
//
//    //把匹配点转换为vector<point2f>的形式
//    std::vector<cv::Point2f> points1;
//    std::vector<cv::Point2f> points2;
//
//    for(int i=0;i<(int)matches_.size();i++)
//    {
//        points1.push_back(prev_keypoint_vec_[matches_[i].queryIdx].pt);
//        points2.push_back(cur_keypoint_vec_[matches_[i].queryIdx].pt);
//    }
//
//    //计算基础矩阵F
////    cv::Mat fundamental_matrix;
////    fundamental_matrix=cv::findFundamentalMat(points1,points2,cv::FM_8POINT);
////    std::cout << "FUNDAMENTAL_MATRIX" <<std::endl<<fundamental_matrix<<std::endl;
//
//    //计算本质矩阵
//    cv::Point2d principal_point(325.1,249.7);//相机光心 标定值
//    double focal_lengh=521 ;//相机焦距
//    cv::Mat essential_matrix;
//    essential_matrix=cv::findEssentialMat(points1,points2,focal_lengh,principal_point);
////    std::cout<<"essential_matrix is"<<std::endl<<essential_matrix<<std::endl;
//
//    //计算单应性矩阵
//    //本列非平面
////    cv::Mat homography_matrix;
////    homography_matrix=cv::findHomography(points1,points2,cv::RANSAC,3);
////    std::cout<<"homography_matrix is"<<std::endl<<homography_matrix<<std::endl;
//
//    //从本质矩阵恢复旋转，平移
//    cv::recoverPose(essential_matrix,points1,points2,R,t,focal_lengh,principal_point);
//    std::cout<<"R :"<<R<<std::endl;
//    std::cout<<"t is"<<t<<std::endl;
//
//}

void pose_estimation_2d2d()
{
    g_thread_num++;

    if (!g_matches_buff.empty())
    {
        std::cout<<"pose estimation now"<<std::endl;
        time_t start,end;
        start=clock();
        cv::Mat R,t;
        //相机内参矩阵
//    cv::Mat K=(cv::Mat_<double>(3,3)<<520.9,0,325.1,0,521,0,249.7,0,0,1);

        //把匹配点转换为vector<point2f>的形式
        std::vector<cv::Point2f> points1;
        std::vector<cv::Point2f> points2;
        mutex_buff.lock();
        auto matches=g_matches_buff.front();
        g_matches_buff.pop();

        auto prev_keypoint_vec=g_prev_keypoints_buff.front();
        g_prev_keypoints_buff.pop();
        auto cur_keypoint_vec=g_cur_keypoints_buff.front();
        g_cur_keypoints_buff.pop();

        mutex_buff.unlock();

        for(int i=0;i<(int)matches.size();i++)
        {
            points1.push_back(prev_keypoint_vec[matches[i].queryIdx].pt);
            points2.push_back(cur_keypoint_vec[matches[i].queryIdx].pt);
        }

        //计算基础矩阵F
//    cv::Mat fundamental_matrix;
//    fundamental_matrix=cv::findFundamentalMat(points1,points2,cv::FM_8POINT);
//    std::cout << "FUNDAMENTAL_MATRIX" <<std::endl<<fundamental_matrix<<std::endl;

        //计算本质矩阵
        cv::Point2d principal_point(325.1,249.7);//相机光心 标定值
        double focal_lengh=521 ;//相机焦距
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
        end=clock();
        std::cout<<"estimation time: "<<(end-start)/1000<<" ms"<<std::endl;

        

//        mutex_buff.lock();
//        g_R_buff.push(R);
//        g_t_buff.push(t);
//        mutex_buff.unlock();

    }
//    if (g_asyc_status!=std::future_status::ready) g_asyc_status=std::future_status::ready;
    g_thread_num--;
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
    std::vector<cv::Point2f> mineral_points_vec;
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
            mineral_points_vec.push_back(middle_point);
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

        if (g_thread_num<g_matches_buff.size())
        {
            std::thread (pose_estimation_2d2d).detach();
            std::cout<<g_thread_num<<std::endl;
            std::chrono::milliseconds dura(1);
            std::this_thread::sleep_for(dura);
        }
    }

}
