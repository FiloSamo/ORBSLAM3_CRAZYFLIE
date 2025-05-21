#ifndef DATA_GRABBER_HPP
#define DATA_GRABBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "include/System.h"  // Include the SLAM system header

#include <queue>
#include <mutex>
#include <memory>

class DataGrabber : public std::enable_shared_from_this<DataGrabber>
{
public:
    DataGrabber();
    DataGrabber(std::shared_ptr<ORB_SLAM3::System> pSLAM, bool bClahe, 
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
    std::shared_ptr<rclcpp::Node> ros_node,const std::string camera_frame_name);

    void grabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    cv::Mat getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg);

    void getImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    void virtual processData();
    void publishSE3fToOdom(const Sophus::SE3f& se3);

    std::queue<sensor_msgs::msg::Image::SharedPtr> img0Buf;
    std::queue<ORB_SLAM3::IMU::Point> imu0Buf;

    std::mutex mImuMutex;
    std::mutex mImgMutex;
    std::condition_variable mImuReady;
    std::condition_variable mImgReady;

    std::shared_ptr<ORB_SLAM3::System> mpSLAM;
    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    nav_msgs::msg::Odometry odom_msg_;
    std::shared_ptr<rclcpp::Node> rosNode_;
    const std::string tf_frame;
};

#endif // DATA_GRABBER_HPP