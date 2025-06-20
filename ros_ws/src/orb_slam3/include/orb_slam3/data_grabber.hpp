/*
 *    __________  _____ ____     __  __      _ __        
 *   / ____/ __ \/ ___// __ \   / / / /___  (_) /_  ____ 
 *  / /   / /_/ /\__ \/ /_/ /  / / / / __ \/ / __ \/ __ \
 * / /___/ ____/___/ / ____/  / /_/ / / / / / /_/ / /_/ /
 * \____/_/    /____/_/       \____/_/ /_/_/_.___/\____/ 
 *                                                       
 *                                                               
 * Authors: Filippo Samorì, Filippo Ugolini and Daniele Crivellari
 * 20/06/2025
 * University of Bologna, Italy
 * License: BSD-3-Clause
 */ 

#ifndef DATA_GRABBER_HPP
#define DATA_GRABBER_HPP

// ROS2 core and message includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "custom_msgs/msg/image_and_int.hpp"

#include "include/System.h"  // Include the SLAM system header

#include <queue>
#include <mutex>
#include <memory>

// DataGrabber class: Handles image and IMU data acquisition, buffering, and processing for ORB_SLAM3
class DataGrabber : public std::enable_shared_from_this<DataGrabber>
{
public:
    // Default constructor
    DataGrabber();

    // Main constructor
    // pSLAM: pointer to the ORB_SLAM3 system
    // bClahe: enable/disable CLAHE image enhancement
    // rospub: ROS publisher for odometry
    // ros_node: ROS node pointer
    // camera_frame_name: name of the camera frame for TF
    DataGrabber(std::shared_ptr<ORB_SLAM3::System> pSLAM, bool bClahe, 
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
        std::shared_ptr<rclcpp::Node> ros_node, const std::string camera_frame_name);

    // Callback to grab image messages
    void grabImage(const custom_msgs::msg::ImageAndInt::SharedPtr msg);

    // Convert ROS image message to OpenCV Mat
    cv::Mat getImage(const custom_msgs::msg::ImageAndInt::SharedPtr &img_msg);

    // Callback to grab IMU messages
    void getImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    // Main processing loop (can be overridden)
    void virtual processData();

    // Publish SLAM pose as nav_msgs/Odometry
    void publishSE3fToOdom(const Sophus::SE3f& se3, int timestamp);

    // Buffers for incoming image and IMU data
    std::queue<custom_msgs::msg::ImageAndInt::SharedPtr> img0Buf;
    std::queue<ORB_SLAM3::IMU::Point> imu0Buf;

    // Mutexes and condition variables for thread safety and synchronization
    std::mutex mImuMutex;
    std::mutex mImgMutex;
    std::condition_variable mImuReady;
    std::condition_variable mImgReady;

    // Pointer to the ORB_SLAM3 system
    std::shared_ptr<ORB_SLAM3::System> mpSLAM;

    // Flag to enable CLAHE image enhancement
    const bool mbClahe;

    // CLAHE object for image contrast enhancement
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    
    // ROS publisher and message for odometry
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    nav_msgs::msg::Odometry odom_msg_;

    // ROS node pointer
    std::shared_ptr<rclcpp::Node> rosNode_;

    // Name of the TF frame for the camera
    const std::string tf_frame;
};

#endif // DATA_GRABBER_HPP