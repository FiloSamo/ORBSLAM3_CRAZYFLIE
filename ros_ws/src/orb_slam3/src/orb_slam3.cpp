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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <rclcpp/qos.hpp>
#include <rmw/types.h>  

#include "include/System.h"  // Include the SLAM system header

#include "orb_slam3/data_grabber.hpp"

#include <queue>
#include <mutex>
#include <thread>


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create a ROS node
    auto node = std::make_shared<rclcpp::Node>("example_slam");

    // Declare parameters for the node
    node->declare_parameter("config_path", "");
    node->declare_parameter("vocab_path", "");
    
    std::string config_path = node->get_parameter("config_path").as_string();
    std::string vocab_path = node->get_parameter("vocab_path").as_string();

    bool showPangolin = true ; // true If you want to spone the Pangolin window with pose estimation drawed
    bool bEqual = true; // true If you want to use CLAHE for image equalization

    // Publish odom message from SLAM
    auto odom_publ = node->create_publisher<nav_msgs::msg::Odometry>("orb_slam3/odom", 10);

    // Create SLAM system and DataGrabber //IMU_MONOCULAR
    auto SLAM = std::make_shared<ORB_SLAM3::System>(vocab_path, config_path, ORB_SLAM3::System::MONOCULAR, showPangolin); // Create SLAM system

    // DataGrabber object to handle image processing and SLAM
    auto igb = std::make_shared<DataGrabber>(SLAM, bEqual,  odom_publ, node, "oak-d_frame");

    // Creating Image subscription
    std::string imgTopicName = "/cam0/image_raw" ;

    // Creating Imu subscription
    std::string imuTopicName = "/imu0" ;

    // Subscribe to the camera image topic
    auto sub_img0 = node->create_subscription<custom_msgs::msg::ImageAndInt>(
        imgTopicName, 5, [igb](const custom_msgs::msg::ImageAndInt::SharedPtr msg) { igb->grabImage(msg); });

    // Subscribe to the camera image topic
    auto sub_imu0 = node->create_subscription<sensor_msgs::msg::Imu>(
        imuTopicName, 50, [igb](const sensor_msgs::msg::Imu::SharedPtr msg) { igb->getImu(msg); });
    
    // Start processing images in a separate thread
    std::thread image_thread(&DataGrabber::processData, igb);

    // Run the ROS node
    rclcpp::spin(node);
    std::cout << "Node stop to spinning!" << std::endl;

    // Shutdown the node and wait for the thread to complete
    rclcpp::shutdown();
    image_thread.join();

    return 0;
}