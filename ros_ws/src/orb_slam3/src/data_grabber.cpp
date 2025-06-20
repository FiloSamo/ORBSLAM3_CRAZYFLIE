#include "orb_slam3/data_grabber.hpp"

// Default constructor
DataGrabber::DataGrabber() : mbClahe(false) {}

// Constructor with parameters for SLAM system, CLAHE, ROS publisher, node, and camera frame name
DataGrabber::DataGrabber(std::shared_ptr<ORB_SLAM3::System> pSLAM, bool bClahe, 
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
    std::shared_ptr<rclcpp::Node> ros_node, const std::string camera_frame_name)
    : mpSLAM(pSLAM), mbClahe(bClahe),
    odom_pub_(rospub), rosNode_(ros_node),
    tf_frame(camera_frame_name) 
    {
        // Initialize odometry message frame IDs
        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = tf_frame;

        // Initialize odometry message position to zero
        odom_msg_.pose.pose.position.x = 0.0;
        odom_msg_.pose.pose.position.y = 0.0;
        odom_msg_.pose.pose.position.z = 0.0;
        
        // Initialize odometry message orientation to zero quaternion
        odom_msg_.pose.pose.orientation.x = 0.0;
        odom_msg_.pose.pose.orientation.y = 0.0;
        odom_msg_.pose.pose.orientation.z = 0.0;
        odom_msg_.pose.pose.orientation.w = 0.0;
    }

// Callback to grab an image message and push it to the buffer
void DataGrabber::grabImage(const custom_msgs::msg::ImageAndInt::SharedPtr img_msg)
{
    std::unique_lock<std::mutex> lock(mImgMutex);

    if (!img0Buf.empty())
        img0Buf.pop();  // Remove the oldest image to process the latest one

    img0Buf.push(img_msg);
    lock.unlock();

    mImgReady.notify_all();  // Notify the processing thread
}

// Convert a ROS image message to a cv::Mat object
cv::Mat DataGrabber::getImage(const custom_msgs::msg::ImageAndInt::SharedPtr &img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        auto image_ptr = std::make_shared<sensor_msgs::msg::Image>(img_msg->image);
        cv_ptr = cv_bridge::toCvShare(image_ptr, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ImageGrabber"), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
    return cv_ptr->image.clone();
}

// Callback to grab an IMU message and push it to the buffer
void DataGrabber::getImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg){
    
    // Extract timestamp and IMU data
    double timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
    float acc_x = imu_msg->linear_acceleration.x;
    float acc_y = imu_msg->linear_acceleration.y;
    float acc_z = imu_msg->linear_acceleration.z;
    float gyro_x = imu_msg->angular_velocity.x;
    float gyro_y = imu_msg->angular_velocity.y;
    float gyro_z = imu_msg->angular_velocity.z;

    // Create IMU point and push to buffer
    ORB_SLAM3::IMU::Point p(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, timestamp);
    std::unique_lock<std::mutex> lock(mImuMutex);
    imu0Buf.push(p);

    lock.unlock();
    mImuReady.notify_all();
}

// Main processing loop for images and IMU data
void DataGrabber::processData()
{
    // for debugging
    double old_tStamp = 0;
    double new_tStamp = 0;
    //

    while (rclcpp::ok())
    {
        cv::Mat im;
        double tIm = 0;
        int timestamp = 0;
        // Wait for an image in the buffer
        {
            std::unique_lock<std::mutex> lk(mImgMutex);
            while(img0Buf.empty()){
                mImgReady.wait(lk);
            }
        }   

        // Get the latest image and its timestamp
        {
            std::lock_guard<std::mutex> lock(mImgMutex);
            timestamp = img0Buf.front()->integer;
            im = getImage(img0Buf.front());
            tIm = img0Buf.front()->image.header.stamp.sec + img0Buf.front()->image.header.stamp.nanosec * 1e-9;
        }

        // Wait for IMU data to be available and synchronized
        {
            std::unique_lock<std::mutex> lk(mImuMutex);
            while(imu0Buf.empty() || tIm > imu0Buf.back().t){
                mImuReady.wait(lk);
            }
        }

        // Remove the processed image from the buffer
        {
            std::lock_guard<std::mutex> lock(mImgMutex);
            img0Buf.pop();
        }

        if (im.empty()) {
            continue;
        }

        // Optionally apply CLAHE to the image
        if (mbClahe) {
            mClahe->apply(im, im);  // Apply CLAHE if enabled
        }
                    
        // Process the image in the SLAM system
        Sophus::SE3f curr_pose;
        
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        vImuMeas.clear();

        // Collect all IMU measurements up to the image timestamp
        {
            std::lock_guard<std::mutex> lock(mImuMutex);
            while(imu0Buf.front().t <= tIm && !imu0Buf.empty()) {
                vImuMeas.push_back(imu0Buf.front());
                imu0Buf.pop();
            }
        }
                        
        try {
            // Track the current frame with image and IMU data
            curr_pose = mpSLAM->TrackMonocular(im, tIm, vImuMeas); 
            //curr_pose = mpSLAM->TrackMonocular(im, tIm); // Without IMU measurements

            // For Debugging -------------------------------------------
            new_tStamp = tIm;
            // if (new_tStamp - old_tStamp > 0) {
            //     std::cout << "Time stamp difference: " << new_tStamp - old_tStamp << std::endl;
            // }
            old_tStamp = new_tStamp;
            std::cout << "Frame computed, number of imu measurements: " << vImuMeas.size() << std::endl;

            //  --------------------------------------------
        } catch (const std::exception &e) {
            std::cerr << "Exception caught: " << e.what() << std::endl;
        }
        
        // Publish the pose as an odometry message
        publishSE3fToOdom(curr_pose, timestamp);           
        
    }
}

// Publish a Sophus::SE3f pose as a nav_msgs::msg::Odometry message
void DataGrabber::publishSE3fToOdom(const Sophus::SE3f& se3, int timestamp) {
    
    // Extract the translation (position)
    Eigen::Vector3f translation = se3.translation();
    odom_msg_.header.stamp.sec = timestamp;

    odom_msg_.pose.pose.position.x = translation.x();
    odom_msg_.pose.pose.position.y = translation.y();
    odom_msg_.pose.pose.position.z = translation.z();

    // Extract the rotation and convert to quaternion
    Eigen::Matrix3f rotation_matrix = se3.rotationMatrix();
    Eigen::Quaternionf quaternion(rotation_matrix);

    odom_msg_.pose.pose.orientation.x = quaternion.x();
    odom_msg_.pose.pose.orientation.y = quaternion.y();
    odom_msg_.pose.pose.orientation.z = quaternion.z();
    odom_msg_.pose.pose.orientation.w = quaternion.w();

    // Publish the odometry message
    odom_pub_->publish(odom_msg_);
}