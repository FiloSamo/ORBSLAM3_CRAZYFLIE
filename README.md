# ORBSLAM3_CRAZYFLIE

ORBSLAM3_CRAZYFLIE is a project that integrates the ORB-SLAM3 Visual Inertia Odometry system with the Crazyflie nano quadcopter platform. This enables real-time localization for autonomous navigation and research applications.

## Features

- Real-time Visual Inertial Odometry using ORB-SLAM3
- ROS2 (Robot Operating System 2) compatibility

## Requirements

- Crazyflie 2.x quadcopter
- AI deck
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- ROS2 Humble
- Python 3.x
- C++17 compatible compiler

## Installation

1. Clone this repository:
    ```bash
    cd ~
    git clone --recursive https://github.com/FiloSamo/ORBSLAM3_CRAZYFLIE.git
    ```
2. Build Custom Pangolin Library from the provided source code:
    ```bash
    cd ORBSLAM3_CRAZYFLIE/Pangolin
    source install_prerequisites.sh
    mkdir build
    cd build
    cmake .. && make
    make install
    ```

3. You need to add a Environment variable with the path to Pangolin lib (I suggest you to add this command in your .bashrc file in the ~ folder):
    ```bash
    export Pangolin_DIR="/~/ORBSLAM3_CRAZYFLIE/Pangolin/build:$Pangolin_DIR"
    ```

4. Install ORB-SLAM3 and its dependencies (see [ORB-SLAM3 installation guide](https://github.com/UZ-SLAMLab/ORB_SLAM3)).

5. Update the .bashrc file with the environment variable with path to the ORB-SLAM3 library:
    ```bash
    export LD_LIBRARY_PATH="/~/ORBSLAM3_CRAZYFLIE/ORB_SLAM3/lib:/usr/local/lib:$LD_LIBRARY_PATH"
    ```

6. Inside the /~/ORBSLAM3_CRAZYFLIE/ros_ws/src/orb_slam3 package, you need to update the CMakeList.txt file with the correct paths to your libraries:
    ```txt
    set(ORB_SLAM3_DIR "/~/ORBSLAM3_CRAZYFLIE/ORB_SLAM3")  # Adjust this to your ORB_SLAM3 directory

    set(PANGOLIN_LIB_DIR "/~/ORBSLAM3_CRAZYFLIE/Pangolin/build/src/libpangolin.so")  # Adjust this to your Pangolin lib path
    ```

7. Install the dependecies for the execution of the project:
    ```bash
    cd ~/ORBSLAM3_CRAZYFLIE/ros_ws
    rosdep install --from-paths src
    ```
8. Build the ros2 workspace:
    ```bash
    colcon build
    ```

## Usage

After the installation procedure, you can perform a real time VIO with the crazyflie by following this steps:

1. Source the ros2 environment:
    ```bash
    cd ~/ORBSLAM3_CRAZYFLIE/ros_ws
    source install/local_setup.bash
    ``` 
2. Connect to the "WiFi streaming example".
   
3. Launch the streamers for the images and the imu data (with the correct URI):
    ```bash
    ros2 launch crazyflie_package streaming.launch.py URI:=radio://0/86/2M/E7E7E7E7E7
    ```  

4. Launch the VIO:
    ```bash
    ros2 launch orb_slam3 monocular_vio.launch.py
    ```  
## Calibration

In order to use the VIO you need to provide the correct intrinsic parameters for the camera and for the IMU. In addition, you need to provide the homogeneous transformation between the camera and the IMU. To do that, we have used the well established Kalibr tool.

### Camera calibration (suggested)

First of all, you need to create a ros bag where you must record the /cam0/image_raw and /imu0 topics. As mentioned in the kalibr guide, you need to capture a video of a calibration pattern (es. aprilgrid) while moving the drone to excitate the IMU along all the axes.

Then, you have to convert the ros bag from the ROS2 format to the ROS1 format. You can use the rosbags utils:

    pip install rosbags

    rosbags-convert --src [ROS2 BAG FOLDER PATH] --dst [ROS1 BAG FILE (.bag)]

After the recording, you need to run the Kalibr node to obtain the extrinsic parameters of the camera:

    rosrun kalibr kalibr_calibrate_cameras ...

The parameters will be insertend in a camchain.yaml file.

### IMU noise density and random walk estimation

The IMU noise density and random walk are necessary to perform a joint optimization between the recorded camera motion and the IMU's data to obtain the homogeneus transformation between the two.

In order to estimate the noise density and the random walk, it is possible to use the Ros1 package allan_variance_ros. It is necessary to record a ros bag (the longer it is, better it is, suggested 3h) for the /imu0 topic with the drone that is standstill. After the recording, you need run the following commands:

1. Reorganize the messages by timestamps.
    ```bash
    rosrun allan_variance_ros cookbag.py --input original_rosbag --output cooked_rosbag.bag
    ```

2. Create a folder where to put the ros bag.
    ```bash
    mkdir bag_folder
    mv coocked_rosbag.bag ./bag_folder
    ```
3. Create a config file.
    ```bash
    ----
    ```

4. Run the command for the parameters estimation:
    ```bash
    rosrun ...
    ```

A imu.yaml file will be created with the estimated parameters.

### Joint Camera and IMU calibration

After that the camchain.yaml and imu.yaml are ready, you can use the previously recorded ros bag to obtain the homogeneous transformation. 

    rosrun kalibr .....

## Usage

1. Connect your Crazyflie and camera.
2. Launch the SLAM node:
    ```bash
    ./run_orbslam3_crazyflie
    ```
3. (Optional) Use ROS to visualize and control the Crazyflie.

## Contributing

Contributions are welcome! Please open issues or submit pull requests.

## License

This project is licensed under the MIT License.

## Acknowledgements

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [Bitcraze Crazyflie](https://www.bitcraze.io/)
