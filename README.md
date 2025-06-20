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

7. Inside the /~/ORBSLAM3_CRAZYFLIE/ros_ws/src/orb_slam3 package, you need to add the ORBvoc.txt file to the config folder. You can find that file in the ORB_SLAM3/Vocabulary folder.

8. Install the dependecies for the execution of the project:
    ```bash
    cd ~/ORBSLAM3_CRAZYFLIE/ros_ws
    rosdep install --from-paths src
    ```
9. Build the ros2 workspace:
    ```bash
    colcon build
    ```

## Drone setup

In order to perform the Visual Inertia Odometry with the Crazyflie, you have to setup the hardware correctly. You should follow the guide provided by bitcrazy to setup the drone and the AI deck firmaware https://www.bitcraze.io/documentation/tutorials/getting-started-with-aideck/

After the drone setup, you **must** perform a calibration to obtain the intrinsic parameters of the camera and the imu and the homogeneus transformation between the camera frame and the imu frame. Then, you have to update the parameters inside the ORBSLAM3_CRAZYFLIE project.

The parameters used by orb_slam3 are in the **camera_and_slam_settings.yaml** file inside the config folder of the orb_slam3 ros2 package (~/ORBSLAM3_CRAZYFLIE/ros_ws/src/orb_slam3/config/camera_and_slam_settings.yaml). 


Note: When we have implemented the project, the last release (2025.02) of the firmware didn't work properly (The image flow was blocked after some time), so we used the release 2024.10.2 .

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

### Optional

You can pass optional parameters to the launch file:

- `URI`: Radio address of the Crazyflie.
- `IP`: IP address for WiFi communication.
- `PORT`: Port number for WiFi communication.
- `LOG_ACTIVE`: Set to `True` to enable logging of the Crazyflie console output to a text file.

When `LOG_ACTIVE` is enabled, a file named `console_crazyflie.txt` will be created in the directory where you run the command.

Example usage:
```bash
ros2 launch crazyflie_package streaming.launch.py URI:=radio://0/86/2M/E7E7E7E7ED IP:=192.168.4.1 PORT:=5000 LOG_ACTIVE:=False
```
The default values are those shown in the example command.


## Calibration (suggested procedure)

In order to use the VIO you need to provide the correct intrinsic parameters for the camera and for the IMU. In addition, you need to provide the homogeneous transformation between the camera and the IMU. 

You can perform the calibration in the way you prefer, in our case we have used the well established Kalibr tool. To use the Kalibr calibration package Ros1 melodic is necessary. We used a docker container with ros1 melodic to run the code.

#### Camera calibration

First of all, you need to create a rosbag where you must record the /cam0/image_raw and /imu0 topics. As mentioned in the kalibr guide, you need to capture a video of a calibration pattern (es. aprilgrid) while moving the drone to excitate the IMU along all the axes.

Then, you have to convert the rosbag from the ROS2 format to the ROS1 format. You can use the rosbags utils:

```bash
pip install rosbags
rosbags-convert --src [ROS2 BAG FOLDER PATH] --dst [ROS1 BAG FILE (.bag)]
```

After the recording, you need to run the Kalibr node to obtain the extrinsic parameters of the camera:

1. write a param.yaml file with the information related to the camera and the calibration target:

    ```yaml
    target_type: 'aprilgrid'
    tagCols: 3
    tagRows: 4
    tagSize: 0.05
    tagSpacing: 0.2
    ```

2. Run the Kalibr cameras calibration script:
    ```bash
    rosrun kalibr kalibr_calibrate_cameras --target ./param.yaml --bag ./calibration_bag.bag --models pinhole-radtan --topics /cam0/image_raw
    ```

3. The parameters will be insertend by the script in the camchain.yaml file.

#### IMU noise density and random walk estimation

The IMU noise density and random walk are necessary to perform a joint optimization between the recorded camera motion and the IMU's data to obtain the homogeneus transformation between the two.

In order to estimate the noise density and the random walk, it is possible to use the Ros1 package allan_variance_ros https://github.com/ori-drs/allan_variance_ros. It is necessary to record a ros bag (the longer it is, better it is, suggested 3h) for the /imu0 topic with the drone that is standstill. After the recording, you need run the following commands:

1. Reorganize the messages by timestamps.
    ```bash
    rosrun allan_variance_ros cookbag.py --input original_rosbag --output cooked_rosbag.bag
    ```

2. Create a folder where to put the ros bag.
    ```bash
    mkdir bag_folder
    mv coocked_rosbag.bag ./bag_folder
    ```
3. Create a config file config.yaml.

    ```yaml
    imu_topic: "/imu0"
    imu_rate: 100
    measure_rate: 100 # Rate to which imu data is subsampled
    sequence_time: 10800
    ```

4. Run the command for the parameters estimation:

    ```bash
    rosrun allan_variance_ros allan_variance ./bag_folder ./config.yaml
    ```

A imu.yaml file will be created with the estimated parameters.

#### Joint Camera and IMU calibration

After that the camchain.yaml and imu.yaml are ready, you can use the previously recorded ros bag to obtain the homogeneous transformation.

```bash
rosrun kalibr_calibrate_imu_camera --bag ./calibration_bag.bag --cam ./camchain.yaml --imu ./imu.yaml --target ./param.yaml
```
The result is written inside the calibration_bag-camchain-imucam.yaml.


Note: It may be necessary to perform a cast of the calibration_bag.bag becouse kalibr has some trouble at the beginning and at the end of the rosbag. To do that you can use the following command that removes some data from the beginning and the end of the rosbag.

To check the starting time and the finish time of the calibration_bag.bag, you can run the following command:

```bash
rosbag info calibration_bag.bag 
```

then you have to run the rosbag filter command adding some seconds to the starting time and removing some seconds to the end time.

```bash
rosbag filter calibration_bag.bag trimmed_calibration_bag.bag "t.to_sec() > [new starting time] and t.to_sec() < [new end time]"
```

After the calibration, you have to update the parameters inside the ORBSLAM3_CRAZYFLIE project.
The parameters used by Orb_slam3 are in the camera_and_slam_settings.yaml file inside the config folder of the orb_slam3 ros2 package (~/ORBSLAM3_CRAZYFLIE/ros_ws/src/orb_slam3/config/camera_and_slam_settings.yaml). 


## Contributing

Contributions are welcome! Please open issues or submit pull requests.

## License

This project is licensed under the GPLv3 License.

## Acknowledgements

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [Bitcraze Crazyflie](https://www.bitcraze.io/)
