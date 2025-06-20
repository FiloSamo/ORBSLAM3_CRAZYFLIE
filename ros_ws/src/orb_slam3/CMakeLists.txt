cmake_minimum_required(VERSION 3.8)
project(orb_slam3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED) 

# Find OpenCV (ORB_SLAM3 depends on OpenCV)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)

set(ORB_SLAM3_DIR "/home/filippo/Ros2/ORBSLAM3_CRAZYFLIE/workspace/ORB_SLAM3")  # Adjust this to your ORB_SLAM3 directory

set(PANGOLIN_LIB_DIR "/home/filippo/Ros2/ORBSLAM3_CRAZYFLIE/workspace/Pangolin/build/src/libpangolin.so")  # Adjust this to your Pangolin lib path

include_directories(include)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  )
  
add_executable(orb_slam3 src/orb_slam3.cpp src/data_grabber.cpp)

link_directories(
  ${ORB_SLAM3_DIR})

# Link the ORB_SLAM3 library and dependencies
target_link_libraries(orb_slam3
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${OPENGL_LIBRARIES} 
  ${GLEW_LIBRARIES}  # Add this line to link GLEW
  ${PANGOLIN_LIB_DIR}
)


# Specify ROS 2 package dependencies
ament_target_dependencies(orb_slam3
  rclcpp
  sensor_msgs
  cv_bridge
  nav_msgs
  custom_msgs
)

install(TARGETS
  orb_slam3
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
