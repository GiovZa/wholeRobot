# object_detection_with_intel_d455
Using Jetson Xavier NX and D455 Intel Camera to find Pose Estimation of objects through tf in RVIZ on ROS Melodic

Based off of https://github.com/bandasaikrishna/object_detection_and_3d_pose_estimation.git code, but with parameters remapped to include rgbd_launch since D455 can't use normal pointcloud with Jetson.

Requires installation of rgbd_launch which can be installed with the following inputted into linux terminal:

git clone https://github.com/ros-drivers/rgbd_launch.git

Requires installation of find-object-2d which can be installed with the following inputted into linux terminal:

sudo apt-get install ros-$ROS_DISTRO-find-object-2d
