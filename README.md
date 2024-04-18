# CPE631_final_sample, Human Robot Interaction Environment

## Requirements:
- Ubuntu 20.04
- ROS - Noetic
- The simulation environment is not guaranteed to work with any other ROS or Ubuntu environment.

## Pre-requisites:
1. C++11 compiler which is basically gcc/g++.
2. Gazebo-ROS packages, which should come with a Desktop-full installation of ROS Noetic.

## Running Instructions:
1. Install dependencies:

    `sudo apt install ros-noetic-navigation`

    `sudo apt install ros-noetic-turtlebot3-*`

2. Suppose you have made a catkin workspace named catkin_ws.
3. Once you are finished with all the dependencies, you can git clone this package from github into your ROS workspace and catkin_make it.

    `cd ~/catkin_ws/src`

    `git clone --recurse-submodules https://github.com/SunnyGuang/cpe631`

    `cd ~/catkin_ws`

    `catkin_make`
4. If you installed all dependencies correctly, you shouldn’t get any errors.
5. Change the file permissions to make it executable in /src fold if required:

    `cd/cpe631/src`

    `ls`

    `chmod +x *`
6. You can run the environment by executing

    `roslaunch cpe631 cafe.launch`
7. Once you successfully run the node, you will see topic related to laser range finder, IMU and controlling the velocity of the robot. You can subscribe to and publish on these topics in your own controller to get the sensor information and publish the desired velocities to move the robot.

    Linear and angular velocity topic: 

    `/cmd_vel`
    
    IMU topic:

    `/imu`

    Laser Range finder topic:

    `/scan`

    Pose of each model in the world:

    `/gazebo/model_states`

## Reporting Issues
If something is not working or you have any questions, please report it with a GitHub Issue.