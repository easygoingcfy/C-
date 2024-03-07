/**
 * @file updater_node.cpp
 * @author caofangyu (caofy@antwork.link)
 * @brief 
 * @version 0.1
 * @date 2024-03-06
 * 
 * Copyright (c) 2015-2024 Xunyi Ltd. All rights reserved.
 * 
 */

#include <ros/ros.h>

#include "antwork_updater.h"

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "update_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Your code here
    AntworkUpdater updater;
    updater.run();

    ROS_INFO("waitting for node shutdwon...");
    ros::waitForShutdown();

    return 0;
}