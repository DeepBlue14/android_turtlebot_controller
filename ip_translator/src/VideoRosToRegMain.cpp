/*
 * File:   VideoRosToReg_main.cpp
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This file... TODO: add description.
 *
 * Created July 6, 2015 at 10:30
 */


#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <iostream>
#include <string>
#include <cstdlib>

#include "VideoRosToReg.h"

using namespace ros;
using namespace std;

VideoRosToReg videoRosToReg;

void callback(const sensor_msgs::ImageConstPtr& msg)
{
    videoRosToReg.publishTcp(msg);
}


int main(int argc, char **argv)
{
    init(argc, argv, "VideoRosToReg");

    ROS_INFO("Starting VideoRosToReg");

    NodeHandle nh;
    Subscriber sub = nh.subscribe<sensor_msgs::Image>(/*"/usb_cam/image_raw"*/"/camera/rgb/image_rect_color", 1, callback);


    if(argc == 2)
    {
        videoRosToReg.connect2Client(atoi(argv[1]) );
    }
    else
    {
        videoRosToReg.connect2Client(50000);
    }

    ros::spin();


    return EXIT_SUCCESS;
}
