/*
 * File:   RosServer_main.cpp
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This is a example of how ros_ip_transform can be used
 *                   to convert ROSTopic --> TCP/IP.
 *
 * Created July 6, 2015 at 10:30
 */


#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <iostream>
#include <string>
#include <cstdlib>

#include "RosServer.h"

using namespace ros;
using namespace std;

RosServer rosServer;

void callback(const sensor_msgs::ImageConstPtr& msg)
{
    rosServer.publishTcp(msg);
}


int main(int argc, char **argv)
{
    init(argc, argv, "RosServer");

    ROS_INFO("Starting RosServer");

    NodeHandle nh;
    Subscriber sub = nh.subscribe<sensor_msgs::Image>(/*"/usb_cam/image_raw"*/"/camera/rgb/image_rect_color", 10, callback);


    if(argc == 2)
    {
        rosServer.connect2Client(atoi(argv[1]) );
    }
    else
    {
        rosServer.connect2Client(50000);
    }

    ros::spin();


    return EXIT_SUCCESS;
}
