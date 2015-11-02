/*
 * File:   VideoRegToRos_main.cpp
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This is a example of how ros_ip_transform can be used
 *                   to convert TCP/IP --> ROSTopic.
 *
 * Created July 6, 2015 at 10:30
 */
 

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <iostream>
#include <string>
#include <cstdlib>

#include "VideoRegToRos.h"

using namespace ros;
using namespace std;


int main(int argc, char **argv)
{
    init(argc, argv, "VideoRegToRos");

    ROS_INFO("Starting VideoRegToRos");

    RosIpT::VideoRegToRos videoRegToRos;
    NodeHandle nh;
    Publisher* mainsPub = videoRegToRos.getPublisher();
    *mainsPub = nh.advertise<sensor_msgs::Image>("/test/rgb/image_rect_color", 1);
    
    videoRegToRos.connect2Server("127.0.0.1", 50000);

    videoRegToRos.spinTcp();
    

    return EXIT_SUCCESS;
}
