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
#include <geometry_msgs/Point.h>

#include <iostream>
#include <string>
#include <cstdlib>

#include "RosServer.h"

using namespace ros;
using namespace std;


int main(int argc, char **argv)
{
    init(argc, argv, "RosServer");

    ROS_INFO("Starting RosServer");

    RosServer rosServer;
    NodeHandle nh;
    Subscriber sub;


    if(argc == 3)
    {
        cout << "HERE (1)" << endl;
        switch(atoi(argv[2]) )
        {
            case 0:
                cout << "HERE (2)" << endl;
                sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 10, &RosServer::callback, &rosServer);
                break;
            case 1:
                cout << "HERE (3)" << endl;
                sub = nh.subscribe<geometry_msgs::Point>("/tablet/geometry_msgs/point", 10, &RosServer::callback2, &rosServer);
                break;
            default:
                cout << "HERE (4)" << endl;
                sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 10, &RosServer::callback, &rosServer);
        }
        
        rosServer.connect2Client(atoi(argv[1]) );
    }
    else
    {
        sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 10, &RosServer::callback, &rosServer);
        rosServer.connect2Client(50000);
    }

    ros::spin();


    return EXIT_SUCCESS;
}
