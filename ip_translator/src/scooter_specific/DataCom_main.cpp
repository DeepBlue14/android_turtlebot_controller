#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <string>
#include <cstdlib>

#include "DataCom.h"

using namespace ros;
using namespace std;

DataCom dataCom;

void callback(const sensor_msgs::ImageConstPtr& msg)
{
    cout << "@ callback(...)" << endl;
    dataCom.publishTcp(msg);
}


int main(int argc, char **argv)
{
    init(argc, argv, "DataCom");

    ROS_INFO("Starting DataCom (port 50001)");

    NodeHandle nh;
    Subscriber sub = nh.subscribe<sensor_msgs::Image>(/*"/usb_cam/image_raw"*/"/camera/rgb/image_rect_color", 1, callback);
    Publisher* mainsPub = dataCom.getPublisher();
    Publisher* mainsUpdatePub = dataCom.getUpdatePublisher();
    //*mainsPub = nh.advertise<geometry_msgs::Point>("/scooter/geometry_msgs/pixel_point", 10);
    *mainsPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
    *mainsUpdatePub = nh.advertise<std_msgs::String>("/robot_001/update", 10);
    dataCom.sc = new sound_play::SoundClient();
    dataCom.connect2Client(50001);
    ros::spin();


    return EXIT_SUCCESS;
}
