#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <string>

#include "To_ros_ip.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ToIpTest");
    
    NodeHandle nh;
    
    To_ros_ip toRosIp;
    Publisher* mainsPub = toRosIp.getPublisher();
    //*mainsPub = nh.advertise<std_msgs::String>("/fromtcp/to/rosip", 10);
    *mainsPub = nh.advertise<std_msgs::Int32>("/recvval", 10);
    //*mainsPub = nh.advertise<geometry_msgs::Point>("/fromtcp/to/rosip", 10);
    cout << "finished ROS stuff" << endl;
    toRosIp.connecter("10.0.4.6", 50001);
    cout << "finished \"To\" connecter" << endl;
    
    toRosIp.sendOnce();
    
    ros::Rate r(10); //10 hz
    while(ros::ok())
    {
        
        toRosIp.subscribe(/*To_ros_ip::STD_INT_SEQ,*/ /*To_ros_ip::ROS_POINT*/ To_ros_ip::STD_INT, To_ros_ip::ROS_INT32); //reads tcp data --> publishes rostopic
        r.sleep();
    }
    
    return 0;
}
