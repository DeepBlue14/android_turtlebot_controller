#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/Point.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <iostream>
#include <string>

#include "From_ros_ip.h"

using namespace std;

From_ros_ip fromRosIp;

void callback(
            /*sensor_msgs::CompressedImageConstPtr& msg*/
            /*const geometry_msgs::Point msg*/
            const std_msgs::Int32::ConstPtr& msg
            /*const std_msgs::String::ConstPtr&*/)
{
    fromRosIp.publish(msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<
                                    /*sensor_msgs::CompressedImage*/
                                    /*geometry_msgs::Point*/
                                    std_msgs::Int32
                                    /*std_msgs::String*/>("/chatter", 10, callback);
                                    
    cout << "finished ros stuff" << endl;
    
    fromRosIp.connecter(50001);
    cout << "ran connecter" << endl;

    ros::spin();
    
    return 0;
}
