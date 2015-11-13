/*
 * File:   Reciever.h
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: To use this class, call connector(...) to initialize the class.
 *                   Then in your ROS callback method, call From_ros_ip::publish(...) to send the data over TCP/IP
 *
 * Reference: http://www.linuxhowtos.org/C_C++/socket.htm
 * "nmap -PN 10.0.4.6"
 * http://doc.qt.io/qt-5/qtwebsockets-examples.html (websocket)
 * https://github.com/RobotWebTools/web_video_server (http)
 *
 * Created July 29, 2015 at 7:30
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Char.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

#include <geometry_msgs/Point.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

//OpenCV and ROS <---> OpenCV
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <assert.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h> /*James was here*/
#include <arpa/inet.h> /*James was here*/

using namespace std;

#ifndef FROM_ROS_IP_H
#define FROM_ROS_IP_H

class From_ros_ip
{
    private:
        static int m_comm_fd;

    public: 
        From_ros_ip();
        int connecter(int port);
        // std_msgs
        static void publish(const std_msgs::Bool::ConstPtr& msg);
        static void publish(const std_msgs::Byte::ConstPtr& msg);
        static void publish(const std_msgs::ByteMultiArray::ConstPtr& msg);
        static void publish(const std_msgs::Char::ConstPtr& msg);
        static void publish(const std_msgs::ColorRGBA::ConstPtr& msg);
        static void publish(const std_msgs::Duration::ConstPtr& msg);
        static void publish(const std_msgs::Empty::ConstPtr& msg);
        static void publish(const std_msgs::Float32::ConstPtr& msg);
        static void publish(const std_msgs::Float32MultiArray::ConstPtr& msg);
        static void publish(const std_msgs::Float64::ConstPtr& msg);
        static void publish(const std_msgs::Float64MultiArray::ConstPtr& msg);
        static void publish(const std_msgs::Int16::ConstPtr& msg);
        static void publish(const std_msgs::Int16MultiArray::ConstPtr& msg);
        static void publish(const std_msgs::Int32::ConstPtr& msg);//tested
        static void publish(const std_msgs::Int32MultiArray::ConstPtr& msg);
        static void publish(const std_msgs::Int64::ConstPtr& msg);
        static void publish(const std_msgs::Int64MultiArray::ConstPtr& msg);
        static void publish(const std_msgs::Int8::ConstPtr& msg);
        static void publish(const std_msgs::Int8MultiArray::ConstPtr& msg);
        static void publish(const std_msgs::MultiArrayDimension::ConstPtr& msg);
        static void publish(const std_msgs::MultiArrayLayout msg);
        static void publish(const std_msgs::String::ConstPtr& msg);//tested
        static void publish(const std_msgs::Time::ConstPtr& msg);
        static void publish(const std_msgs::UInt16::ConstPtr& msg);
        static void publish(const std_msgs::UInt16MultiArray::ConstPtr& msg);
        static void publish(const std_msgs::UInt32::ConstPtr& msg);
        static void publish(const std_msgs::UInt32MultiArray::ConstPtr& msg);
        static void publish(const std_msgs::UInt64::ConstPtr& msg);
        static void publish(const std_msgs::UInt64MultiArray msg);
        static void publish(const std_msgs::UInt8::ConstPtr& msg);
        static void publish(const std_msgs::UInt8MultiArray::ConstPtr& msg);
        // unsigned char[] serialization
        //static void publish(const unsigned char* msg, Type type = CHARS, char delim = ',');
        static void publish(const geometry_msgs::Point msg, char delim = ',');
        static void publish(const sensor_msgs::CompressedImageConstPtr& msg);
        static void publish(const sensor_msgs::ImageConstPtr& msg);
        void finish(); //close the socket after finished using it (i.e. at the end of the program).
        string* toString();
        ~From_ros_ip();

};

#endif
