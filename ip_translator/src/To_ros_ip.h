//see: http://stackoverflow.com/questions/12984816/get-the-number-of-bytes-available-in-socket-by-recv-with-msg-peek-in-c

#ifndef TO_ROS_IP_H
#define TO_ROS_IP_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>

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
using namespace ros;

class To_ros_ip
{
    private:
		int sokt;
        string serverIPStr;
        char* serverIP;
        static int m_comm_fd;
        struct sockaddr_in serverAddr;
        socklen_t addrLen;
        Publisher* pub;

    public:    
        enum From
        {
            STD_CHARPTR = 0,
            STD_INT,
            STD_INT_SEQ,    //sequence of ints in char* form
            STD_BOOL,
            STD_FLOAT,
            CV_MAT
            //...
        };
        
        enum To
        {
            ROS_STRING = 0,
            ROS_INT32,
            ROS_POINT,
            ROS_IMAGE
            //...
        };
      
		To_ros_ip();
		int connecter(const char* address, int port);
		void sendOnce();
		void subscribe(From fromType = STD_CHARPTR, To toType = ROS_STRING);
		void finish();
		Publisher* getPublisher();
		string* toString();
		~To_ros_ip();

};

#endif /* TO_ROS_IP_H */
