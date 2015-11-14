#ifndef DATA_COM_H
#define DATA_COM_H

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <assert.h>

#include <sys/socket.h> 
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h> 


using namespace ros;
using namespace std;


class DataCom
{   
    private:
        static int m_comm_fd;
        int m_port;
        int m_count;
        int localSocket;
        int remoteSocket;
        int port;
        struct sockaddr_in localAddr,
                           remoteAddr;
        int addrLen;
        Publisher* pub;
        
    public:
        DataCom();
        //void callback(const sensor_msgs::ImageConstPtr& image);
        bool connect2Client(int port);
        //void connectionBrokeHandler(int port);
        void publishTcp(const sensor_msgs::ImageConstPtr& msg);
        Publisher* getPublisher();
        ~DataCom();

};


#endif /* DATA_COM_H */
