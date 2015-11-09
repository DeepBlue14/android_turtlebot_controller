/*
 * File:   RosServerWs.h
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This program converts sensor_msgs::Image to OpenCV matrix,
 *                   which it then sends over a TCP/IP connection.  This allows
 *                   it to be easily recieved and decoded by a client system
 *                   running Windows, Linux, OS X, FreeBSD, NetBSD, OpenBSD,
 *                   Android, IOS, Maemo, or BlackBerry, and using any
 *                   language supported by OpenCV (C/C++, Java, Android,
 *                   Python, and MATLAB/OCTAVE, as well as partial
 *                   functionality for C#, Perl, Ruby, and Ch.
 *
 * Created September 17, 2015 at 6:00pm
 */


#ifndef ROS_SERVER_WS_H
#define ROS_SERVER_WS_H

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/String.h>

#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QObject>
#include <QVector>
#include <QByteArray>

#include <QWebSocketServer>
#include <QWebSocket>

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

#include "RosIpT.h"

using namespace ros;
using namespace cv;
using namespace std;
using namespace RosIpT;


class RosIpT::RosServerWs : public QObject
{
    Q_OBJECT
    
    private:
        QWebSocketServer* webSocketServer;
        QVector<QWebSocket*>* clientVecPtr;
        Publisher* pub;
        
        
    private slots:
        void handleConnectSlot();
        void handleRecv(QString message);
        void handleSockDisconnect();
    
    
    public:
        RosServerWs(uint16_t port, QObject* parent = 0);
        void connect2Client(int port);
        void publishWs(const std_msgs::String::ConstPtr& msg);
        Publisher* getPublisher();
        ~RosServerWs();

};


#endif /* ROS_SERVER_WS_H */
