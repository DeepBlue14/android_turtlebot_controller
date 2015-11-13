#include "To_ros_ip.h"


int To_ros_ip::m_comm_fd;


To_ros_ip::To_ros_ip()
{
    pub = new Publisher();
}


int To_ros_ip::connecter(const char* address, int port)
{
    int listen_fd, comm_fd;
 
    struct sockaddr_in servaddr;
 
    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
 
    bzero( &servaddr, sizeof(servaddr));
 
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htons(INADDR_ANY);
    servaddr.sin_port = htons(port);
 
    bind(listen_fd, (struct sockaddr *) &servaddr, sizeof(servaddr));
 
    listen(listen_fd, 10);
    
    m_comm_fd = comm_fd = accept(listen_fd, (struct sockaddr*) NULL, NULL);
    ROS_INFO("Accepted the connection\n"); 
    //cout << "listen: " << listen_fd << "\tcomm: " << comm_fd << endl;
}


void To_ros_ip::sendOnce()
{
    string str = "ready";
    stringstream ss;
    ss << (str.size()+1);
    string tmpStr = ss.str();
    unsigned char* sizeAsCharPtr = (unsigned char*) tmpStr.c_str();
    unsigned char* dataCharPtr = (unsigned char*) str.c_str();

    int bytes = 0;
    cout << "about to send: " << dataCharPtr << ", size: " << (str.size()+1) << endl;
    ssize_t r;
    //r = send(m_comm_fd, sizeAsCharPtr, sizeof(int), 0); // send size of data 
    r = send(m_comm_fd, dataCharPtr, str.size()+1, 0); //send data
}


void To_ros_ip::subscribe(From fromType, To toType)
{
    cout << "@ subscribe" << endl;
	int bytes = 0;
	if(fromType == STD_INT && toType == ROS_INT32)
	{
	    cout << "chosen if block" << endl;
	    int buffer;
	    if((bytes = recv(sokt, &buffer, 1, MSG_WAITALL)) == -1)
        {
    	    ROS_WARN("recv failed\n");
        }
        else
        {
            cout << "data: " << buffer << endl;
	        std_msgs::Int32 msg;
	        msg.data = buffer;
	        pub->publish(msg);
        }
	}
	else if(fromType == STD_CHARPTR && toType == ROS_STRING)
	{
	    string str = "x";
	    const int SIZE_OF_C_INTEGER = sizeof(int);

	    for(size_t i = 0; i < SIZE_OF_C_INTEGER-1; i++)
	    {
	        str.append("x");
	    }

	    unsigned char* sizeCharPtr = (unsigned char*) str.c_str();
	    
	    recv(sokt, sizeCharPtr, SIZE_OF_C_INTEGER, MSG_WAITALL);

	    const char* numCharStar2 = (const char*) sizeCharPtr;
	    int num = atoi(numCharStar2);

	    str.erase();
	    for(size_t i = 0; i < num; i++)
	    {
	        str.append("x");
	    }
	
	    unsigned char* buffer = (unsigned char*) str.c_str();
        if((bytes = recv(sokt, buffer, num, MSG_WAITALL)) == -1)
        {
    	    ROS_WARN("recv failed\n");
        }
        else
        {
            cout << "data: " << buffer << endl;
            std_msgs::String msg;
	        stringstream ss;
	        ss << buffer;
	        msg.data = ss.str();
	        pub->publish(msg);
        }
	
	}
	else if(fromType == STD_INT_SEQ && toType == ROS_POINT)
	{
	    string str = "x";
	    const int SIZE_OF_C_INTEGER = sizeof(int);

	    for(size_t i = 0; i < SIZE_OF_C_INTEGER-1; i++)
	    {
	        str.append("x");
	    }

	    unsigned char* sizeCharPtr = (unsigned char*) str.c_str();
	
	    recv(sokt, sizeCharPtr, SIZE_OF_C_INTEGER, MSG_WAITALL);
	    cout << "size: " << sizeCharPtr << endl;
    
	    const char* numCharStar2 = (const char*) sizeCharPtr;
	    int num = atoi(numCharStar2);

	    str.erase();
	    for(size_t i = 0; i < num; i++)
	    {
	        str.append("x");
	    }
	
	    unsigned char* buffer = (unsigned char*) str.c_str();
        if((bytes = recv(sokt, buffer, num, MSG_WAITALL)) == -1)
        {
    	    ROS_WARN("recv failed\n");
        }
        else
        {
            cout << "data: " << buffer << endl;
            string tmpStr((const char*) buffer);
            geometry_msgs::Point msg;
            string xStr;
            string yStr;
            string zStr;
            int delimCount = 0;
            for(size_t i = 0; i < tmpStr.size(); i++)
            {
                if(tmpStr.at(i) != ',')
                {
                    if(delimCount == 0)
                    {
                        char ch = tmpStr.at(i);
                        xStr += ch;
                    }
                    else if(delimCount == 1)
                    {
                        char ch = tmpStr.at(i);
                        yStr += ch;
                    }
                    else if(delimCount == 2)
                    {
                        char ch = tmpStr.at(i);
                        zStr += ch;
                    }
                }
                else
                {
                    delimCount++;
                }
            }
            cout << "x (str): " << xStr << ", y (str): " << yStr << ", z (str): " << zStr << endl;
            msg.x = atof( xStr.c_str() );
            msg.y = atof(yStr.c_str() );
            msg.z = atof(zStr.c_str() );;
	        pub->publish(msg);
        }
	}
	else if(fromType == CV_MAT && toType == ROS_IMAGE)
	{
	    cv_bridge::CvImagePtr cv_ptr;
        cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC(3));

        try
        {
            ;//cv_ptr = cv_bridge::toCvCopy(input/*, sensor_msgs::image_encodings::RGB16*//*RGB8*/);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what() );
        }
    
        int imgSize = img.total() * img.elemSize();
        unsigned char* iptr = img.data;
        int bytes = 0;
    
        if(!img.isContinuous() )
        {
            img = img.clone();
        }
        cout << "waiting for: " << imgSize << " bytes" << endl; //should be 921600
        cv::namedWindow("Client", 1);
    
        if((bytes = recv(sokt, iptr, imgSize, MSG_WAITALL)) == -1)
        {
            cerr << "recv failed, recieved bytes = " << bytes << endl;
        }
    
        cv::imshow("Client", img);
        cv::waitKey(3);
    
        cv_ptr->image = img;
        pub->publish(cv_ptr->toImageMsg() );
	}
	//else if...
	
	
	
}


void To_ros_ip::finish()
{
    //close socket
    close(sokt);
    //close(serverPort);
}


Publisher* To_ros_ip::getPublisher()
{
    return pub;
}


string* To_ros_ip::toString()
{
    ;
}


To_ros_ip::~To_ros_ip()
{
    ;
}
