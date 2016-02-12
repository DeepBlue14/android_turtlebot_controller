#include "DataCom.h"
//works

int DataCom::m_comm_fd;


DataCom::DataCom()
{
    globalCounter = 0;
    m_count = 0;
    pub = new Publisher();
    updatePub = new Publisher();
    
}


bool DataCom::connect2Client(int port)
{
    int listen_fd, comm_fd;
 
    struct sockaddr_in servaddr;
 
    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
 
    bzero( &servaddr, sizeof(servaddr));
 
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htons(INADDR_ANY);
    servaddr.sin_port = htons(50001);
 
    bind(listen_fd, (struct sockaddr *) &servaddr, sizeof(servaddr));
 
    listen(listen_fd, 10);
 
    m_comm_fd = comm_fd = accept(listen_fd, (struct sockaddr*) NULL, NULL);
    cout << "Successfully connected to client" << endl;
    
    return true;
}


void DataCom::publishTcp(const sensor_msgs::ImageConstPtr& msg)
{
    char str[100];
    bzero(str, 100);
    cout << "blocking before read..." << endl;
    ssize_t rretn = read(m_comm_fd, str, 100);
    cout << "read: " << str << endl;
    
    int delimCounter = 0;
    string xStr, yStr;
    int x = 0, y = 0;
    
    //extract chars from string
    for(size_t i = 0; i < 100; i++)
    {
        if(str[i] == '|' && delimCounter == 0)
        {
            delimCounter++;
        }
        else if(str[i] == '|' && delimCounter == 1)
        {
            delimCounter++;
        }
        else if(str[i] == '|' && delimCounter == 2)
        {
            break;
        }
        else if(str[i] != '|' && delimCounter == 1)
        {
            xStr += str[i];
        }
        else if(str[i] != '|' && delimCounter == 2)
        {
            yStr += str[i];
        }
            
    }
    
    x = atoi(xStr.c_str());
    y = atoi(yStr.c_str());
    cout << "x: " << x << ", y: " << y << endl;
    
    
    
    /*geometry_msgs::Point pixelPoint;
    pixelPoint.x = x;
    pixelPoint.y = y;
    
    pub->publish(pixelPoint);*/
    geometry_msgs::Twist cmd;
    if(x == 111)
    {
        //forward
        cmd.linear.x = 0.1;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
    }
    else if(x == 112)
    {
        //forward
        cmd.linear.x = 0.2;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
    }
    else if(x == 221)
    {
        //backward
        cmd.linear.x = -0.1;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
    }
    else if(x == 222)
    {
        //backward
        cmd.linear.x = -0.2;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
    }
    else if(x == 333)
    {   //rotate left
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.5;
    }
    else if(x == 444)
    {
        //rotate right
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = -0.5;
    }
    else if(y == 000)
    {
        //sc->say("Hello, my  name is Blue.  My I P address is ten, zero, three, sixteen.  Not really, I'm lying.");
    }
    
    
    
    if(x == 000 || x == 111 || x == 112 || x == 221 || x == 222 || x == 333 || x == 444)
    {
        pub->publish(cmd);
        //cout << "dist: " << 
        globalCounter++;
        if(globalCounter > 25)
        {
            std_msgs::String myInt;
            myInt.data = "1.68943";
            updatePub->publish(myInt);
            globalCounter = 0;
        }
    }
}


Publisher* DataCom::getPublisher()
{
    return pub;
}


Publisher* DataCom::getUpdatePublisher()
{
    return updatePub;
}


DataCom::~DataCom()
{
    close(remoteSocket);
}
