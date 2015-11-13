#include "RosServer.h"


int RosServer::m_comm_fd;


RosServer::RosServer()
{
    m_count = 0;
    pub = new Publisher();
}


void RosServer::callback(const sensor_msgs::ImageConstPtr& msg)
{
    this->publishTcp(msg);
}


void RosServer::callback2(const geometry_msgs::Point msg)
{
    cout << "in callback2" << endl;
    this->publishTcp(msg);
}


bool RosServer::connect2Client(int port)
{
    localSocket = socket(AF_INET, SOCK_STREAM, 0);
    if(localSocket == -1)
    {
        perror("socket() call failed!!");
    }
    m_port = port;
    addrLen = sizeof(struct sockaddr_in);
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = htons(m_port);
    
    if(bind(localSocket, (struct sockaddr*)&localAddr, sizeof(localAddr)) < 0)
    {
        perror("Can't bind() socket");
        exit(1);
    }
    
    listen(localSocket, 1);
    
    cout << "Waiting for connections...\n"
              << "Server Port: " << m_port << endl;
    
    remoteSocket = accept(localSocket, (struct sockaddr *)&remoteAddr, (socklen_t*)&addrLen);
    if(remoteSocket < 0)
    {
        perror("Accept failed!");
        exit(1);
    }
    cout << "Connection accepted; initiating data transfer" << endl;
    
    return true;
}


void RosServer::publishTcp(const sensor_msgs::ImageConstPtr& msg)
{
    if(m_count > 10) // Reject the first 10, since some of the first images may contain discolorations
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat img, imgGray;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what() );
        }
    
        img = cv_ptr->image;
        //cv::imshow("Server", img);
        //cv::waitKey(3);
    
        imgGray = img.clone();
        //cvtColor(img, imgGray, CV_BGR2GRAY);
        //cv::imshow("About To Send", imgGray);
        //cv::waitKey(3);
    
        int imgSize = imgGray.total() * imgGray.elemSize();
        //cout << "sending: " << imgSize << " bytes" << endl;
        int bytes = 0;
        int key;
    
        if((bytes = send(remoteSocket, imgGray.data, imgSize, 0)) < 0)
        {
            cerr << "bytes = " << bytes << endl;
        }
    }
    else
    {
        m_count++;
        cout << "Skipping tcp transfer" << endl;
    }

}


void RosServer::publishTcp(const geometry_msgs::Point& msg)
{
    
    cout << "about to send" << endl;
    string str = "HelloWorld";
    int imgSize = str.size();
    char* iptrTmp = (char*) str.c_str();
    unsigned char* iptr = (unsigned char*) iptrTmp;
    int bytes = 0;
    
    if((bytes = send(remoteSocket, iptr, imgSize, 0)) < 0)
    {
        cerr << "bytes = " << bytes << endl;
    }
    else
    {
        cout << "Sent msg" << endl;
    }

}


Publisher* RosServer::getPublisher()
{
    return pub;
}


RosServer::~RosServer()
{
    close(remoteSocket);
}
