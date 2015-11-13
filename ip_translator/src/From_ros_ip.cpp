#include "From_ros_ip.h"


int From_ros_ip::m_comm_fd;


From_ros_ip::From_ros_ip()
{
    ;
}


int From_ros_ip::connecter(int port)
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


void From_ros_ip::publish(const std_msgs::Bool::ConstPtr& msg)
{
    bool stdInt = msg->data;
    ssize_t r;
    r = send(m_comm_fd, &stdInt, sizeof(bool), 0);
}


void From_ros_ip::publish(const std_msgs::Byte::ConstPtr& msg)
{
    bool stdInt = msg->data;
    ssize_t r;
    r = send(m_comm_fd, &stdInt, sizeof(bool), 0);
}


void From_ros_ip::publish(const std_msgs::ByteMultiArray::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::Char::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::ColorRGBA::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::Duration::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::Empty::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::Float32::ConstPtr& msg)
{
    int32_t stdInt = msg->data;
    ssize_t r;
    r = send(m_comm_fd, &stdInt, sizeof(int32_t), 0);
}


void From_ros_ip::publish(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::Float64::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::Int16::ConstPtr& msg)
{
    int16_t stdInt = msg->data;
    ssize_t r;
    r = send(m_comm_fd, &stdInt, sizeof(int16_t), 0);
}


void From_ros_ip::publish(const std_msgs::Int16MultiArray::ConstPtr& msg)
{

}


void From_ros_ip::publish(const std_msgs::Int32::ConstPtr& msg)
{
    cout << "about to send data" << endl;
    int32_t stdInt = msg->data;
    ssize_t r;
    r = send(m_comm_fd, &stdInt, sizeof(int32_t), 0);
    cout << "sent: " << r << " bytes" << endl;
}


void From_ros_ip::publish(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::Int64::ConstPtr& msg)
{
    int64_t stdInt = msg->data;
    ssize_t r;
    r = send(m_comm_fd, &stdInt, sizeof(int64_t), 0);
}


void From_ros_ip::publish(const std_msgs::Int64MultiArray::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::Int8::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::MultiArrayDimension::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::MultiArrayLayout msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::String::ConstPtr& msg)
{
    string str = msg->data;
    stringstream ss;
    ss << (str.size()+1);
    string tmpStr = ss.str();
    unsigned char* sizeAsCharPtr = (unsigned char*) tmpStr.c_str();
    unsigned char* dataCharPtr = (unsigned char*) str.c_str();

    int bytes = 0;
    
    ssize_t r;
    r = send(m_comm_fd, sizeAsCharPtr, sizeof(int), 0); // send size of data 
    r = send(m_comm_fd, dataCharPtr, str.size()+1, 0); //send data
}


void From_ros_ip::publish(const std_msgs::Time::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::UInt16::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::UInt32::ConstPtr& msg)
{
    uint32_t stdInt = msg->data;
    ssize_t r;
    r = send(m_comm_fd, &stdInt, sizeof(uint32_t), 0);
}


void From_ros_ip::publish(const std_msgs::UInt32MultiArray::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::UInt64::ConstPtr& msg)
{
    uint64_t stdInt = msg->data;
    ssize_t r;
    r = send(m_comm_fd, &stdInt, sizeof(uint64_t), 0);
}


void From_ros_ip::publish(const std_msgs::UInt64MultiArray msg)
{
    ;
}


void From_ros_ip::publish(const std_msgs::UInt8::ConstPtr& msg)
{
    uint8_t stdInt = msg->data;
    ssize_t r;
    r = send(m_comm_fd, &stdInt, sizeof(uint8_t), 0);
}


void From_ros_ip::publish(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    ;
}


void From_ros_ip::publish(const geometry_msgs::Point msg, char delim)
{
    stringstream ss;
    ss << msg.x;
    string tmpStr;// = "9,";
    tmpStr.append(ss.str() );
    string s = &delim;
    s.erase(1, 1);
    tmpStr.append(s);
    ss.str(std::string() );
    ss << msg.y;
    tmpStr.append(ss.str() );
    tmpStr.append(s);
    ss.str(std::string() );
    ss << msg.z;
    tmpStr.append(ss.str() );
    
    ss.str(std::string() );
    ss << tmpStr.size()+1;
    
    unsigned char* sizeAsCharPtr = (unsigned char*) ss.str().c_str();
    unsigned char* dataCharPtr = (unsigned char*) tmpStr.c_str();
    
    cout << "size: " << sizeAsCharPtr << endl;

    ssize_t r;
    r = send(m_comm_fd, sizeAsCharPtr, sizeof(int), 0); // send size of data
    r = send(m_comm_fd, dataCharPtr, tmpStr.size()+1, 0); //send data
}


void From_ros_ip::publish(const sensor_msgs::CompressedImageConstPtr& msg)
{
    cout << "jpg size: " << msg->data.size() << endl;
    stringstream ss;
    ss << msg->data.size();
    string sizeStr = ss.str();
    unsigned char* sizeCharPtr = (unsigned char*) sizeStr.c_str();
    //cout << "\tsending type: " << msg->format << endl;
    //printf("size of uint8_t: %ld ", sizeof(uint8_t));
    
    ssize_t r;
    r = send(m_comm_fd, sizeCharPtr, sizeStr.size(), 0); 
    r = send(m_comm_fd, &(msg->data), msg->data.size(), 0);
}


void From_ros_ip::publish(const sensor_msgs::ImageConstPtr& msg)
{
    //convert to OpenCV type- - - - - - - - - - - - - - - - - -
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img, imgGray;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg/*, sensor_msgs::image_encodings::RGB16*//*RGB8*/); // TYPE_32SC4
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what() );
    }
    
    img = cv_ptr->image;
    cv::imshow("Server", img);
    cv::waitKey(3);
    
    imgGray = img.clone();
    //cvtColor(img, imgGray, CV_BGR2GRAY);
    //cv::imshow("About To Send", imgGray);
    //cv::waitKey(3);
    
    int imgSize = imgGray.total() * imgGray.elemSize();
    //cout << "sending: " << imgSize << " bytes" << endl;
    int bytes = 0;
    int key;
    
    if((bytes = send(m_comm_fd, imgGray.data, imgSize, 0)) < 0)
    {
        cerr << "bytes = " << bytes << endl;
    }
}


void From_ros_ip::finish()
{
    //close(m_comm_fd);
}


string* From_ros_ip::toString()
{
    string* tmp = new string();
    
    return tmp;
}


From_ros_ip::~From_ros_ip()
{
    ;
}
