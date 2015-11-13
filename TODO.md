#TODO

- add callback2 and fromTcp in RosClient.*
- Fix drift
- look into speech time-out
- http://answers.ros.org/question/31944/rosrun-node_name/
- finish or remove map rotation

**See**
For for modified turtlebot_teleop_key stuff:
https://github.com/turtlebot/turtlebot/blob/indigo/turtlebot_teleop/scripts/turtlebot_teleop_key
http://pharos.ece.utexas.edu/wiki/index.php/Teleoperate_iRobot_Create_using_ROS_Hydro
http://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20robot%20base%20controllers%20to%20drive%20the%20robot#CA-1426c28b03091e68ce47656633f2c51ad6982d68_67

nav: http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map

use this in Ros client/server translators for string <--> Point:
```
#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>

using namespace std;

int main()
{
    float num1 = 1234.5678912345;
    float num2 = 9876.54321;
    ostringstream ss;
    
    ss << setprecision(10) << num1;
    string tmpStr1;
    tmpStr1 = ss.str();

    ss.str(string() );
    ss << setprecision(10) << num2;
    string tmpStr2;
    tmpStr2 = ss.str();
    
    string str = tmpStr1;
    str.append("|");
    str.append(tmpStr2);
   
    cout << "tmpStr1: " << tmpStr1 << endl;
    cout << "tmpStr2: " << tmpStr2 << endl;
    cout << "str: " << str << endl;
    //cout << sizeof(str) << endl;

    return 0;
}
```
