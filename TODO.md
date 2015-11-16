#TODO

- add callback2 and fromTcp in RosClient.*
- look into speech time-out
- http://answers.ros.org/question/31944/rosrun-node_name/
- finish or remove map rotation

**See**
- http://pharos.ece.utexas.edu/wiki/index.php/Writing_A_Simple_Node_that_Moves_the_iRobot_Create_Robot


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
