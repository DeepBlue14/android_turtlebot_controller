#Android TurtleBot Ui
*Android interface for controlling a robot*

*Author/Maintainer:* James Kuczynski,  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Undergraduate Researcher,  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Robotics Laboratory][4],  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[University of Massachusetts Lowell][3].  
*Email:* jkuczyns@cs.uml.edu

=====


###**Index**

- [Project Synopsis](#project-synopsis)
- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [Install](#install)
- [Future Work](#future-work)


###**Project Synopsis**

This package provides a UI to control a mobile robot with a Android tablet or smartphone.  This particular project uses a TurtleBot (gen. 1 with a Create base).  The backend consists of the programs which the robot must run to communicate with the Android device.

*WARNING*: This project is currently in the pre-alpha stage.*


###**Introduction**

*TBA...*


###**Dependencies**

**Android Side**
- [OpenCV][2]

**Robot Side**
- [ROS][1] indigo
- [OpenCV][2]

###**Install**

*TBA...*

**Run**
- Power on the Create
- Plug in both the kinect and Create USBs.
```
roslaunch turtlebot_bringup minimal.launch --screen             # starts the robot base.  If it fails, make sure that the Create is on,
                                                                # and that the robot-brain is connected to the internet.  You may have
                                                                # to unplug and re-plug in the Create.
roslaunch turtlebot_dashboard turtlebot_dashboard.launch        # starts a UI to display status info
roslaunch turtlebot_bringup 3dsensor.launch                     # starts the camera
rosrun ip_translator RosServer                                  # sends camera feed from the robot's 3D camera 
rosrun ip_translator DataCom                                    # recieves string data from the Android device
#start the Android app

```

###**Future Work**

*TBA...*

[1]: http://www.ros.org/
[2]: http://opencv.org/
[3]: http://www.uml.edu/
[4]: http://robotics.cs.uml.edu/
