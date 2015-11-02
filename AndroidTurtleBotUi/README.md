#Android Video IP Application
*Android interface for controlling Rethink Baxter robot*

*Author/Maintainer:* James Kuczynski,  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Undergraduate Researcher,  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Robotics Laboratory][5],  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[University of Massachusetts Lowell][4].  
*Email:* jkuczyns@cs.uml.edu

=====


###**Index**

- [Project Synopsis](#project-synopsis)
- [Dependencies](#dependencies)
- [Install](#install)
- [Future Work](#future-work)


###**Project Synopsis**

This package provides a UI to control a robot (specifically, [Rethink Baxter][6]) with a Android tablet or smartphone.


###**Dependencies**

- [ROS][1]
- [OpenCV][2]
- [PCL][3] >= 1.8
- [pcl_ros][3] Jade
- A [Rethink Baxter][6] robot
- An Android device (or emulator)
- [ros_ip_transform][7]
- [object_separator][8]


###**Install**

download https://github.com/WPI-RAIL/async_web_server_cpp
and https://github.com/RobotWebTools/web_video_server, and run ```catkin_make``` on them
To reduce the TCP buffer size:
async_web_server -> http_connetction.cpp --> HttpConnection::write(.....) --> pending_write_buffers and pending_write_ressources
(see https://github.com/RobotWebTools/web_video_server/issues/13)

To execute the individual nodes, run:
```
roslaunch openni2_launch openni2_launch                 # start sensor
rosrun web_video_server web_video_server                # send image stream over http; make sure you are running
                                                        # the version you compiled from source, NOT the version
                                                        # which comes default with ROS indigo
rosrun ros_ip_transform RosServer __name:=left_cam      # send a frame from the left camera
rosrun ros_ip_transform RosServer __name:=right_cam     # send a frame from the right camera
rosrun ros_ip_transform DataCom                         # recieve point, send bounding-box image
 
rosrun object_separator object_separator                # do lccp object segmentation
rosrun object_separator generate_boundingbox            # generate bounding box around object--not implmented yet
```


###**Future Work**

The idea of controling a robot via a smartphone or tablet is not an entirely new one; however, little research has been done in this area.  Therefore, it would be interesting to modify this package to make it more generalized to work with other robots.  I will make these changes within the next year.

###**TODO**

1) Enable BOTH cameras.
2) see link for handling scrolling and still getting correct location on webview:
        http://stackoverflow.com/questions/10808387/android-getting-exact-scroll-position-in-listview
3) Finish dealing with the backend stuff (last .java file in navigator).

[1]: http://www.ros.org/
[2]: http://opencv.org/
[3]: http://pointclouds.org/
[4]: http://www.uml.edu/
[5]: http://robotics.cs.uml.edu/
[6]: http://www.rethinkrobotics.com/baxter/
[7]: https://github.com/DeepBlue14/image_transport_inverse
[8]: https://github.com/uml-robotics/object_separator


