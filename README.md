## Edwin\_Stereo

This package is specifically for stereoscopic vision for [Edwin](http://github.com/olinrobotics/edwin), a Eusocial Coworker Robot Project for Olin Robotics Lab.

## Building

This package has several dependencies that are not part of the standard ROS Indigo Ecosystem.

- OpenCV 3 (3.2.0)

## RUNNING THE DEMO

```bash
roscore
roslaunch edwin_moveit_config hardware.launch
roslaunch edwin_moveit_config rosserial.launch
roslaunch edwin_moveit_config real.launch
roslaunch edwin_moveit_config move_group_interface.launch
roslaunch edwin_stereo demo.launch
#rosrun rqt_reconfigure rqt_reconfigure
```
