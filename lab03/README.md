## Lab 03 - ROS Workspace and Filesystem

* Create and build a catkin workspace

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

* Familiarize yourself with [ROS filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem). Locate the 'geometry_msgs' package.

* Create a catkin package called 'cs476', which depends on std_msgs and rospy

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg cs476 std_msgs rospy
```

* Building a catkin workspace and sourcing the setup file

```
$ cd ~/catkin_ws
$ catkin_make
```

* Note that whenever you build a new package, you will need to update your environment

```
$ source ~/catkin_ws/devel/setup.bash
```
