## Lab 02 - Introduction to Robot Operating System

Run ROS talker-listener demo

* In terminal 1, start a ROS core with

```
$ roscore
```

* In terminal 2, run a talker demo node with

```
$ rosrun roscpp_tutorials talker
```

* In terminal 3, run a demo listener node with

```
$ rosrun roscpp_tutorials listener
```

* In terminal 2, close the talker node (Ctrl + C) and publish your own message with

```
$ rostopic pub /chatter std_msgs/String "data: 'ISU COM S 576 Lab'"
```

* In terminal 4, see the list of active nodes

```
$ rosnode list
```

* Show information about the talker node

```
$ rosnode info /talker
```

* See more information about the chatter topic

```
$ rostopic info /chatter
```

* Check the type of the chatter topic

```
$ rostopic type /chatter
```

* Show the message contents of the topic

```
$ rostopic echo /chatter
```