## Lab 04 - Launch File and ROS msg

* Make a 'launch' directory in your cs476 package and create a launch file called talker_listener.launch with the following content

```
<launch>
  <node name="listener" pkg="roscpp_tutorials" type="listener" output="screen"/>
  <node name="talker" pkg="roscpp_tutorials" type="talker" output="screen"/>
</launch>
```

* roslaunch the launch file.

```
$ roslaunch talker_listener.launch
```

* Make a 'msg' directory in your cs476 package and define a new ROS msg called FloatArray.msg with a variable-length array of type float32

```
float32[] nums
```

* Open package.xml, and uncomment these two lines

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

* Open CMakeLists.txt and add the message_generation dependency to the find_package call, which already exists in your CMakeLists.txt so that you can generate messages. The find_package call should look like

```
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
```

* Also, in CMakeLists.txt, uncomment the relevant sections so that you have something like this

```
add_message_files(
  FILES
  FloatArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
```

* Now that we have made some new messages, we need to make our package again

```
$ roscd cs476
$ cd ../..
$ catkin_make
```

* Make sure that ROS can see your new ROS msg

```
$ rosmsg show cs476/FloatArray
```