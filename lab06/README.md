## Lab 06 - Coordinates and Transformation

### ROS tf2 Wiki

[tf2 - ROS Wiki](http://wiki.ros.org/tf2)

`tf2` is the second generation of the transform library, which lets the user keep track of multiple coordinate frames over time. `tf2` maintains the relationship between coordinate frames in a tree structure buffered in time, and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time.

### Related tools

* Print information about the current transform tree

  `rosrun tf tf_monitor`

* Print information about the transform between two frames

  `rosrun tf tf_echo src_frame tar_frame`

* View frames in a pdf file

  `rosrun tf view_frames`

* Visualize frames in RViz

  1. `rosrun rviz rviz`
  2. Select `world` as the fixed frame
  3. Add `tf` to visualizer

* Check `/tf` and `/tf_static` topic

  `rostopic info /tf`

  `rostopic info /tf_static`

* Visualize ROS node graph (You can check the subscriber and publisher of `/tf` and `/tf_staic` here)

  `rosrun rqt_graph rqt_graph`

### Exercise

This exercise will give you a good idea of what `tf2` can do for you. It shows off some of the tf2 power in a multi-robot example using [turtlesim](http://wiki.ros.org/turtlesim). This also introduces using [tf2_echo](http://wiki.ros.org/tf2#tf2_echo), [view_frames](http://wiki.ros.org/tf2#view_frames), and [rviz](http://wiki.ros.org/rviz).

You will use the `tf2` library to create three coordinate frames: a world frame, a turtle1 frame, and a turtle2 frame. This tutorial uses a **tf2 broadcaster** to publish the turtle coordinate frames and a **tf2 listener** to compute the difference in the turtle frames and move one turtle to follow the other. You will how to broadcast coordinate frames of a robot to `tf2` and query transformation between frames from `tf2`.

This lab exercise is modified from the following two tutorials.

* C++

  [tf2/Tutorials/Writing a tf2 broadcaster (C++) - ROS Wiki](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29)
  [tf2/Tutorials/Writing a tf2 listener (C++) - ROS Wiki](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29)

* Python

  [tf2/Tutorials/Writing a tf2 broadcaster (Python) - ROS Wiki](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29)

  [tf2/Tutorials/Writing a tf2 listener (Python) - ROS Wiki](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29)

#### Step 1: Meet TurtleSim

`turtlesim` is a tool made for teaching ROS and ROS package. To install the `turtlesim`:

```
$ sudo apt-get install ros-$(rosversion -d)-turtlesim
```

![turtlesim.png](http://wiki.ros.org/turtlesim?action=AttachFile&do=get&target=turtlesim.png)

We will use two of the programs in the `turtlesim` package: `turtlesim_node` and `turtle_teleop_key`.

`turtlesim_node` provides a simple 2D robot simulator. It can simulate the motion of multiple turtles (`turtle1`, `turtle2`, etc.) simultaneously and visualize them in a GUI. It subscribes `turtleX/cmd_vel` topic and publish the `turtleX/pose` topic, where `turtleX` is the name of the turtle robot.

* The `/turtle1/cmd_vel` topic is associated with the message type `geometry_msgs/Twist`. It contains the linear and angular command velocity for `turtleX`. The turtle will execute a velocity command for 1 second then time out. `Twist.linear.x` is the forward velocity, `Twist.linear.y` is the strafe velocity, and `Twist.angular.z` is the angular velocity.

	The `/turtleX/pose` topic is associated with the message type `geometry_msgs/Pose`, which contains the x, y, theta, linear velocity, and angular velocity of turtleX.

    ```
  /turtle1/cmd_vel (topic)  ->  /turtlesim (node)  -> /turtle1/pose (topic)
    ```

* The `turtlesim_node` and the `turtle_teleop_key` node are communicating with each other over a ROS Topic. `turtle_teleop_key` **publishes** the key strokes on the topic `/turtle1/cmd_vel`, while `turtlesim` **subscribes** to the same topic to receive the key strokes.  That is you can use the four arrow keys on your keyboard to send velocity command to the `turtlesim` via this `/teleop_turtle` node.

  ```
  /teleop_turtle (node)  ->  /turtle1/cmd_vel (topic)  ->  /turtlesim (node)  -> /turtle1/pose (topic)
  ```

To start the `turtlesim_node` and `turtle_teleop_key`, in one terminal, run the following command

```
rosrun turtlesim turtlesim_node
```

and in another terminal

````
rosrun turtlesim turtle_teleop_key
````

With this terminal active, you can use the arrow keys to control the turtle.

#### Step 2:

The `turtlesim_node` node can publish turtle's pose but it is not in a `tf2` form and cannot be directly used by other tools like `rviz`. In the second step, we will create a `tf2` broadcaster to publish the pose of a turtle to `tf2`. This node will be used for both two turtles, and thus we will implement it in a more generic way.

```python
#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    # create a tf2 broadcaster
    br = tf2_ros.TransformBroadcaster()

    # create a empty message instance
    t = geometry_msgs.msg.TransformStamped()

    # construct the tf2 message
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # send message to /tf topic
    br.sendTransform(t)

if __name__ == '__main__':
    # initialize the tf2 broadcaster node
    rospy.init_node('tf2_turtle_broadcaster')

    # get the turtle name, which is specified in the roslaunch file
    turtlename = rospy.get_param('~turtle')

    # subscribes the /turtlename/pose topic and handles the message
    # from the topic in the handle_turtle_pose method
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,  # specify the message type
                     handle_turtle_pose,  # handler function
                     turtlename)          # parameter of the handler function

    # keep this ros node running
    rospy.spin()
```



#### Step 3: Listener

In step 3, we'll create a `tf2` listener.  This node basically do three things:

1. let the `turtlesim` spawns another turtle `turtle2`
2. keep checking the transformation between `turtle1` frame and `turtle2` frame
3. send velocity message to `turtle2/cmd_vel`  to let `turtle2` follows the `turtle1`

```python
#!/usr/bin/env python
import rospy
import math

# The tf2_ros package provides an implementation of a
# tf2_ros.TransformListener to help make the task of
# receiving transforms easier.
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    # Here, we create a tf2_ros.TransformListener object.
    # Once the listener is created, it starts receiving
    # tf2 transformations over the wire, and buffers them
    # for up to 10 seconds.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Request the turtlesim to generate another turtle
    # named "turtle2". Here we use the spawner ROS service
    # provided by the turtlesim package. We haven't introduced
    # the ROS service in our lab before, however, it is not a
    # hard to understand concept, and you can simply treat
    # it as a function call.
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2') # x, y, theta, and turtle's name

    # Create a messasge publisher for the "turtle2/cmd_vel" topic
    # You can send velocity message to this topic to control the
    # motion of turtle2.
    turtle_vel = rospy.Publisher('turtle2/cmd_vel',
                                 geometry_msgs.msg.Twist,
                                 queue_size=1)

    # Loop every 100 ms
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Get the transformation from turtle1 to turtle2
            trans = tfBuffer.lookup_transform('turtle2', 'turtle1', rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

		# Create a velocity message for turtle2
        msg = geometry_msgs.msg.Twist()
        # Specify the velocity for turtle2
        # "msg.angular.z" is treated as the orientation of turtle2.
        msg.angular.z = 4 * math.atan2(trans.transform.translation.y,
                                       trans.transform.translation.x)
        # "msg.linear.x" is treated as the speed of turtle2.
        # Here, we let the speed of turtle2 be proportional to the
        # distance between turtle1 and turtle2.
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2
                                       + trans.transform.translation.y ** 2)
		# Send out the message
        turtle_vel.publish(msg)

        # Sleep for 100 ms
        rate.sleep()
```



#### Step 4: Start nodes with a ROS launch file

Your project should be organized as follows:

```
/catkin_ws
   - /src
      - /my_tf2_homework
         - /launch
            - start_demo.launch
         - /scripts
            - turtle_tf2_broadcaster.py
            - turtle_tf2_listener.py
         - /src
         - CMakeLists.txt
         - package.xml
```

We will use a launch file to start all the five ROS nodes.

```xml
<launch>
	<node pkg="turtlesim" type="turtlesim_node" name="sim"/>
	<node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>

	<node name="turtle1_tf2_broadcaster"
		  pkg="my_tf2_homework" type="turtle_tf2_broadcaster.py"
	      respawn="false" output="screen" >
        <!-- set a parameter 'turtle' in the node's private namespace -->
        <!-- rosparam /turtle1_tf2_broadcaster/turtle = turtle1 -->
		<param name="turtle" type="string" value="turtle1" />
	</node>

	<node name="listener"
		  pkg="my_tf2_homework" type="turtle_tf2_listener.py"
          output="screen" />

	<node name="turtle2_tf2_broadcaster"
	      pkg="my_tf2_homework" type="turtle_tf2_broadcaster.py"
	      respawn="false" output="screen" >
        <!-- rosparam /turtle2_tf2_broadcaster/turtle = turtle2 -->
		<param name="turtle" type="string" value="turtle2" />
	</node>
</launch>
```

Run the demo with the command

```
roslaunch my_tf2_homework start_demo.launch
```

Use the keyboard arrow keys to control the turtle1 and the turtle2 will keep moving towards the turtle1. You can also visualize the coordinate frames with the `rviz` tool.

![ ](https://www.programmersought.com/images/316/ddf0fd6a2d0d9619825a69c274650f34.png)

### Reference

* [Euler gimbal lock explaine - YouTube](https://www.youtube.com/watch?v=zc8b2Jo7mno)
* [Quaternions and 3d rotation, explained interactively - YouTube](https://www.youtube.com/watch?v=zjMuIxRvygQ)
