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