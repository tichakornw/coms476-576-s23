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