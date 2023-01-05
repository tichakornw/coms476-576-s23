#!/usr/bin/env python
import rospy
from cs476.msg import FloatArray


def callback(msg):
    msg_str = " ".join(map(str, msg.nums))
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", msg_str)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("listener", anonymous=True)
    msg = rospy.wait_for_message("chatter", FloatArray)
    callback(msg)


if __name__ == "__main__":
    listener()
