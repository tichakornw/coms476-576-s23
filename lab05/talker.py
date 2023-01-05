#!/usr/bin/env python
import rospy
from cs476.msg import FloatArray


def talker():
    pub = rospy.Publisher("chatter", FloatArray, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        arr = [i for i in range(10)]
        pub.publish(arr)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
