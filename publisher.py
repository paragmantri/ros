#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('PubSubTopic', String, queue_size=10)
    rospy.init_node('PublisherNode', anonymous=True)
    rate = rospy.Rate(0.8)
    for i in range(1,1000):
        if not rospy.is_shutdown():
            rospy.loginfo(str(i))
            pub.publish(str(i))
            rate.sleep()



if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass