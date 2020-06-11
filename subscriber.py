#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " subscribed to '%s'", data.data)
    
def subscriber():
    rospy.init_node('SubscriberNode', anonymous=True)
    rospy.Subscriber('PubSubTopic', String, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()