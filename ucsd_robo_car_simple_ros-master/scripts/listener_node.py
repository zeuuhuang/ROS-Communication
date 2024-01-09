#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    sec = rospy.Subscriber("SecurityCode", String, callback)
    alert = rospy.Subscriber("CrashAlert", String, callback)
    stopSignal = rospy.Publisher('ThrottleStop', String, queue_size=10)
    

    if sec == 'I am Autonomous Prime, Autobots, roll out!':
        if alert == 'stop':
            stop()
    rospy.spin()

def stop():
    stopSignal.publish('Stop')

if __name__ == '__main__':
    listener()
