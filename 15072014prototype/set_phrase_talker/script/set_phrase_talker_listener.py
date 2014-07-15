#!/usr/bin/env python

import rospy
from std_msgs.msg import String

pub = ""

def callback(data):
  print data.data
  pub.publish(String(data.data))

def listener():
  rospy.init_node('set_phrase_talker_listener', anonymous=True)
  rospy.Subscriber("event_topic", String, callback)
  rospy.spin()

if __name__ == '__main__':
  pub = rospy.Publisher('levelup_event', String)
  listener()
