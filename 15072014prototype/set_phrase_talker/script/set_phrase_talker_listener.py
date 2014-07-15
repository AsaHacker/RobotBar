#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty

pub = ""

def callback(data):
  print data.data
  pub.publish(Empty())

def listener():
  rospy.init_node('set_phrase_talker_listener', anonymous=True)
  rospy.Subscriber("event_topic", String, callback)
  rospy.spin()

if __name__ == '__main__':
  pub = rospy.Publisher('user_db_observer/levelup_event', Empty)
  listener()
