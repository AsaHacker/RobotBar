#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Empty

user_name_pub = ""

if __name__ == '__main__':
  rospy.init_node('user_prompt', anonymous=True)
  user_name_pub = rospy.Publisher('user_db_observer/set_user_name', String)
  while(not rospy.is_shutdown()):
    rospy.sleep(1)
    input_line = raw_input()
    user_name_pub.publish(String(input_line))

