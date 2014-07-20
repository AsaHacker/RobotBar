#!/usr/bin/env python

import MeCab
import sys
import string

import rospy
import json
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Empty

res_pub = ""
mecab_tagger = MeCab.Tagger (" ".join(sys.argv))

def request_strint_callback(data):
  buf = mecab_tagger.parse(data.data)
  print "---"
  print buf
  res_pub.publish(String(buf))

if __name__ == '__main__':
  rospy.init_node('mecab_node', anonymous=True)
  res_pub = rospy.Publisher('mecab_node/response/string', String)
  rospy.Subscriber("mecab_node/request/string", String, request_strint_callback)
  rospy.spin()
