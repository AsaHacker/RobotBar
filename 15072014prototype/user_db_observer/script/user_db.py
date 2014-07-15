#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Empty

user_name_pub = ""
user_level_pub = ""
user_id = 0
user_db = ""

def level_change_callback(data):
  print data.data
  user_db[user_id]["level"] = data.data
  f = open("user_db.json","w")
  json.dump(user_db,f)
  f.close()

def level_up_callback(data):
  print data.data
  user_db[user_id]["level"] = user_db[user_id]["level"] + 1
  f = open("user_db.json","w")
  json.dump(user_db,f)
  f.close()

def user_name_callback(data):
  tmp = 0
  for d in user_db:
    if ( d["user_name"] == data.data ):
      user_id = tmp
      print "welcome" + " " + data.data
      return
    tmp = tmp + 1
  print "who are you? > " + data.data

def set_listener():
  rospy.init_node('user_db_observer', anonymous=True)
  rospy.Subscriber("user_db_observer/set_level", Int32, level_change_callback)
  rospy.Subscriber("user_db_observer/levelup_event", Empty, level_up_callback)
  rospy.Subscriber("/user_db_observer/set_user_name", String, user_name_callback)
  ## rospy.spin()

if __name__ == '__main__':
  user_name_pub = rospy.Publisher('user_db_observer/user_name', String)
  user_level_pub = rospy.Publisher('user_db_observer/user_level', Int32)
  f = open("user_db.json")
  user_db = json.load(f)
  f.close()
  print(user_db)
  set_listener()
  while(not rospy.is_shutdown()):
    rospy.sleep(1)
    user_name_pub.publish(String(user_db[user_id]["user_name"]))
    user_level_pub.publish(Int32(user_db[user_id]["level"]))

