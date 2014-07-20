#!/usr/bin/env python

import roslib
import rospy
import twitter,yaml,sys
import re, os
from std_msgs.msg import String

def tweet(data):
    m = re.search("[^ ]+.(jpeg|jpg|png|gif)", data.data)
    if m:
        filename = m.group(0)
        message = re.sub(filename,"",data.data).strip()
    else:
        filename = ""
        message = data.data.strip()
    if os.path.exists(filename):
        print "postmedia " + message + "(" + filename + ")"
        api.PostMedia(message,filename)
    else:
        print "postupdate " + message + "(" + filename + ")"
        api.PostUpdate(message)

if __name__ == '__main__':
    rospy.init_node('rostwitter', anonymous=True)
    if os.environ.get("TWITTER_CONF_PATH"):
        root_dir = os.environ.get("TWITTER_CONF_PATH")
    else :
        root_dir = "."
    key = yaml.load(open(root_dir + "/" +"robotbar_twitter.yaml"))
    api = twitter.Api(consumer_key=key["api_key"],
                      consumer_secret=key["api_secret"],
                      access_token_key=key["access_key"],
                      access_token_secret=key["access_secret"])
    rospy.Subscriber("robotbar/tweet/string", String, tweet)
    rospy.spin()
