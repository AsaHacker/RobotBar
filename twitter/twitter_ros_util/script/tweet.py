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

def check_replies():
    global prev_reply_ids
    buf = api.GetReplies()
    if prev_reply_ids:
        for rep in buf:
            if rep.user.screen_name:
                print rep.user.screen_name + " says " + rep.text
                if not rep.id in prev_reply_ids and key["username"] in rep.text:
                    print "pub"
                    rep_pub.publish(String(rep.in_reply_to_screen_name + "\n" + rep.text))
    else:
        print "empty"
    prev_reply_ids = map(lambda x:x.id, buf)

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
    prev_reply_ids = []
    rep_pub = rospy.Publisher('twitter/reply/string', String)
    rospy.Subscriber("twitter/tweet/string", String, tweet)
    ## rospy.spin()
    while(not rospy.is_shutdown()):
        check_replies()
        rospy.sleep(30)
