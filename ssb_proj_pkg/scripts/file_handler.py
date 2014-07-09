#!/usr/bin/env python
import rospy
from ssb_srvs_msgs.srv import MultiString
from ssb_srvs_msgs.srv import MultiStringResponse
import os
import commands
import wave

def get_files_with_key_in_directory(req):
    str_get = req.data[0]    # data.data = ["directory, filetype"]
    str_split = str_get.split(", ")
    search_dir = "".join((commands.getoutput("echo $SSB_DATA"),
                          "/",
                          str_split[0]))
    filetype = str_split[1];
    candidates = os.listdir(search_dir)
    results = MultiStringResponse()
    for i in xrange(len(candidates)):
        if candidates[i].endswith(filetype):
            results.data.append(candidates[i])
    return results

def get_wav_playtime(req):
    strs_get = req.data    # data.data = ["file0", "file1", ...]
    results = MultiStringResponse()
    search_dir = "".join((commands.getoutput("echo $SSB_DATA"), "/sound/voice/"))
    for i in xrange(len(strs_get)):
        fileurl = "".join((search_dir, strs_get[i]))
        wf = wave.open(fileurl, "r")
        results.data.append(str(float(wf.getnframes())/wf.getframerate()))
    return results

if __name__ == "__main__":
    rospy.init_node('file_handler')
    sfile = rospy.Service('py_file', MultiString, get_files_with_key_in_directory)
    swav = rospy.Service('py_wav', MultiString, get_wav_playtime)
    rospy.spin()
