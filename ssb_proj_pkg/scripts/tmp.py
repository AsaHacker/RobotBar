#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from jsk_gui_msgs.msg import VoiceMessage

#def talker():
#    pub = rospy.Publisher('chatter', String)
#    rospy.init_node('talker', anonymous=True)
#    r = rospy.Rate(10)
#    while not rospy.is_shutdown():
#        str = "hello world"
#        rospy.loginfo(str)
#        pub.publish(str)
#        r.sleep()

#if __name__ == '__main__':
#    try:
#        talker()
#    except rospy.ROSInterruptException: pass

#coding: utf-8
import wave
import pyaudio
import os
import commands
import numpy


def printWaveInfo(wf):
    print "channes:", wf.getnchannels()
    print "sample:", wf.getsampwidth()
    print "hz:", wf.getframerate()
    print "frames:", wf.getnframes()
    print "params:", wf.getparams()
    print "length:", float(wf.getnframes()) / wf.getframerate()
    
if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('file', VoiceMessage)

    data = commands.getoutput("echo $SSB_DATA")
    #os.chdir("".join((data,"/sound/voice")))
    wf = wave.open("/Users/ssb/workspace/data/sound/voice/OTHER_what_can_I_do.wav", "r")
    wf2 = wave.open("/Users/ssb/workspace/data/sound/voice/OTHER_failure2.wav", "r")
    tmp = os.listdir("".join((data,"/sound/voice")))
    tmp2 = []
    for i in xrange(len(tmp)):
        if tmp[i].endswith(".wav"):
            tmp2.append(tmp[i])
    str = VoiceMessage()
    str.texts = tmp2
    pub.publish(str)
    print(tmp2)
        
    printWaveInfo(wf)

    test_str = "start=2.0, end=4.0"
    test_str_2 = test_str.split(", ")
    start_from = float((test_str_2[0].split("="))[1])
    end_to = float((test_str_2[1].split("="))[1])

    p = pyaudio.PyAudio()
    p2 = pyaudio.PyAudio()
    stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)
    stream2 = p2.open(format=p2.get_format_from_width(wf2.getsampwidth()),
                    channels=wf2.getnchannels(),
                    rate=wf2.getframerate(),
                    output=True)

    chunk = 1024
    #data = wf.readframes(chunk)
    begin = start_from*wf.getframerate()
    end = end_to*wf.getframerate()
    data = wf.readframes(int(begin))
    data2 = wf2.readframes(int(begin))
    for i in xrange(int((end - begin)/chunk)):
        data = wf.readframes(chunk)
        data2 = wf2.readframes(chunk)
        decodeddata = numpy.fromstring(data, numpy.int16)
        decodeddata2 = numpy.fromstring(data2, numpy.int16)
        newdata = (decodeddata*2.0).astype(numpy.int16)
        newdata2 = (decodeddata2*2.0).astype(numpy.int16)
        stream2.write(newdata2.tostring())
        stream.write(newdata.tostring())
    #while data != '':
    #    stream.write(data)
    #    data = wf.readframes(chunk)
    stream.close()
    stream2.close()
    p.terminate()
    p2.terminate()
