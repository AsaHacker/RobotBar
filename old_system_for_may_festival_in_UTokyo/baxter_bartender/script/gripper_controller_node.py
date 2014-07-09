#!/usr/bin/env python
import argparse

import rospy

import baxter_interface
import baxter_external_devices
import functools
import std_msgs

def string_command_callback(gripper, msg):
    if ( msg.data == "calib" ):
        gripper.calibrate()
    elif ( msg.data == "open" ):
        gripper.open()
    elif ( msg.data == "close" ):
        gripper.close()

def string_robot_command_callback(rs, msg):
    if ( msg.data == "servo_on" ):
        rs.enable()
    elif ( msg.data == "servo_off" ):
        rs.disable()

def main():
    rospy.init_node("gripper_controller_node")
    ##
    rs = baxter_interface.RobotEnable()
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left')
    right = baxter_interface.Gripper('right')
    done = False
    ## rs.enable()
    rospy.Subscriber("gripper_controller_node/larm/command/string", std_msgs.msg.String, functools.partial(string_command_callback, left))
    rospy.Subscriber("gripper_controller_node/rarm/command/string",std_msgs.msg.String,functools.partial(string_command_callback, right))
    rospy.Subscriber("posture_controller_node/body/command/string", std_msgs.msg.String, functools.partial(string_robot_command_callback,rs))
    def clean_shutdown():
        if not init_state:
            print("Disabling robot...")
            rs.disable()
        print("Exiting example.")
    rospy.on_shutdown(clean_shutdown)
    rospy.Rate(5)
    ##
    rospy.spin()

if __name__ == '__main__':
    main()
