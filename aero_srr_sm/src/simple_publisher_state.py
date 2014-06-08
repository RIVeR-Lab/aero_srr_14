#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach

class SimplePublisherState(smach.State):
    def __init__(self, topic, msg_type, msg, latch = False, io_keys=[]):
        smach.State.__init__(self, outcomes=['succeeded'], io_keys=io_keys)
        self.msg = msg;
        self.pub = rospy.Publisher(topic, msg_type, latch = latch)

    def execute(self, userdata):
        self.pub.publish(self.msg)
        return 'succeeded'
        
