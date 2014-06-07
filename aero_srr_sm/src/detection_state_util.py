#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
from arm_state_util import *
import actionlib
from jaco_msgs.msg import *
from jaco_msgs.srv import *
import tf
from geometry_msgs.msg import *
from nav_msgs.msg import *

def detect_monitor_cb(ud, msg):
    ud['detection_msg'] = msg
    return False

def create_detect_state():
    return smach_ros.MonitorState("/aero/lower_stereo/object_detection/filtered", PoseStamped, detect_monitor_cb, output_keys = ['detection_msg'])

class DelayState( smach.State ):
    def __init__( self, delay = 3.0 ):
        smach.State.__init__(self,outcomes=['succeeded', 'aborted'])
        self.delay = delay

    def execute( self, userdata ):
        rospy.sleep( self.delay )
        return 'succeeded'

class CheckNearPrecacheState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['far', 'near', 'close', 'aborted'], input_keys=['detection_msg'], output_keys=['detection_msg'])
        self.tf_listener = tf.TransformListener()
    def execute(self, userdata):
        object_location = userdata['detection_msg']
        try:
            self.tf_listener.waitForTransform('aero/base_footprint', object_location.header.frame_id, object_location.header.stamp, rospy.Duration(1))
        
            object_location_base = self.tf_listener.transformPose('aero/base_footprint', object_location)
            object_position = object_location_base.pose.position

            print('Checking base relative location ', str(object_position) )
            if object_position.x > 2:
                return 'far'
            if object_position.x > 0.85:
                return 'near'
            return 'close'
        except tf.Exception:
            return 'aborted'
