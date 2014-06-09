#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
from move_state_util import *
from sensor_state_util import *
import actionlib
from move_base_msgs.msg import *
import tf
import math
from detection_state_util import *


def home_child_term_cb(outcome_map):
    if outcome_map['WAIT_FOR_DETECTION'] == 'invalid':
        return True
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'succeeded':
        return True
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'preempted':
        return True
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'aborted':
        return True
    else:
        return False

def home_out_cb(outcome_map):
    if outcome_map['WAIT_FOR_DETECTION'] == 'invalid':
        return 'succeeded'
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'succeeded':
        return 'aborted'
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'preempted':
        return 'preempted'
    elif outcome_map['DRIVE_WHILE_DETECTING'] == 'aborted':
        return 'aborted'
    else:
        return 'aborted'


def detect_beacon_monitor_cb(ud, msg):
    ud['detection_msg'] = msg
    return False

def create_detect_beacon_state():
    return smach_ros.MonitorState("/aero/upper_stereo/beacon_estimate/filtered", PoseStamped, detect_beacon_monitor_cb, output_keys = ['detection_msg'])



class CheckNearPlatformState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['far', 'near', 'close', 'y_offset', 'aborted'])
        self.tf_listener = tf.TransformListener()
    def execute(self, userdata):
        try:
            time = rospy.Time.now()
            self.tf_listener.waitForTransform('aero/base_footprint', 'aero/in_front_of_platform', time, rospy.Duration(1))
        
            (trans, rot) = self.tf_listener.lookupTransform('aero/base_footprint', 'aero/in_front_of_platform', time)
            dist = math.sqrt(trans[0] ** 2 + trans[1] ** 2 + trans[2] ** 2)
            y_offset = trans[1]

            print('Checking distance: ', dist )
            if dist > 10:
                return 'far'
            if dist > 5:
                return 'near'
            if abs(y_offset) > 1:
                return 'y_offset'
            return 'close'
        except tf.Exception:
            return 'aborted'


def create_drive_home_state():
    drive_home_state = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with drive_home_state:
        search_concurrence = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                                    default_outcome='aborted',
                                                    child_termination_cb=home_child_term_cb,
                                                    outcome_cb=home_out_cb)
        with search_concurrence:
            smach.Concurrence.add('WAIT_FOR_DETECTION', create_detect_beacon_state())

            drive_search_state = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
            with drive_search_state:
                smach.StateMachine.add('MOVE_TOWARD_PLATFORM',
                                       create_move_state_in_frame('aero/in_front_of_platform', 0, 0, math.pi),
                                       transitions={'succeeded':'SPIRAL_NEAR_PLATFORM',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('SPIRAL_NEAR_PLATFORM', create_spiral_search_state_in_frame('aero/in_front_of_platform', 0, 0, math.pi, 20, 4, 4, math.pi/4),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
            smach.Concurrence.add('DRIVE_WHILE_DETECTING', drive_search_state)

        smach.StateMachine.add('SEARCH_FOR_PLATFORM', search_concurrence,
                               transitions={'succeeded':'WAIT_BEFORE_CHECK_NEAR_PLATFORM',
                                            'aborted':'aborted',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('WAIT_BEFORE_CHECK_NEAR_PLATFORM', DelayState(1.0),
                               transitions={'succeeded':'CHECK_NEAR_PLATFORM'})

        smach.StateMachine.add('CHECK_NEAR_PLATFORM', CheckNearPlatformState(),
                               transitions={'far':'MOVE_NEAR_PLATFORM',
                                            'near':'MOVE_CLOSE_PLATFORM',
                                            'close':'SHUTTER_LASER_FOR_MOUNT',
                                            'y_offset':'DRIVE_BACKWARD_FROM_BEACON',
                                            'aborted':'SEARCH_FOR_PLATFORM'})

        smach.StateMachine.add('DRIVE_BACKWARD_FROM_BEACON', create_drive_backward_state(0.5, 3),
                               transitions={'succeeded':'WAIT_BEFORE_CHECK_NEAR_PLATFORM'})
            
        smach.StateMachine.add('MOVE_NEAR_PLATFORM',
                               create_move_state_in_frame('aero/in_front_of_platform', 4, 0, math.pi),
                               transitions={'succeeded':'WAIT_BEFORE_CHECK_NEAR_PLATFORM',
                                            'aborted':'WAIT_BEFORE_CHECK_NEAR_PLATFORM',
                                            'preempted':'preempted'})

        smach.StateMachine.add('MOVE_CLOSE_PLATFORM',
                               create_move_state_in_frame('aero/in_front_of_platform', 0, 0, math.pi),
                               transitions={'succeeded':'WAIT_BEFORE_CHECK_NEAR_PLATFORM',
                                            'aborted':'WAIT_BEFORE_CHECK_NEAR_PLATFORM',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('SHUTTER_LASER_FOR_MOUNT', create_shutter_laser_state(),
                               transitions={'succeeded':'MOUNT_PLATFORM', 'failed': 'MOUNT_PLATFORM'})
        smach.StateMachine.add('MOUNT_PLATFORM',
                               create_move_state_in_frame('aero/starting_location', 0, 0, math.pi),
                               transitions={'succeeded':'UNSHUTTER_LASER_SUCCESS',
                                            'aborted':'UNSHUTTER_LASER_ABORT',
                                            'preempted':'UNSHUTTER_LASER_PREEMPT'})
        smach.StateMachine.add('UNSHUTTER_LASER_ABORT', create_unshutter_laser_state(),
                               transitions={'succeeded':'aborted'})
        smach.StateMachine.add('UNSHUTTER_LASER_PREEMPT', create_unshutter_laser_state(),
                               transitions={'succeeded':'preempted'})

        smach.StateMachine.add('UNSHUTTER_LASER_SUCCESS', create_unshutter_laser_state(),
                               transitions={'succeeded':'succeeded'})
    return drive_home_state
