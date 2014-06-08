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
                smach.StateMachine.add('MOVE_NEAR_PLATFORM',
                                       create_move_state_in_frame('aero/starting_location', 5, 0, math.pi),
                                       transitions={'succeeded':'SPIRAL_NEAR_PLATFORM',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('SPIRAL_NEAR_PLATFORM', create_spiral_search_state_in_frame('aero/starting_location', 5, 0, math.pi, 20, 4, 4, math.pi/4),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
            smach.Concurrence.add('DRIVE_WHILE_DETECTING', drive_search_state)

        smach.StateMachine.add('SEARCH_FOR_PLATFORM', search_concurrence,
                               transitions={'succeeded':'WAIT_BEFORE_NAV_IN_FRONT_OF_PLATFORM',
                                            'aborted':'aborted',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('WAIT_BEFORE_NAV_IN_FRONT_OF_PLATFORM', DelayState(2.0),
                               transitions={'succeeded':'MOVE_IN_FRONT_OF_PLATFORM'})

            
        smach.StateMachine.add('MOVE_IN_FRONT_OF_PLATFORM',
                               create_move_state_in_frame('aero/starting_location', 4, 0, math.pi),
                               transitions={'succeeded':'SHUTTER_LASER',
                                            'aborted':'aborted',
                                            'preempted':'preempted'})
        
        smach.StateMachine.add('SHUTTER_LASER', create_shutter_laser_state(),
                               transitions={'succeeded':'NAV_ONTO_PLATFORM', 'failed': 'NAV_ONTO_PLATFORM'})
        smach.StateMachine.add('NAV_ONTO_PLATFORM',
                               create_move_state_in_frame('aero/starting_location', 0, 0, math.pi),
                               transitions={'succeeded':'UNSHUTTER_LASER',
                                            'aborted':'aborted',
                                            'preempted':'preempted'})
        smach.StateMachine.add('UNSHUTTER_LASER', create_unshutter_laser_state(),
                               transitions={'succeeded':'succeeded'})
    return drive_home_state
