#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
from arm_state_util import *
import actionlib
import tf
from geometry_msgs.msg import *
from nav_msgs.msg import *


class DelayState( smach.State ):
    def __init__( self, delay ):
        smach.State.__init__(self,outcomes=['succeeded'])
        self.delay = delay

    def execute( self, userdata ):
        rospy.sleep( self.delay )
        return 'succeeded'


def timeout_child_term_cb(outcome_map):
    if outcome_map['DELAY'] == 'succeeded':
        return True
    elif outcome_map['NESTED_STATE']:
        return True
    else:
        return False

def timeout_out_cb(outcome_map):
    if outcome_map['NESTED_STATE']:
        return outcome_map['NESTED_STATE']
    return 'preempted'

def add_state_timeout(delay, nested_state):
    if 'preempted' not in nested_state.get_registered_outcomes():
        raise Exception('State to add timeout to does not have a preempted state')
    timeout_concurrence = smach.Concurrence(outcomes=nested_state.get_registered_outcomes(),
                                                 default_outcome='preempted',
                                                 input_keys=list(nested_state.get_registered_input_keys()),
                                                 output_keys=list(nested_state.get_registered_output_keys()),
                                                 child_termination_cb=timeout_child_term_cb,
                                                 outcome_cb=timeout_out_cb)

    with timeout_concurrence:
        smach.Concurrence.add('DELAY', DelayState(delay))
        smach.Concurrence.add('NESTED_STATE', nested_state)

    return timeout_concurrence
