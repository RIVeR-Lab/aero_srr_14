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


def add_state_timeout(delay, nested_state):
    if 'preempted' not in nested_state.get_registered_outcomes():
        raise Exception('State to add timeout to does not have a preempted state')
    outcome_map = dict()
    for outcome in nested_state.get_registered_outcomes():
        outcome_map[outcome] = {'NESTED_STATE': outcome}
    outcome_map['preempted']['DELAY'] = 'succeeded'
    timeout_concurrence = smach.Concurrence(outcomes=nested_state.get_registered_outcomes(),
                                                 default_outcome='preempted',
                                                 input_keys=list(nested_state.get_registered_input_keys()),
                                                 output_keys=list(nested_state.get_registered_output_keys()),
                                                 outcome_map=outcome_map)

    with timeout_concurrence:
        smach.Concurrence.add('DELAY', DelayState(delay))
        smach.Concurrence.add('NESTED_STATE', nested_state)

    return timeout_concurrence
