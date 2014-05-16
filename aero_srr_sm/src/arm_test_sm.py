#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
import smach_ros
from pprint import pprint
from actionlib import *
from actionlib.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from move_base_msgs.msg import *
import math
import tf2_ros
import tf

from fake_state import *
from detection_pickup_state import *
from move_state_util import *
from arm_state_util import *


def child_term_cb(outcome_map):
    if outcome_map['WAIT_FOR_DETECTION'] == 'invalid':
        return True
    elif outcome_map['WAIT_DETECT_FAKE'] == 'succeeded':
        return True
    elif outcome_map['WAIT_DETECT_FAKE'] == 'preempted':
        return True
    elif outcome_map['WAIT_DETECT_FAKE'] == 'aborted':
        return True
    else:
        return False

def out_cb(outcome_map):
    if outcome_map['WAIT_FOR_DETECTION'] == 'invalid':
        return 'succeeded'
    elif outcome_map['WAIT_DETECT_FAKE'] == 'succeeded':
        return 'failed'
    elif outcome_map['WAIT_DETECT_FAKE'] == 'preempted':
        return 'failed'
    elif outcome_map['WAIT_DETECT_FAKE'] == 'aborted':
        return 'failed'
    else:
        return 'failed'
    
def monitor_cb(ud, msg):
    print msg
    ud['detection_msg'] = msg
    return False
    
# main
def main():
    rospy.init_node('arm_test_sm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', FakeState(),
                               transitions={'succeeded':'WAIT_DETECT',
                                            'aborted':'failed',
                                            'preempted':'failed'})

        wait_detect_concurrence = smach.Concurrence(outcomes=['succeeded', 'failed'],
                                            default_outcome='failed',
                                            output_keys=['detection_msg'],
                                            child_termination_cb=child_term_cb,
                                            outcome_cb=out_cb)
        with wait_detect_concurrence:
                smach.Concurrence.add('WAIT_FOR_DETECTION',
                                      smach_ros.MonitorState("/aero/lower_stereo/object_detection/filtered", Odometry, monitor_cb, output_keys = ['detection_msg']))
                #smach.Concurrence.add('WAIT_DETECT_FAKE', FakeState())
        smach.StateMachine.add('WAIT_DETECT', wait_detect_concurrence,
                               transitions={'succeeded':'DETECTION_PICKUP',
                                            'failed':'failed'})

        smach.StateMachine.add('DETECTION_PICKUP', DetectionPickupState(),
                               transitions={'succeeded':'succeeded',
                                            'aborted':'failed',
                                            'preempted':'failed'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    #rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
