#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
from std_msgs.msg import *
from std_srvs.srv import *
from simple_publisher_state import *
from smach_ros import ServiceState
from detection_state_util import DelayState

def create_clear_costmap_state():
    return ServiceState('/aero/nav_controller/clear_costmaps', Empty)

def create_laser_state(is_shuttered):
    return SimplePublisherState('/aero/laser_shutter', Bool, Bool(is_shuttered), True)

def create_shutter_laser_state():
    laser_shutter_state = smach.StateMachine(outcomes=['succeeded', 'failed'])
    with laser_shutter_state:
        smach.StateMachine.add('SHUTTER_LASER', create_laser_state(True),
                               transitions={'succeeded':'WAIT_CLEAR_COSTMAP'})
        
        smach.StateMachine.add('WAIT_CLEAR_COSTMAP', DelayState(0.5),
                               transitions={'succeeded':'CLEAR_COSTMAP',
                                            'aborted':'CLEAR_COSTMAP'})
        
        smach.StateMachine.add('CLEAR_COSTMAP', create_clear_costmap_state(),
                               transitions={'succeeded':'succeeded',
                                            'aborted':'failed',
                                            'preempted': 'failed'})
    return laser_shutter_state

def create_unshutter_laser_state():
    return create_laser_state(False)
