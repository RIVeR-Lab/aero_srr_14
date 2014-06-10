#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
from arm_state_util import *
import actionlib
from jaco_msgs.msg import *
from jaco_msgs.srv import *
import tf
import sys


def main(argv):
    if len(argv) != 6:
        raise Exception('expected exactly six arguments')

    rospy.init_node('arm_pose_util', anonymous=True)
    client = actionlib.SimpleActionClient('/aero/jaco/arm_trajectory', TrajectoryAction)

    x = float(argv[0])
    y = float(argv[1])
    z = float(argv[2])
    roll = float(argv[3])
    pitch = float(argv[4])
    yaw = float(argv[5])

    trajectory = [create_arm_api_pose(x, y, z, roll, pitch, yaw)]

    goal = TrajectoryGoal(trajectory)

    print 'waiting for server'
    client.wait_for_server()
    print 'commanding arm'
    client.send_goal(goal)
    print 'waiting for result'
    client.wait_for_result()
    print 'done'
        
if __name__ == '__main__':
    main(sys.argv[1:])
