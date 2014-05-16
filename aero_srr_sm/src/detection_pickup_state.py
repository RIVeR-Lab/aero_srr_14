#!/usr/bin/env python

import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
from arm_state_util import *
import actionlib
from jaco_msgs.msg import *
from jaco_msgs.srv import *
import tf

class DetectionPickupState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['detection_msg'])
        self.client = actionlib.SimpleActionClient('/aero/jaco/arm_trajectory', TrajectoryAction)
        self.tf_listener = tf.TransformListener()
        self.home_client = rospy.ServiceProxy('/aero/jaco/home_arm', HomeArm)
            

    def execute(self, userdata):
        rospy.loginfo('Executing pickup state')
        print('Got detection ', str(userdata['detection_msg']) )

        self.client.wait_for_server()
        rospy.wait_for_service('/aero/jaco/home_arm')

        object_location = PoseStamped()
        object_location.pose = userdata['detection_msg'].pose.pose;
        object_location.header = userdata['detection_msg'].header;
        print('Got location ', str(object_location) )

        try:
            self.tf_listener.waitForTransform('jaco_api_origin', object_location.header.frame_id, object_location.header.stamp, rospy.Duration(1))
        
            object_location_api = self.tf_listener.transformPose('jaco_api_origin', object_location)
            object_position = object_location_api.pose.position

            trajectory = [create_arm_api_pose(0.5, -0.2, 0.4, -math.pi/2, math.pi/2, 0),
                            create_arm_api_pose(object_position.x, object_position.y-0.1, 0.4, -math.pi/2, math.pi/4, 0),
                            create_arm_fingers(create_arm_api_pose(object_position.x, object_position.y-0.1, object_position.z+0.1, -math.pi/2, 0, 0), 1, 1, 1),
                            create_arm_api_pose(object_position.x, object_position.y+0.04, object_position.z+0.1, -math.pi/2, 0, 0),
                            create_arm_fingers(create_arm_api_pose(object_position.x, object_position.y+0.04, object_position.z+0.1, -math.pi/2, 0, 0), 60, 60, 60),
                            create_arm_api_pose(object_position.x, object_position.y+0.04, object_position.z+0.2, -math.pi/2, 0, 0),
                            create_arm_api_pose(0.4, 0.2, 0.4, -math.pi/2, 0, 0),
                            create_arm_api_pose(-0.2, 0.5, 0.4, -math.pi/2, -math.pi/2, 0),
                            create_arm_api_pose(-0.3, 0.1, 0.4, -math.pi/2, -math.pi*3/5, 0),
                            create_arm_fingers(create_arm_api_pose(-0.3, 0.1, 0.4, -math.pi/2, -math.pi*3/5, 0), 1, 1, 1),
                            create_arm_api_pose(-0.3, 0.1, 0.6, -math.pi/2, -math.pi*3/5, 0),
                            create_arm_api_pose(-0.3, -0.3, 0.6, -math.pi/2, -math.pi/2, 0)]
            goal=TrajectoryGoal(trajectory);
            self.client.send_goal(goal)

            self.client.wait_for_result()

            rospy.loginfo('Homing Arm')
            self.home_client()

            if self.client.get_state() == GoalStatus.SUCCEEDED:
                return 'succeeded';
            if self.client.get_state() == GoalStatus.PREEMPTED:
                return 'preempted'
        except tf.Exception:
            return 'aborted'
        return 'aborted'
        
