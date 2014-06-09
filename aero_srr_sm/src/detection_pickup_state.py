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
    def __init__(self, timeout):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['detection_msg'])
        self.client = actionlib.SimpleActionClient('/aero/jaco/arm_trajectory', TrajectoryAction)
        self.tf_listener = tf.TransformListener()
        self.home_client = rospy.ServiceProxy('/aero/jaco/home_arm', HomeArm)
        self.timeout = timeout
            

    def execute(self, userdata):
        rospy.loginfo('Executing pickup state')
        #print('Got detection ', str(userdata['detection_msg']) )

        self.client.wait_for_server()
        rospy.wait_for_service('/aero/jaco/home_arm')

        object_location = userdata['detection_msg']

        try:
            self.tf_listener.waitForTransform('jaco_api_origin', object_location.header.frame_id, object_location.header.stamp, rospy.Duration(1))
        
            object_location_api = self.tf_listener.transformPose('jaco_api_origin', object_location)
            object_position = object_location_api.pose.position
            print('Got location ', str(object_position) )


            if object_position.x > 0.55:
              grasp_angle = math.pi/4
            else:
              grasp_angle = 0
            aproach_dist = 0.1
            grasp_through_dist = 0.04
            vertical_offset = 0.085

            rospy.loginfo('Homing Arm')
            self.home_client()

            trajectory = [create_arm_api_pose(0.5, -0.2, 0.4, -math.pi/2, math.pi/2, 0),
                            create_arm_api_pose(object_position.x, object_position.y-0.1, 0.4, -math.pi/2, math.pi/4, 0),
                            create_arm_fingers(create_arm_api_pose(object_position.x-aproach_dist*math.sin(grasp_angle), object_position.y-aproach_dist*math.cos(grasp_angle), object_position.z+vertical_offset, -math.pi/2, grasp_angle, 0), 1, 1, 1),
                            create_arm_api_pose(object_position.x+grasp_through_dist*math.sin(grasp_angle), object_position.y+grasp_through_dist*math.cos(grasp_angle), object_position.z+vertical_offset, -math.pi/2, grasp_angle, 0),
                            create_arm_fingers(create_arm_api_pose(object_position.x+grasp_through_dist*math.sin(grasp_angle), object_position.y+grasp_through_dist*math.cos(grasp_angle), object_position.z+vertical_offset, -math.pi/2, grasp_angle, 0), 60, 60, 60),
                            create_arm_api_pose(object_position.x+grasp_through_dist*math.sin(grasp_angle), object_position.y+grasp_through_dist*math.cos(grasp_angle), object_position.z+0.2, -math.pi/2, grasp_angle, 0),
                            create_arm_api_pose(0.4, 0.2, 0.5, -math.pi/2, 0, 0),
                            create_arm_api_pose(-0.2, 0.5, 0.5, -math.pi/2, -math.pi/2, 0),
                            create_arm_api_pose(-0.35, 0.0, 0.55, -math.pi/2, -math.pi*3/5, 0),
            #release sample
                            create_arm_fingers(create_arm_api_pose(-0.35, 0.0, 0.55, -math.pi/2, -math.pi*3/5, 0), 1, 1, 1),
                            create_arm_api_pose(-0.37, 0.02, 0.7, -math.pi/2, -math.pi*3/5, 0),
                            create_arm_api_pose(-0.37, 0.02, 0.55, math.pi*3/5, -math.pi/3, 0),
                            create_arm_fingers(create_arm_api_pose(-0.37, 0.02, 0.55, math.pi*3/5, -math.pi/3, 0), 60, 60, 60)]
            goal=TrajectoryGoal(trajectory);
            self.client.send_goal(goal)

            if not self.client.wait_for_result(self.timeout):
                self.client.cancel_all_goals()

            rospy.loginfo('Homing Arm')
            self.home_client()

            if self.client.get_state() == GoalStatus.SUCCEEDED:
                return 'succeeded';
            if self.client.get_state() == GoalStatus.PREEMPTED:
                return 'preempted'
        except tf.Exception:
            return 'aborted'
        return 'aborted'
        
