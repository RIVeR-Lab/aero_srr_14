#! /usr/bin/env python

import roslib; roslib.load_manifest('srr_carrot_planner')
import rospy
import actionlib
import tf
import math
import copy

from srr_search.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *

class MoveBaseCarrotPlannerServer:
    def __init__(self):
        self.dist_inc = rospy.get_param('~max_carrot_distance', 0.3)
        
        self.move_base_client = actionlib.SimpleActionClient('move_base_raw', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.carrot_pub = rospy.Publisher('move_base_carrots', PoseArray)

        self.tf_listener = tf.TransformListener()
        
        self.server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.execute, False)
        self.server.register_preempt_callback(self.preempt_cb);

        self.server.start()
        

    def execute(self, goal):
        try:
            array = PoseArray()
            array.header.frame_id = 'aero/odom'

            self.tf_listener.waitForTransform('aero/odom', goal.target_pose.header.frame_id, goal.target_pose.header.stamp, rospy.Duration(1))
            target_pose = self.tf_listener.transformPose('aero/odom', goal.target_pose)
            starting_pose = self.tf_listener.transformPose('aero/odom', PoseStamped(header = Header(frame_id = 'aero/base_footprint')))
            
            dx = target_pose.pose.position.x - starting_pose.pose.position.x
            dy = target_pose.pose.position.y - starting_pose.pose.position.y
            total_dist = math.sqrt(dx**2 + dy**2)
            drive_angle = tf.transformations.quaternion_from_euler(0, 0, math.atan2(dy, dx))
            drive_quaternion = Quaternion(x = drive_angle[0],
                                          y = drive_angle[1],
                                          z = drive_angle[2],
                                          w = drive_angle[3])
            
            current_dist = 0
            
            while not self.server.is_preempt_requested():
                next_goal = MoveBaseGoal()
                next_goal.target_pose = copy.deepcopy(target_pose)
                
                current_dist = current_dist + self.dist_inc
                if current_dist > total_dist:
                    current_dist = total_dist
                else:
                    next_goal.target_pose.pose.orientation = drive_quaternion
                    
                    next_goal.target_pose.pose.position.x = starting_pose.pose.position.x + current_dist*dx/total_dist
                    next_goal.target_pose.pose.position.y = starting_pose.pose.position.y + current_dist*dy/total_dist
                    rospy.logdebug('executing segment: %f  (%f, %f)', current_dist, next_goal.target_pose.pose.position.x, next_goal.target_pose.pose.position.y)
                    
                    array.poses.append(copy.deepcopy(next_goal.target_pose.pose))
                    array.header.stamp = rospy.get_rostime()
                    self.carrot_pub.publish(array)
                    
                    result = self.move_base_client.send_goal_and_wait(next_goal)
                    if current_dist == total_dist:
                        self.server.set_succeeded()
                        return
                    
            self.server.set_preempted()
        except tf.Exception as e:
            rospy.logwarn('error executing path: %s', str(e))
            self.move_base_client.cancel_goal()
            self.server.set_aborted()
            
        
    def preempt_cb(self):
        self.move_base_client.cancel_goal()

            
            
if __name__ == '__main__':
    rospy.init_node('move_base_carrot_planner_server')
    server = MoveBaseCarrotPlannerServer()
    rospy.spin()
