#! /usr/bin/env python

import roslib; roslib.load_manifest('srr_search')
import rospy
import actionlib
import tf
import math
import copy

from srr_search.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *

class SpiralSearchServer:
    def __init__(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.spiral_pub = rospy.Publisher('spiral_poses', PoseArray)
        
        self.server = actionlib.SimpleActionServer('spiral_search', SpiralSearchAction, self.execute, False)
        self.server.register_preempt_callback(self.preempt_cb);

        self.server.start()
        

    def execute(self, goal):
        angle = 0
        array = PoseArray()
        array.header.frame_id = 'aero/odom'
        
        while not self.server.is_preempt_requested():
            next_goal = MoveBaseGoal()
            next_goal.target_pose = copy.deepcopy(goal.center)

            
            quaternion = (
                next_goal.target_pose.pose.orientation.x,
                next_goal.target_pose.pose.orientation.y,
                next_goal.target_pose.pose.orientation.z,
                next_goal.target_pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            radius = goal.start_radius + goal.circle_separation*angle/(math.pi*2)
            if radius > goal.search_radius:
                self.server.set_succeeded()
                return
            
            dx = radius * math.cos(yaw + angle)
            dy = radius * math.sin(yaw + angle)
            dyaw = angle + math.pi/2

            tangent_angle = tf.transformations.quaternion_from_euler(roll, pitch, yaw + dyaw)
            next_goal.target_pose.pose.orientation = Quaternion(x = tangent_angle[0],
                                                                y = tangent_angle[1],
                                                                z = tangent_angle[2],
                                                                w = tangent_angle[3])
            
            next_goal.target_pose.pose.position.x = next_goal.target_pose.pose.position.x + dx
            next_goal.target_pose.pose.position.y = next_goal.target_pose.pose.position.y + dy

            rospy.loginfo("Commanding next spiral step (%f %f) %f  [radius=%f]", dx, dy, dyaw, radius)
            array.poses.append(copy.deepcopy(next_goal.target_pose.pose))
            array.header.stamp = rospy.get_rostime()
            self.spiral_pub.publish(array)
            
            result = self.move_base_client.send_goal_and_wait(next_goal, rospy.Duration.from_sec(10.0), rospy.Duration.from_sec(10.0))

            angle = angle + goal.angle_increment
        self.server.set_preempted()
        
    def preempt_cb(self):
        self.move_base_client.cancel_goal()

            
            
if __name__ == '__main__':
    rospy.init_node('spiral_search_server')
    server = SpiralSearchServer()
    rospy.spin()
