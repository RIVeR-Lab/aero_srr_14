import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
import smach_ros
from pprint import pprint
from actionlib import *
from actionlib.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from srr_search.msg import *
import math
import tf2_ros
import tf


def create_move_goal(frame_id, stamp, x, y, angle):
    goal_angle = tf.transformations.quaternion_from_euler(0, 0, angle);
    goal=MoveBaseGoal(
        target_pose=PoseStamped(
            header = Header(frame_id=frame_id, stamp=stamp),
            pose   = Pose(
                position = Point(x = x, y = y, z = 0),
                orientation = Quaternion(x = goal_angle[0],
                                         y = goal_angle[1],
                                         z = goal_angle[2],
                                         w = goal_angle[3]),
                )
        ));
    return goal
    
def create_move_state_in_frame(frame_id, x, y, angle):
    goal = create_move_goal(frame_id, rospy.get_rostime(), x, y, angle)
    return smach_ros.SimpleActionState('/aero/move_base', MoveBaseAction, goal)

def create_move_state(x, y, angle):
    return create_move_state_in_frame("aero/odom", x, y, angle)


def create_spiral_search_state_in_frame(frame_id, x, y, angle, search_radius, start_radius, circle_separation, angle_increment):
    center_angle = tf.transformations.quaternion_from_euler(0, 0, angle);
    goal=SpiralSearchGoal(
        center=PoseStamped(
            header = Header(frame_id=frame_id),
            pose   = Pose(
                position = Point(x = x, y = y, z = 0),
                orientation = Quaternion(x = center_angle[0],
                                         y = center_angle[1],
                                         z = center_angle[2],
                                         w = center_angle[3]),
                )
            ),
        search_radius = search_radius,
        start_radius = start_radius,
        circle_separation = circle_separation,
        angle_increment = angle_increment
        );

    return smach_ros.SimpleActionState('/aero/spiral_search', SpiralSearchAction, goal)

def create_spiral_search_state(x, y, angle, search_radius, start_radius, circle_separation, angle_increment):
    return create_spiral_search_state_in_frame("aero/odom", x, y, angle, search_radius, start_radius, circle_separation, angle_increment)


class RawDriveState(smach.State):
    def __init__(self, x_vel, ang_vel, time):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.msg = Twist(linear=Vector3(x=x_vel), angular=Vector3(z=ang_vel))
        self.pub = rospy.Publisher('aero/cmd_vel', Twist)
        self.delay = time

    def execute(self, userdata):
        self.pub.publish(self.msg)
        rospy.sleep( self.delay )
        self.pub.publish(Twist())
        return 'succeeded'
        

def create_drive_backward_state(vel, time):
    return RawDriveState(-vel, 0, time)
