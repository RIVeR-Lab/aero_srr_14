import roslib; roslib.load_manifest('aero_srr_sm')
import rospy
import smach
import smach_ros
from pprint import pprint
from actionlib import *
from actionlib.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from jaco_msgs.msg import *
from jaco_msgs.srv import *
import math
import tf2_ros
import tf


def create_arm_pose_from_pose_stamped(pose):
    return TrajectoryPoint(
        position_type = TrajectoryPoint.POSITION_TYPE_CARTESIAN_POSITION,
        position = pose,
        hand_mode = TrajectoryPoint.HAND_MODE_NO_MOVE
        )
def create_arm_api_pose(x, y, z, roll, pitch, yaw):
    return create_arm_pose('jaco_api_origin', rospy.get_rostime(), x, y, z, roll, pitch, yaw)
def create_arm_pose(frame, stamp, x, y, z, roll, pitch, yaw):
    angle = tf.transformations.quaternion_from_euler(roll, pitch, yaw);
    pose = PoseStamped(
        header = Header(frame_id=frame, stamp=stamp),
        pose   = Pose(
            position = Point(x = x, y = y, z = z),
            orientation = Quaternion(x = angle[0],
                                     y = angle[1],
                                     z = angle[2],
                                     w = angle[3]),
            )
    );
    return create_arm_pose_from_pose_stamped(pose)

def create_arm_delay(delay):
    return TrajectoryPoint(
        position_type = TrajectoryPoint.POSITION_TYPE_TIME_DELAY,
        delay = delay,
        hand_mode = TrajectoryPoint.HAND_MODE_NO_MOVE
        )

def create_arm_fingers(base, f1, f2, f3):
    base.fingers = FingerPosition(Finger_1 = f1, Finger_2 = f2, Finger_3 = f3)
    base.hand_mode = TrajectoryPoint.HAND_MODE_POSITION
    return base

def create_arm_trajectory_state(trajectory, timeout=rospy.Duration.from_sec(10.0)):
    goal=TrajectoryGoal(trajectory);
    return smach_ros.SimpleActionState('/aero/jaco/arm_trajectory', TrajectoryAction, goal, exec_timeout=timeout)


class ArmStowState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.client = actionlib.SimpleActionClient('/aero/jaco/arm_joint_angles', ArmJointAnglesAction)
        self.home_client = rospy.ServiceProxy('/aero/jaco/home_arm', HomeArm)

    def execute(self, userdata):
        rospy.loginfo('Stowing arm')

        rospy.wait_for_service('/aero/jaco/home_arm')
        self.home_client()

        self.client.wait_for_server()

        goal = ArmJointAnglesGoal()
        goal.angles.Angle_J1 = 0.05
        goal.angles.Angle_J2 = -1.57079633
        goal.angles.Angle_J3 = -0.296705973
        goal.angles.Angle_J4 = -4.86946861
        goal.angles.Angle_J5 = 1.36135682
        goal.angles.Angle_J6 = 2.68780705
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.client.get_state() == GoalStatus.ABORTED:
            return 'aborted';
        if self.client.get_state() == GoalStatus.PREEMPTED:
            return 'preempted'

        goal = ArmJointAnglesGoal()
        goal.angles.Angle_J1 = 0.05
        goal.angles.Angle_J2 = -2.77507351
        goal.angles.Angle_J3 = -0.296705973
        goal.angles.Angle_J4 = -4.86946861
        goal.angles.Angle_J5 = 1.36135682
        goal.angles.Angle_J6 = 2.68780705
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            return 'succeeded';
        if self.client.get_state() == GoalStatus.PREEMPTED:
            return 'preempted'
        return 'aborted';

