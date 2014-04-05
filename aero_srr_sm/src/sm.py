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
from move_base_msgs.msg import *
import math
import tf2_ros
import tf

# define state Foo
class FakeState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing fake state ')
        outcome = ''
        while outcome not in self.get_registered_outcomes():
            print 'enter a possible outcome: '+str(self.get_registered_outcomes())
            outcome = raw_input('Enter state outcome: ');
            if outcome == 's':
                outcome = 'succeeded';
            if outcome == 'a':
                outcome = 'aborted';
            if outcome == 'p':
                outcome = 'preempted';
        return outcome
        



# main
def main():
    rospy.init_node('aero_srr_sm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', FakeState(),
                               transitions={'succeeded':'LEAVE_PLATFORM',
                                            'aborted':'failed',
                                            'preempted':'failed'})

        leave_platform_goal_angle = tf.transformations.quaternion_from_euler(0, 0, math.pi/2);
        leave_platform_goal=MoveBaseGoal(
            target_pose=PoseStamped(
                header = Header(frame_id="aero/odom"),
                pose   = Pose(
                    position = Point(x = 3, y = 20, z = 0),
                    orientation = Quaternion(x = leave_platform_goal_angle[0],
                                             y = leave_platform_goal_angle[1],
                                             z = leave_platform_goal_angle[2],
                                             w = leave_platform_goal_angle[3]),
                    )
            ));
        towards_precache_goal_angle = tf.transformations.quaternion_from_euler(0, 0, math.pi);
        towards_precache_goal=MoveBaseGoal(
            target_pose=PoseStamped(
                header = Header(frame_id="aero/odom"),
                pose   = Pose(
                    position = Point(x = -20, y = 10, z = 0),
                    orientation = Quaternion(x = towards_precache_goal_angle[0],
                                             y = towards_precache_goal_angle[1],
                                             z = towards_precache_goal_angle[2],
                                             w = towards_precache_goal_angle[3]),
                    )
            ));

        smach.StateMachine.add('LEAVE_PLATFORM',
                               smach_ros.SimpleActionState('/aero/move_base', MoveBaseAction, leave_platform_goal),
                               transitions={'succeeded':'MOVE_TOWARDS_PRECACHE',
                                            'aborted':'failed',
                                            'preempted':'failed'})
        smach.StateMachine.add('MOVE_TOWARDS_PRECACHE',
                               smach_ros.SimpleActionState('/aero/move_base', MoveBaseAction, towards_precache_goal),
                               transitions={'succeeded':'SEARCH_FOR_PRECACHE',
                                            'aborted':'failed',
                                            'preempted':'failed'})
        smach.StateMachine.add('SEARCH_FOR_PRECACHE', FakeState(),
                               transitions={'succeeded':'NAV_TO_PRECACHE',
                                            'aborted':'SEARCH_FOR_PRECACHE',
                                            'preempted':'failed'})
        smach.StateMachine.add('NAV_TO_PRECACHE', FakeState(),
                               transitions={'succeeded':'PICKUP_PRECACHE',
                                            'aborted':'SEARCH_FOR_PRECACHE',
                                            'preempted':'failed'})

        pickup_sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with pickup_sm:
            smach.StateMachine.add('NAV_TO_PICKUP_POSITION', FakeState(),
                                   transitions={'succeeded':'POSITION_ARM_FOR_GRAB',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('POSITION_ARM_FOR_GRAB', FakeState(),
                                   transitions={'succeeded':'GRAB',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('GRAB', FakeState(),
                                   transitions={'succeeded':'PICKUP_SAMPLE',
                                                'aborted':'POSITION_ARM_FOR_GRAB',
                                                'preempted':'preempted'})
            smach.StateMachine.add('PICKUP_SAMPLE', FakeState(),
                                   transitions={'succeeded':'STORE_SAMPLE',
                                                'aborted':'PICKUP_SAMPLE',
                                                'preempted':'preempted'})
            smach.StateMachine.add('STORE_SAMPLE', FakeState(),
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'STORE_SAMPLE',
                                                'preempted':'preempted'})
        smach.StateMachine.add('PICKUP_PRECACHE', pickup_sm,
                               transitions={'succeeded':'NAV_TO_PLATFORM',
                                            'aborted':'SEARCH_FOR_PRECACHE',
                                            'preempted':'failed'})

        smach.StateMachine.add('NAV_TO_PLATFORM', FakeState(),
                               transitions={'succeeded':'NAV_ONTO_PLATFORM',
                                            'aborted':'NAV_TO_PLATFORM',
                                            'preempted':'failed'})
        smach.StateMachine.add('NAV_ONTO_PLATFORM', FakeState(),
                               transitions={'succeeded':'succeeded',
                                            'aborted':'NAV_TO_PLATFORM',
                                            'preempted':'failed'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()