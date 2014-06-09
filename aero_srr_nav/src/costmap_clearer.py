#!/usr/bin/env python
import roslib; roslib.load_manifest('aero_srr_nav')
import rospy
from std_srvs.srv import *

def main():
  rospy.init_node('costmap_clearer')
  clear_delay = rospy.get_param('clear_delay', 10.0)
  rospy.loginfo("waiting for costmap service")
  rospy.wait_for_service('/aero/nav_controller/clear_costmaps')
  while not rospy.is_shutdown():
    try:
      clear_costmap = rospy.ServiceProxy('/aero/nav_controller/clear_costmaps', Empty)
      rospy.loginfo("Created costmap service proxy")
      while not rospy.is_shutdown():
        try:
          rospy.sleep(clear_delay)
          rospy.loginfo("clearing costmap")
          clear_costmap()
        except rospy.ServiceException, e:
          rospy.logerr("could not call service: %s", str(e))
    except rospy.ServiceException, e:
      rospy.logerr("could not create service proxy: %s", str(e))

if __name__ == "__main__":
  main()
