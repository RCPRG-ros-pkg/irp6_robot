#! /usr/bin/env python

import roslib; roslib.load_manifest('sarkofag_servo')
import rospy
import actionlib

from trajectory_msgs.msg import *
from control_msgs.msg import *

if __name__ == '__main__':
  rospy.init_node('simple_trajectory')
  client = actionlib.SimpleActionClient('sarkofag_controller/joint_trajectory_action', FollowJointTrajectoryAction)
  client.wait_for_server()

  print 'server ok'

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1']
  goal.trajectory.points.append(JointTrajectoryPoint([-60.0], [0.0], [], [], rospy.Duration(4.0)))

  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

  client.send_goal(goal)

  client.wait_for_result()

