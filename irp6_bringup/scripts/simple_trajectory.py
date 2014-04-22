#! /usr/bin/env python

import roslib;
import rospy
import actionlib

from trajectory_msgs.msg import *
from control_msgs.msg import *

if __name__ == '__main__':
  rospy.init_node('simple_trajectory')
  client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
  client.wait_for_server()

  print 'server ok'

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  goal.trajectory.points.append(JointTrajectoryPoint([0.4, -1.5418065817051163, 0.0, 1.57, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)))
  goal.trajectory.points.append(JointTrajectoryPoint([0.0, -1.5418065817051163, 0.0, 1.57, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(15.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  client.send_goal(goal)

  client.wait_for_result()
  command_result = client.get_result()
  print 'finish'

