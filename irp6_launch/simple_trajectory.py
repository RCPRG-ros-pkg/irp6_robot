#! /usr/bin/env python

import roslib; roslib.load_manifest('irp6_launch')
import rospy
import actionlib

from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *

if __name__ == '__main__':
  rospy.init_node('simple_trajectory')
  client = actionlib.SimpleActionClient('irp6p_controller/joint_trajectory_action', JointTrajectoryAction)
  client.wait_for_server()

  goal = JointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  goal.trajectory.points.append(JointTrajectoryPoint([0.0, -1.57, 0.0, 0.0, 4.71, -2.9], [0, 0, 0, 0, 0, 0], [], rospy.Duration(10.0)))

  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

  client.send_goal(goal)

  client.wait_for_result()

