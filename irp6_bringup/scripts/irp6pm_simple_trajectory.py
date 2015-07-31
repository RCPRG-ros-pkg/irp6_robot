#! /usr/bin/env python

import rospy
import tf
import actionlib
import time

from controller_manager_msgs.srv import *
from std_msgs.msg import *
from diagnostic_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from cartesian_trajectory_msgs.msg import *

from tf.transformations import *

from PyKDL import Rotation

def done_callback(state, result):
	print "Done callback"
	conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorJoint'], True)

if __name__ == '__main__':
  rospy.init_node('irp6pm_simple_trajectory')
  
  rospy.wait_for_service('/irp6p_manager/switch_controller')
  conmanSwitch = rospy.ServiceProxy('/irp6p_manager/switch_controller', SwitchController)

  
  #
  # Deactivate all generators
  #
  
  # done_callback()
  
  conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorMotor','Irp6pmSplineTrajectoryGeneratorJoint','Irp6pmPoseInt','Irp6pmForceControlLaw','Irp6pmForceTransformation'], True)
  
  
  conmanSwitch(['Irp6pmSplineTrajectoryGeneratorJoint'], [], True)
    
  client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
  client.wait_for_server()

  print 'server ok'


 

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  goal.trajectory.points.append(JointTrajectoryPoint([1.57, -1.5418065817051163, 0.0, 1.57, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  client.send_goal(goal, done_callback)
  
  print 'za send goal'

  # client.wait_for_result()
  # command_result = client.get_result()
  
  # conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorJoint'], True)
  
  
  time.sleep(20)
  print 'finish'
  


