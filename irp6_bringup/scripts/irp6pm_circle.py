#!/usr/bin/env python

import rospy

import actionlib

from numpy import *
from numpy.linalg import *

from controller_manager_msgs.srv import *
from geometry_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from std_msgs.msg import *
from diagnostic_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *

def generateCircle(P1, P2, P3, T):
  
  p1 = array([P1.position.x, P1.position.y, P1.position.z])
  p2 = array([P2.position.x, P2.position.y, P2.position.z])
  p3 = array([P3.position.x, P3.position.y, P3.position.z])
  
  #r = (norm(p1-p2) * norm(p2-p3) * norm(p3-p1))/(2*norm(cross(p1-p2, p2-p3)))
  a = (norm(p2-p3) * norm(p2-p3) * dot(p1-p2, p1-p3))/(2 * norm(cross(p1-p2, p2-p3)) * norm(cross(p1-p2, p2-p3)))
  b = (norm(p1-p3) * norm(p1-p3) * dot(p2-p1, p2-p3))/(2 * norm(cross(p1-p2, p2-p3)) * norm(cross(p1-p2, p2-p3)))
  c = (norm(p1-p2) * norm(p1-p2) * dot(p3-p1, p3-p2))/(2 * norm(cross(p1-p2, p2-p3)) * norm(cross(p1-p2, p2-p3)))
  
  pc = a*p1 + b*p2 + c*p3

  n = cross(p2-p1, p3-p1)/norm(cross(p2-p1, p3-p1))
  
  cosang = dot(p1-pc, p3-pc)
  sinang = norm(cross(p1-pc, p3-pc))
  alpha = arctan2(sinang, cosang)
  
  print alpha
  
  trj = []
  
  number_of_points = T/0.01
  
  i = 1
  
  while (i<=number_of_points):
    pz = pc + (p1-pc)*cos((i/number_of_points)*alpha) + cross(n, p1-pc) * sin((i/number_of_points)*alpha)
    trj.append(CartesianTrajectoryPoint(rospy.Duration(T*i/number_of_points), Pose(Point(pz[0], pz[1], pz[2]), P1.orientation), Twist()))
    i = i + 1
  
  return trj

# MAIN

rospy.init_node('irp6pm_circle')

  rospy.wait_for_service('/irp6p_manager/switch_controller')
  conmanSwitch = rospy.ServiceProxy('/irp6p_manager/switch_controller', SwitchController)


  #
  # Deactivate all generators
  #
  
conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorMotor','Irp6pmSplineTrajectoryGeneratorJoint','Irp6pmPoseInt','Irp6pmForceControlLaw','Irp6pmForceTransformation'], True)



conmanSwitch(['Irp6pmSplineTrajectoryGeneratorJoint'], [], True)
  
joint_client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
joint_client.wait_for_server()

print 'server ok'

goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
goal.trajectory.points.append(JointTrajectoryPoint([0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(5.0)))
goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

joint_client.send_goal(goal)

joint_client.wait_for_result()
command_result = joint_client.get_result()
  


conmanSwitch(['Irp6pmPoseInt'], ['Irp6pmSplineTrajectoryGeneratorJoint'], True)

pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
pose_client.wait_for_server()

print 'server ok'

goal = CartesianTrajectoryGoal()
  
goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.855438961242, -0.1208864692291, 1.281029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()))
goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
pose_client.send_goal(goal)

pose_client.wait_for_result()
command_result = pose_client.get_result()

goal = CartesianTrajectoryGoal()

P1 = Pose(Point(0.855438961242, -0.1208864692291, 1.281029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928))
P2 = Pose(Point(0.755438961242, 0.0, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928))
P3 = Pose(Point(0.855438961242, 0.1208864692291, 1.081029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928))

goal.trajectory.points = generateCircle(P1, P2, P3, 30.0)
goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

pose_client.send_goal(goal)

#print goal.trajectory.points

pose_client.wait_for_result()
command_result = pose_client.get_result()

conmanSwitch([], ['Irp6pmPoseInt'], True)
