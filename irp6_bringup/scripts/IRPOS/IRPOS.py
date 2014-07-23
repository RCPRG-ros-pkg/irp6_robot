#!/usr/bin/env python

import rospy
import tf
import actionlib

from numpy import *
from numpy.linalg import *

from controller_manager_msgs.srv import *
from std_msgs.msg import *
from diagnostic_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from force_control_msgs.msg import *
from tf.transformations import *

from sensor_msgs.msg import *

import threading
import PyKDL
import tf_conversions.posemath as pm



class IRPOS:
	robot_name = None
	conmanSwitch = None
	
	motor_client = None
	joint_client = None
	tool_client = None
	pose_client = None
	irp6p_tfg_motor_client = None
	irp6p_tfg_joint_client = None

	motor_position_subscriber = None
	joint_position_subscriber = None
	cartesian_position_subscriber = None
	wrench_subscriber = None

	last_motor_position = None
	last_joint_position = None	
	last_cartesian_position = None	
	last_wrench = None

	lmp_lock = threading.Lock()
	ljp_lock = threading.Lock()
	lcp_lock = threading.Lock()
	lw_lock = threading.Lock()

	fcl_param_publisher = None
	tg_param_publisher = None

	tool_weigth = None
	tool_mass_center = None

	def __init__(self, nodeName, robotName):
		self.robot_name = robotName	
		rospy.init_node(nodeName)
		rospy.wait_for_service('/controller_manager/switch_controller')

		self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

		self.motor_position_subscriber = rospy.Subscriber('/motor_states', JointState, self.motor_position_callback)
		self.joint_position_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_position_callback)
		self.cartesian_position_subscriber = rospy.Subscriber('/cartesian_position', Pose, self.cartesian_position_callback)
		self.wrench_subscriber = rospy.Subscriber('/ati_wrench', Wrench, self.wrench_callback)

		self.fcl_param_publisher = rospy.Publisher('/irp6p_arm/fcl_param', ForceControl)
		self.tg_param_publisher = rospy.Publisher('/irp6p_arm/tg_param', ToolGravityParam)
		
		self.motor_client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_motor', FollowJointTrajectoryAction)
		self.motor_client.wait_for_server()

		self.joint_client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
		self.joint_client.wait_for_server()

		self.irp6p_tfg_motor_client = actionlib.SimpleActionClient('/irp6p_tfg/spline_trajectory_action_motor', FollowJointTrajectoryAction)
		self.irp6p_tfg_motor_client.wait_for_server()

		self.irp6p_tfg_joint_client = actionlib.SimpleActionClient('/irp6p_tfg/spline_trajectory_action_joint', FollowJointTrajectoryAction)
		self.irp6p_tfg_joint_client.wait_for_server()

		self.tool_client = actionlib.SimpleActionClient('/irp6p_arm/tool_trajectory', CartesianTrajectoryAction)
		self.tool_client.wait_for_server()

		self.pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
		self.pose_client.wait_for_server()

		print "[IRPOS] System ready"

	def motor_position_callback(self, data):
		self.lmp_lock.acquire()
		self.last_motor_position = data.position
		self.lmp_lock.release()

	def joint_position_callback(self, data):
		self.ljp_lock.acquire()
		self.last_joint_position = data.position
		self.ljp_lock.release()

	def cartesian_position_callback(self, data):
		self.lcp_lock.acquire()
		self.last_cartesian_position = data
		self.lcp_lock.release()

	def wrench_callback(self, data):
		self.lw_lock.acquire()
		self.last_wrench = data
		self.lw_lock.release()

        def move_to_synchro_position(self, duration):
		print "[IRPOS] Move to synchro position"

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorMotor'], [], True)

		motorGoal = FollowJointTrajectoryGoal()
		motorGoal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
		motorGoal.trajectory.points.append(JointTrajectoryPoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(duration)))
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.motor_client.send_goal(motorGoal)
		self.motor_client.wait_for_result()

		result = self.motor_client.get_result()
		print "[IRPOS] Result: "+str(result)

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorMotor'], True)

	# MOTOR      
	def move_to_motor_position(self, motor_positions, time_from_start):
		print "[IRPOS] Move to motor position"

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorMotor'], [], True)

		motorGoal = FollowJointTrajectoryGoal()
		motorGoal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
		motorGoal.trajectory.points.append(JointTrajectoryPoint(motor_positions, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(time_from_start)))
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.motor_client.send_goal(motorGoal)
		self.motor_client.wait_for_result()

		result = self.motor_client.get_result()
		print "[IRPOS] Result: "+str(result)

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorMotor'], True)

	def move_along_motor_trajectory(self, points):
		print "[IRPOS] Move along motor trajectory"

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorMotor'], [], True)

		motorGoal = FollowJointTrajectoryGoal()
		motorGoal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
		for i in points:
			motorGoal.trajectory.points.append(JointTrajectoryPoint(i.positions, i.velocities, i.accelerations, i.effort, i.time_from_start))
			print str(i.positions)
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.motor_client.send_goal(motorGoal)
		self.motor_client.wait_for_result()

		result = self.motor_client.get_result()
		print "[IRPOS] Result: "+str(result)

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorMotor'], True)

	def get_motor_position(self):
		self.lmp_lock.acquire()
		ret = self.last_motor_position
		self.lmp_lock.release()		
		return ret

	# JOINT
	def move_to_joint_position(self, joint_positions, time_from_start):
		print "[IRPOS] Move to joint position"

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorJoint'], [], True)
		
		jointGoal = FollowJointTrajectoryGoal()
		jointGoal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
		jointGoal.trajectory.points.append(JointTrajectoryPoint(joint_positions, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(time_from_start)))
		jointGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.joint_client.send_goal(jointGoal)
		self.joint_client.wait_for_result()
		
		result = self.joint_client.get_result()
		print "[IRPOS] Result: "+str(result)

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorJoint'], True)

	# tested
	def move_along_joint_trajectory(self, points):
		print "[IRPOS] Move along joint trajectory"

		print str(self.get_cartesian_pose())

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorJoint'], [], True)
		
		jointGoal = FollowJointTrajectoryGoal()
		jointGoal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
		for i in points:
			jointGoal.trajectory.points.append(JointTrajectoryPoint(i.positions, i.velocities, i.accelerations, i.effort, i.time_from_start))
			print str(i.positions)
		jointGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.joint_client.send_goal(jointGoal)
		self.joint_client.wait_for_result()
		
		result = self.joint_client.get_result()
		print "[IRPOS] Result: "+str(result)

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorJoint'], True)


	def get_joint_position(self):
		self.ljp_lock.acquire()
		ret = self.last_joint_position
		self.ljp_lock.release()		
		return ret

	# CARTESIAN
	def move_to_cartesian_pose(self, time_from_start, pose):
		print "[IRPOS] Move to cartesian trajctory"
				
		self.conmanSwitch([self.robot_name+'mPoseInt'], [], True)

		cartesianGoal = CartesianTrajectoryGoal()
		cartesianGoal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(time_from_start), pose, Twist()))
		cartesianGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

		self.pose_client.send_goal(cartesianGoal)
		self.pose_client.wait_for_result()

		result = self.pose_client.get_result()
		print "[IRPOS] Result: "+str(result)

		self.conmanSwitch([], [self.robot_name+'mPoseInt'], True)

	def move_along_cartesian_trajectory(self, points):
		print "[IRPOS] Move along cartesian trajectory"

		self.conmanSwitch([self.robot_name+'mPoseInt'], [], True)

		cartesianGoal = CartesianTrajectoryGoal()
		for i in points:
			cartesianGoal.trajectory.points.append(CartesianTrajectoryPoint(i.time_from_start, i.pose, i.twist))
			print str(i.pose.position.x)+str(i.pose.position.y)+str(i.pose.position.z)
		cartesianGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

		self.pose_client.send_goal(cartesianGoal)
		self.pose_client.wait_for_result()

		result = self.pose_client.get_result()
		print "[IRPOS] Result: "+str(result)

		self.conmanSwitch([], [self.robot_name+'mPoseInt'], True)

	def move_along_cartesian_circle(self, P1, P2, P3, time_from_start):
		print "[IRPOS] Move along cartesian circle"

		self.conmanSwitch([self.robot_name+'mPoseInt'], [], True)

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
  
		trj = []
  
		number_of_points = time_from_start/0.01
  
		i = 1
  
		while (i<=number_of_points):
			pz = pc + (p1-pc)*cos((i/number_of_points)*alpha) + cross(n, p1-pc) * sin((i/number_of_points)*alpha)
			trj.append(CartesianTrajectoryPoint(rospy.Duration(time_from_start*i/number_of_points), Pose(Point(pz[0], pz[1], pz[2]), P1.orientation), Twist()))
			i = i + 1

		goal = CartesianTrajectoryGoal()
		goal.trajectory.points = trj
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.pose_client.send_goal(goal)
		self.pose_client.wait_for_result()

		result = self.pose_client.get_result()
		print "[IRPOS] Result: "+str(result)

		self.conmanSwitch([], [self.robot_name+'mPoseInt'], True)
  
	def get_cartesian_pose(self):
		self.lcp_lock.acquire()
		ret = self.last_cartesian_position
		self.lcp_lock.release()		
		return ret

	# TOOL	
	def set_tool_geometry_params(self, transformation):
		print "[IRPOS] Set tool geometry params"

		goal = CartesianTrajectoryGoal() 

		goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(0.0), transformation, Twist()))
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

		self.tool_client.send_goal(goal)
		self.tool_client.wait_for_result()
		result = self.tool_client.get_result()
		print "[IRPOS] Result: "+str(result)
	
	def set_tool_physical_params(self, weight, mass_center_position):
		print "[IRPOS] Set tool physical params"

		self.tool_weight = weight
		self.tool_mass_center = mass_center_position

	# FORCE CONTROL
	def start_force_controller(self, inertia, reciprocaldamping, wrench, twist):
		print "[IRPOS] Start force controller"

		forceControlGoal = ForceControl()
		forceControlGoal.inertia = inertia
		forceControlGoal.reciprocaldamping = reciprocaldamping
		forceControlGoal.wrench = wrench
		forceControlGoal.twist = twist
  
		self.fcl_param_publisher.publish(forceControlGoal)
 
		tg_goal = ToolGravityParam()
		tg_goal.weight = self.tool_weight
		tg_goal.mass_center = self.tool_mass_center
 
		self.tg_param_publisher.publish(tg_goal)

		self.conmanSwitch([self.robot_name+'mForceTransformation',self.robot_name+'mForceControlLaw'], [], True)

	def stop_force_controller(self):
		print "[IRPOS] Start force controller"

		self.conmanSwitch([], [self.robot_name+'mForceTransformation',self.robot_name+'mForceControlLaw'], True)
		return 0

	def get_force_readings(self):
		self.lw_lock.acquire()
		ret = self.last_wrench
		self.lw_lock.release()		
		return ret

	# GRIPPER
	def tfg_to_motor_position(self, motor_position, time_from_start):
		print "[IRPOS] Tfg to motor position"		
		self.conmanSwitch([self.robot_name+'tfgSplineTrajectoryGeneratorMotor'], [], True)
  
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = ['joint1']
		goal.trajectory.points.append(JointTrajectoryPoint([motor_position], [0.0], [], [], rospy.Duration(time_from_start)))
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.irp6p_tfg_motor_client.send_goal(goal)
		self.irp6p_tfg_motor_client.wait_for_result()

		command_result = self.irp6p_tfg_motor_client.get_result()
    
		self.conmanSwitch([], [self.robot_name+'tfgSplineTrajectoryGeneratorMotor'], True)  

	def tfg_to_joint_position(self, joint_position, time_from_start):
		print "[IRPOS] Tfg to joint position"	
		self.conmanSwitch([self.robot_name+'tfgSplineTrajectoryGeneratorJoint'], [], True)
  
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = ['joint1']
		goal.trajectory.points.append(JointTrajectoryPoint([joint_position], [0.0], [], [], rospy.Duration(time_from_start)))
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.irp6p_tfg_joint_client.send_goal(goal)
		self.irp6p_tfg_joint_client.wait_for_result()

		command_result = self.irp6p_tfg_joint_client.get_result()
     
		self.conmanSwitch([], [self.robot_name+'tfgSplineTrajectoryGeneratorJoint'], True)

