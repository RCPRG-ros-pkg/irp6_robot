#!/usr/bin/env python

import rospy
import tf
import actionlib
import sys
import time

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

	BCOLOR = '\033[94m'
	ENDC = '\033[0m'

	robot_name = None
	robot_joint_names = None

	conmanSwitch = None
	
	motor_client = None
	joint_client = None
	tool_client = None
	pose_client = None
	tfg_motor_client = None
	tfg_joint_client = None

	motor_position_subscriber = None
	joint_position_subscriber = None
	tfg_motor_position_subscriber = None
	tfg_joint_position_subscriber = None
	cartesian_position_subscriber = None
	wrench_subscriber = None

	last_motor_position = None
	last_joint_position = None
	last_tfg_motor_position = None
	last_tfg_joint_position = None	
	last_cartesian_position = None	
	last_wrench = None

	lmp_lock = threading.Lock()
	ljp_lock = threading.Lock()
	ltfgmp_lock = threading.Lock()
	ltfgjp_lock = threading.Lock()
	lcp_lock = threading.Lock()
	lw_lock = threading.Lock()

	fcl_param_publisher = None
	tg_param_publisher = None

	tool_weight = None
	tool_mass_center = None

	def __init__(self, nodeName, robotName, robotJointNumbers):
		self.robot_name = robotName
		#self.robot_joint_names = robotJointNames
		self.robot_joint_names = []
		for i in range(1,robotJointNumbers+1):
			self.robot_joint_names.append('joint'+str(i))


		rospy.init_node(nodeName)
		
		robotNameLower = robotName.lower()
		
		rospy.wait_for_service('/controller_manager/switch_controller')

		self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

		self.motor_position_subscriber = rospy.Subscriber('/'+robotNameLower+'_arm/motor_states', JointState, self.motor_position_callback)
		self.joint_position_subscriber = rospy.Subscriber('/'+robotNameLower+'_arm/joint_states', JointState, self.joint_position_callback)


		self.tfg_motor_position_subscriber = rospy.Subscriber('/'+robotNameLower+'_tfg/motor_state', JointState, self.tfg_motor_position_callback)
		self.tfg_joint_position_subscriber = rospy.Subscriber('/'+robotNameLower+'_tfg/joint_state', JointState, self.tfg_joint_position_callback)

		self.cartesian_position_subscriber = rospy.Subscriber('/'+robotNameLower+'_arm/cartesian_position', Pose, self.cartesian_position_callback)
		self.wrench_subscriber = rospy.Subscriber('/'+robotNameLower+'m_wrench', Wrench, self.wrench_callback)		
	
		self.fcl_param_publisher = rospy.Publisher('/'+robotNameLower+'_arm/fcl_param', ForceControl, queue_size=0)
		self.tg_param_publisher = rospy.Publisher('/'+robotNameLower+'_arm/tg_param', ToolGravityParam, queue_size=0)
		
		self.motor_client = actionlib.SimpleActionClient('/'+robotNameLower+'_arm/spline_trajectory_action_motor', FollowJointTrajectoryAction)
		self.motor_client.wait_for_server()

		self.joint_client = actionlib.SimpleActionClient('/'+robotNameLower+'_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
		self.joint_client.wait_for_server()

		self.tfg_motor_client = actionlib.SimpleActionClient('/'+robotNameLower+'_tfg/spline_trajectory_action_motor', FollowJointTrajectoryAction)
		self.tfg_motor_client.wait_for_server()

		self.tfg_joint_client = actionlib.SimpleActionClient('/'+robotNameLower+'_tfg/spline_trajectory_action_joint', FollowJointTrajectoryAction)
		self.tfg_joint_client.wait_for_server()

		self.tool_client = actionlib.SimpleActionClient('/'+robotNameLower+'_arm/tool_trajectory', CartesianTrajectoryAction)
		self.tool_client.wait_for_server()

		self.pose_client = actionlib.SimpleActionClient('/'+robotNameLower+'_arm/pose_trajectory', CartesianTrajectoryAction)
		self.pose_client.wait_for_server()

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorMotor', self.robot_name+'mSplineTrajectoryGeneratorJoint', self.robot_name+'mPoseInt', self.robot_name+'mForceTransformation', self.robot_name+'mForceControlLaw', self.robot_name+'tfgSplineTrajectoryGeneratorJoint', self.robot_name+'tfgSplineTrajectoryGeneratorMotor'], True)

		self.motor_client.cancel_goal()
		self.joint_client.cancel_goal()
		self.tfg_motor_client.cancel_goal()
		self.tfg_joint_client.cancel_goal()
		self.pose_client.cancel_goal()

		print self.BCOLOR+"[IRPOS] System ready"+self.ENDC

	def motor_position_callback(self, data):
		self.lmp_lock.acquire()
		self.last_motor_position = data.position
		self.lmp_lock.release()

	def joint_position_callback(self, data):
		self.ljp_lock.acquire()
		self.last_joint_position = data.position
		self.ljp_lock.release()

	def tfg_motor_position_callback(self, data):
		self.ltfgmp_lock.acquire()
		self.last_tfg_motor_position = data.position
		self.ltfgmp_lock.release()

	def tfg_joint_position_callback(self, data):
		self.ltfgjp_lock.acquire()
		self.last_tfg_joint_position = data.position
		self.ltfgjp_lock.release()

	def cartesian_position_callback(self, data):
		self.lcp_lock.acquire()
		self.last_cartesian_position = data
		self.lcp_lock.release()

	def wrench_callback(self, data):
		self.lw_lock.acquire()
		self.last_wrench = data
		self.lw_lock.release()

	def get_zeros_vector(self):
		return [0.0] * len(self.robot_joint_names)

	def spline_error_code_to_string(self, error_code):
		if (error_code==0): 
			return "SUCCESSFUL"
		elif (error_code==-1): 
			return "INVALID_GOAL"
		elif (error_code==-2): 
			return "INVALID_JOINTS"
		elif (error_code==-3): 
			return "OLD_HEADER_TIMESTAMP"
		elif (error_code==-4): 
			return "PATH_TOLERANCE_VIOLATED"
		elif (error_code==-5): 
			return "GOAL_TOLERANCE_VIOLATED"
		return "UNKNOWN"

	def cartesian_error_code_to_string(self, error_code):
		if (error_code==0): 
			return "SUCCESSFUL"
		elif (error_code==-1): 
			return "ROOT_TRANSFORM_FAILED"
		elif (error_code==-2): 
			return "TOOL_TRANSFORM_FAILED"
		elif (error_code==-3): 
			return "PATH_TOLERANCE_VIOLATED"
		elif (error_code==-4): 
			return "INVALID_POSTURE"
		return "UNKNOWN"

        def move_to_synchro_position(self, duration):
		print self.BCOLOR+"[IRPOS] Move to synchro position"+self.ENDC

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorMotor'], [], True)

		motorGoal = FollowJointTrajectoryGoal()
		motorGoal.trajectory.joint_names = self.robot_joint_names
		motorGoal.trajectory.points.append(JointTrajectoryPoint(self.get_zeros_vector(), self.get_zeros_vector(), [], [], rospy.Duration(duration)))
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.motor_client.send_goal(motorGoal)
		self.motor_client.wait_for_result()

		result = self.motor_client.get_result()
		code = self.spline_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorMotor'], True)

	# MOTOR POSITION   

	def move_to_motor_position(self, motor_positions, time_from_start):
		print self.BCOLOR+"[IRPOS] Move to motor position"+self.ENDC

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorMotor'], [], True)

		motorGoal = FollowJointTrajectoryGoal()
		motorGoal.trajectory.joint_names = self.robot_joint_names
		motorGoal.trajectory.points.append(JointTrajectoryPoint(motor_positions, self.get_zeros_vector(), [], [], rospy.Duration(time_from_start)))
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.motor_client.send_goal(motorGoal)
		self.motor_client.wait_for_result()

		result = self.motor_client.get_result()
		code = self.spline_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorMotor'], True)

	def move_rel_to_motor_position(self, motor_positions, time_from_start):
		print self.BCOLOR+"[IRPOS] Move to motor position"+self.ENDC

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorMotor'], [], True)

		actual_motors = self.get_motor_position()

		motorGoal = FollowJointTrajectoryGoal()
		motorGoal.trajectory.joint_names = self.robot_joint_names
		motorGoal.trajectory.points.append(JointTrajectoryPoint(map(sum, zip(list(actual_motors),motor_positions)), self.get_zeros_vector(), [], [], rospy.Duration(time_from_start)))
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.motor_client.send_goal(motorGoal)
		self.motor_client.wait_for_result()

		result = self.motor_client.get_result()
		code = self.spline_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorMotor'], True)

	def move_along_motor_trajectory(self, points):
		print self.BCOLOR+"[IRPOS] Move along motor trajectory"+self.ENDC

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorMotor'], [], True)

		motorGoal = FollowJointTrajectoryGoal()
		motorGoal.trajectory.joint_names = self.robot_joint_names
		for i in points:
			motorGoal.trajectory.points.append(JointTrajectoryPoint(i.positions, i.velocities, i.accelerations, i.effort, i.time_from_start))
			#print str(i.positions)
		motorGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.motor_client.send_goal(motorGoal)
		self.motor_client.wait_for_result()

		result = self.motor_client.get_result()
		code = self.spline_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorMotor'], True)

	def get_motor_position(self):
		self.lmp_lock.acquire()
		ret = self.last_motor_position
		self.lmp_lock.release()		
		return ret

	# JOINT POSITION

	def move_to_joint_position(self, joint_positions, time_from_start):
		print self.BCOLOR+"[IRPOS] Move to joint position"+self.ENDC

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorJoint'], [], True)
		
		jointGoal = FollowJointTrajectoryGoal()
		jointGoal.trajectory.joint_names = self.robot_joint_names
		jointGoal.trajectory.points.append(JointTrajectoryPoint(joint_positions, self.get_zeros_vector(), [], [], rospy.Duration(time_from_start)))
		jointGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.joint_client.send_goal(jointGoal)
		self.joint_client.wait_for_result()
		
		result = self.joint_client.get_result()
		code = self.spline_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorJoint'], True)

	def move_rel_to_joint_position(self, joint_positions, time_from_start):
		print self.BCOLOR+"[IRPOS] Move relative to joint position"+self.ENDC

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorJoint'], [], True)
		
		actual_joints = self.get_joint_position()

		jointGoal = FollowJointTrajectoryGoal()
		jointGoal.trajectory.joint_names = self.robot_joint_names
		jointGoal.trajectory.points.append(JointTrajectoryPoint(map(sum, zip(list(actual_joints),joint_positions)), self.get_zeros_vector(), [], [], rospy.Duration(time_from_start)))
		jointGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.joint_client.send_goal(jointGoal)
		self.joint_client.wait_for_result()
		
		result = self.joint_client.get_result()
		code = self.spline_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorJoint'], True)

	def move_along_joint_trajectory(self, points):
		print self.BCOLOR+"[IRPOS] Move along joint trajectory"+self.ENDC

		self.conmanSwitch([self.robot_name+'mSplineTrajectoryGeneratorJoint'], [], True)
		
		jointGoal = FollowJointTrajectoryGoal()
		jointGoal.trajectory.joint_names = self.robot_joint_names
		for i in points:
			jointGoal.trajectory.points.append(JointTrajectoryPoint(i.positions, i.velocities, i.accelerations, i.effort, i.time_from_start))
			print str(i.positions)
		jointGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.joint_client.send_goal(jointGoal)
		self.joint_client.wait_for_result()
		
		result = self.joint_client.get_result()
		code = self.spline_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mSplineTrajectoryGeneratorJoint'], True)


	def get_joint_position(self):
		self.ljp_lock.acquire()
		ret = self.last_joint_position
		self.ljp_lock.release()		
		return ret

	# CARTESIAN POSITION

	def move_to_cartesian_pose(self, time_from_start, pose):
		print self.BCOLOR+"[IRPOS] Move to cartesian trajctory"+self.ENDC
				
		self.conmanSwitch([self.robot_name+'mPoseInt'], [], True)

		cartesianGoal = CartesianTrajectoryGoal()
		cartesianGoal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(time_from_start), pose, Twist()))
		cartesianGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

		self.pose_client.send_goal(cartesianGoal)
		self.pose_client.wait_for_result()

		result = self.pose_client.get_result()
		code = self.cartesian_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mPoseInt'], True)

	def move_rel_to_cartesian_pose(self, time_from_start, rel_pose):
		print self.BCOLOR+"[IRPOS] Move relative to cartesian trajctory"+self.ENDC
				
		self.conmanSwitch([self.robot_name+'mPoseInt'], [], True)

		actual_pose = self.get_cartesian_pose()
		pose = Pose(Point(actual_pose.position.x+rel_pose.position.x, actual_pose.position.y+rel_pose.position.y, actual_pose.position.z+rel_pose.position.z), Quaternion(actual_pose.orientation.x+rel_pose.orientation.x, actual_pose.orientation.y+rel_pose.orientation.y, actual_pose.orientation.z+rel_pose.orientation.z, actual_pose.orientation.w+rel_pose.orientation.w))

		cartesianGoal = CartesianTrajectoryGoal()
		cartesianGoal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(time_from_start), pose, Twist()))
		cartesianGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)
		
		self.pose_client.send_goal(cartesianGoal)
		self.pose_client.wait_for_result()

		result = self.pose_client.get_result()
		code = self.cartesian_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mPoseInt'], True)

	def move_rel_to_cartesian_pose_with_contact(self, time_from_start, rel_pose, wrench_constraint):
		print self.BCOLOR+"[IRPOS] Move relative to cartesian trajctory with contact"+self.ENDC
				
		self.conmanSwitch([self.robot_name+'mPoseInt', self.robot_name+'mForceTransformation'], [], True)
		time.sleep(0.05)

		actual_pose = self.get_cartesian_pose()
		pose = Pose(Point(actual_pose.position.x+rel_pose.position.x, actual_pose.position.y+rel_pose.position.y, actual_pose.position.z+rel_pose.position.z), Quaternion(actual_pose.orientation.x+rel_pose.orientation.x, actual_pose.orientation.y+rel_pose.orientation.y, actual_pose.orientation.z+rel_pose.orientation.z, actual_pose.orientation.w+rel_pose.orientation.w))

		cartesianGoal = CartesianTrajectoryGoal()
		cartesianGoal.wrench_constraint = wrench_constraint
		cartesianGoal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(time_from_start), pose, Twist()))
		cartesianGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

		self.pose_client.send_goal(cartesianGoal)
		self.pose_client.wait_for_result()

		result = self.pose_client.get_result()
		code = self.cartesian_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mPoseInt', self.robot_name+'mForceTransformation'], True)

	def move_along_cartesian_trajectory(self, points):
		print self.BCOLOR+"[IRPOS] Move along cartesian trajectory"+self.ENDC

		self.conmanSwitch([self.robot_name+'mPoseInt'], [], True)

		cartesianGoal = CartesianTrajectoryGoal()
		for i in points:
			cartesianGoal.trajectory.points.append(CartesianTrajectoryPoint(i.time_from_start, i.pose, i.twist))
			#print str(i.pose.position.x)+str(i.pose.position.y)+str(i.pose.position.z)
		cartesianGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

		self.pose_client.send_goal(cartesianGoal)
		self.pose_client.wait_for_result()

		result = self.pose_client.get_result()
		code = self.cartesian_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mPoseInt'], True)

	def move_along_cartesian_circle(self, P1, P2, P3, time_from_start):
		print self.BCOLOR+"[IRPOS] Move along cartesian circle"+self.ENDC

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
		code = self.cartesian_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mPoseInt'], True)
  
	def get_cartesian_pose(self):
		self.lcp_lock.acquire()
		ret = self.last_cartesian_position
		self.lcp_lock.release()		
		return ret

	# TOOL PARAMS

	def set_tool_geometry_params(self, transformation):
		print self.BCOLOR+"[IRPOS] Set tool geometry params"+self.ENDC

		goal = CartesianTrajectoryGoal() 

		goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(0.0), transformation, Twist()))
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)

		self.tool_client.send_goal(goal)
		self.tool_client.wait_for_result()

		result = self.tool_client.get_result()
		code = self.cartesian_error_code_to_string(result.error_code)
		print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC
	
	def set_tool_physical_params(self, weight, mass_center_position):
		print self.BCOLOR+"[IRPOS] Set tool physical params"+self.ENDC

		self.tool_weight = weight
		self.tool_mass_center = mass_center_position

	# FORCE CONTROL

	def start_force_controller(self, inertia, reciprocaldamping, wrench, twist):
		print self.BCOLOR+"[IRPOS] Start force controller"+self.ENDC

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

		self.conmanSwitch([self.robot_name+'mForceTransformation'], [], True)
		time.sleep(0.05)
		self.conmanSwitch([self.robot_name+'mForceControlLaw'], [], True)

	def set_force_controller_goal(self, inertia, reciprocaldamping, wrench, twist):
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

	def stop_force_controller(self):
		print self.BCOLOR+"[IRPOS] Stop force controller"+self.ENDC

		self.conmanSwitch([], [self.robot_name+'mForceTransformation',self.robot_name+'mForceControlLaw'], True)
		return 0

	def get_force_readings(self):
		self.lw_lock.acquire()
		ret = self.last_wrench
		self.lw_lock.release()		
		return ret

	# GRIPPER

	def tfg_to_motor_position(self, motor_position, time_from_start):
		print self.BCOLOR+"[IRPOS] Tfg to motor position"+self.ENDC		
		self.conmanSwitch([self.robot_name+'tfgSplineTrajectoryGeneratorMotor'], [], True)
  
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = ['joint1']
		goal.trajectory.points.append(JointTrajectoryPoint([motor_position], [0.0], [], [], rospy.Duration(time_from_start)))
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.tfg_motor_client.send_goal(goal)
		self.tfg_motor_client.wait_for_result()

		command_result = self.tfg_motor_client.get_result()
    
		self.conmanSwitch([], [self.robot_name+'tfgSplineTrajectoryGeneratorMotor'], True)  

	def tfg_to_joint_position(self, joint_position, time_from_start):
		print self.BCOLOR+"[IRPOS] Tfg to joint position"+self.ENDC	
		self.conmanSwitch([self.robot_name+'tfgSplineTrajectoryGeneratorJoint'], [], True)
  
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = ['joint1']
		goal.trajectory.points.append(JointTrajectoryPoint([joint_position], [0.0], [], [], rospy.Duration(time_from_start)))
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.tfg_joint_client.send_goal(goal)
		self.tfg_joint_client.wait_for_result()

		command_result = self.tfg_joint_client.get_result()
     
		self.conmanSwitch([], [self.robot_name+'tfgSplineTrajectoryGeneratorJoint'], True)

	def get_tfg_motor_position(self):
		self.ltfgmp_lock.acquire()
		ret = self.last_tfg_motor_position
		self.ltfgmp_lock.release()		
		return ret
	
	def get_tfg_joint_position(self):
		self.ltfgjp_lock.acquire()
		ret = self.last_tfg_joint_position
		self.ltfgjp_lock.release()		
		return ret

# DEMO FUNCTIONS

# POSTUMENT DEMOS

def irp6p_multi_trajectory():
	irpos = IRPOS("IRpOS", "Irp6p", 6)

	motor_trajectory = [JointTrajectoryPoint([0.4, -1.5418065817051163, 0.0, 1.57, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)), JointTrajectoryPoint([10.0, 10.0, 0.0, 10.57, 10.57, -20.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(12.0))]
        irpos.move_along_motor_trajectory(motor_trajectory)

	joint_trajectory = [JointTrajectoryPoint([0.4, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(3.0)),JointTrajectoryPoint([0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(6.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)

	rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.18029263241))
	rot2 = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.3, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
	cartesianTrajectory = [CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()), CartesianTrajectoryPoint(rospy.Duration(6.0), pm.toMsg(rot), Twist()),CartesianTrajectoryPoint(rospy.Duration(9.0), pm.toMsg(rot2), Twist())]
	irpos.move_along_cartesian_trajectory(cartesianTrajectory)

	toolParams = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
	irpos.set_tool_geometry_params(toolParams)

	rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
	cartesianTrajectory = [CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()),
CartesianTrajectoryPoint(rospy.Duration(6.0), pm.toMsg(rot), Twist()),CartesianTrajectoryPoint(rospy.Duration(9.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.63691, 0.096783, 0.75634, -0.11369)), Twist())]
	irpos.move_along_cartesian_trajectory(cartesianTrajectory)

	toolParams = Pose(Point(0.0, 0.0, 0.25), Quaternion(0.0, 0.0, 0.0, 1.0))
	irpos.set_tool_geometry_params(toolParams)

	print "Irp6p 'multi_trajectory' test compleated"

def irp6p_multi_trajectory2():
	irpos = IRPOS("IRpOS", "Irp6p", 6)

	irpos.move_to_motor_position([0.4, -1.5418065817051163, 0.0, 1.57, 1.57, -2.0], 10.0)
	irpos.move_to_motor_position([10.0, 10.0, 0.0, 10.57, 10.57, -20.0], 2.0)

	irpos.move_to_joint_position([0.4, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], 3.0)
	irpos.move_to_joint_position([0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], 3.0)

	rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.18029263241))
	rot2 = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.3, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
	irpos.move_to_cartesian_pose(3.0, Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)))
	irpos.move_to_cartesian_pose(3.0,pm.toMsg(rot))
	irpos.move_to_cartesian_pose(3.0,pm.toMsg(rot2))

	toolParams = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
	irpos.set_tool_geometry_params(toolParams)

	rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
	irpos.move_to_cartesian_pose(3.0,Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)))
	irpos.move_to_cartesian_pose(3.0,pm.toMsg(rot))
	irpos.move_to_cartesian_pose(3.0,Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.63691, 0.096783, 0.75634, -0.11369)))

	toolParams = Pose(Point(0.0, 0.0, 0.25), Quaternion(0.0, 0.0, 0.0, 1.0))
	irpos.set_tool_geometry_params(toolParams)

	print "Irp6p 'multi_trajectory2' test compleated"

def irp6p_get_status():
	irpos = IRPOS("IRpOS", "Irp6p", 6)

	print('[Joint position]')
	print str(irpos.get_joint_position())
	print('[Motor position]')
	print str(irpos.get_motor_position())
	print('[Cartesian pose]')
	print str(irpos.get_cartesian_pose())
	print('[Wrench]')
	print str(irpos.get_force_readings())

	print "Irp6p 'get_status test compleated"

def irp6p_synchro_position():
	irpos = IRPOS("IRpOS", "Irp6p", 6)

	irpos.move_to_synchro_position(10.0)

	print "Irp6p 'synchro_position' test compleated"

# TRACK DEMOS

def irp6otm_multi_trajectory():
		
	print "Irp6otm 'multi_trajectory' test compleated"

def irp6otm_multi_trajectory2():
	irpos = IRPOS("IRpOS", "Irp6ot", 7)

	irpos.move_to_motor_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 10.0)
	irpos.move_to_motor_position([50.0, 10.0, 10.0, 0.0, 10.57, 10.57, -20.0], 2.0)

	irpos.move_to_joint_position([0.0, 0.4, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], 3.0)
	irpos.move_to_joint_position([0.1, 0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -1.57], 3.0)

	print "Irp6otm 'multi_trajectory2' test compleated"

def irp6otm_get_status():
	irpos = IRPOS("IRpOS", "Irp6ot", 7)

	print('[Joint position]')
	print str(irpos.get_joint_position())
	print('[Motor position]')
	print str(irpos.get_motor_position())
	print('[Cartesian pose]')
	print str(irpos.get_cartesian_pose())
	print('[Wrench]')
	print str(irpos.get_force_readings())

	print "Irp6otm 'get_status' test compleated"

def irp6otm_synchro_position():
	irpos = IRPOS("IRpOS", "Irp6ot", 7)

	irpos.move_to_synchro_position(10.0)

	print "Irp6otm 'synchro_position' test compleated"
	
def test():
	print 'START TEST'

	print 'END TEST'

#MAIN

if __name__ == '__main__':
	if sys.argv[1]=="p_m":
		irp6p_multi_trajectory()	
	elif sys.argv[1]=="p_m2":
		irp6p_multi_trajectory2()	
	elif sys.argv[1]=="p_s":
		irp6p_get_status()
	elif sys.argv[1]=="p_sp":
		irp6p_synchro_position()
	elif sys.argv[1]=="ot_m":
		irp6otm_multi_trajectory()	
	elif sys.argv[1]=="ot_m2":
		irp6otm_multi_trajectory2()	
	elif sys.argv[1]=="ot_s":
		irp6otm_get_status()	
	elif sys.argv[1]=="ot_sp":
		irp6otm_synchro_position()
	elif sys.argv[1]=="test":
		test()
	

