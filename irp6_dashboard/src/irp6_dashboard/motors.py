# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Ported from pr2_motors: Vincent Rabaud, Aldebaran Robotics, 2014
#

from python_qt_binding.QtGui import QMessageBox

import actionlib
import rospy
from rqt_robot_dashboard.widgets import MenuDashWidget
import std_srvs.srv

from irpos import *


class Motors(MenuDashWidget):
    """
    Dashboard widget to display motor state and allow interaction.
    """
    def __init__(self, context):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        :param reset_callback: calback for the "reset" action
        :type reset_callback: function
        :param halt_callback: calback for the "reset" action
        :type halt_callback: function
        """
        ok_icon = ['bg-green.svg', 'ic-motors.svg']
        warn_icon = ['bg-yellow.svg', 'ic-motors.svg', 'ol-warn-badge.svg']
        err_icon = ['bg-red.svg', 'ic-motors.svg', 'ol-err-badge.svg']
        stale_icon = ['bg-grey.svg', 'ic-motors.svg', 'ol-stale-badge.svg']

        icons = [ok_icon, warn_icon, err_icon, stale_icon]

        super(Motors, self).__init__('Motors', icons)
        self.update_state(3)
        
        self.irp6p_move_to_synchro_pos_action = self.add_action('Irp6p move to synchro pos', self.irp6p_move_to_synchro_pos)
        self.irp6ot_move_to_synchro_pos_action = self.add_action('Irp6ot move to synchro pos', self.irp6ot_move_to_synchro_pos)
        # self.add_action('Motion one', self.on_motion_one)
        self.motion_in_progress_state = False
        self.motion_in_progress_state_previous = False

        self.irpos = IRPOS("", "Irp6p", 6)
        
        
    def enable_post_synchro_actions(self):
         self.irp6p_move_to_synchro_pos_action.setDisabled(False)
         self.irp6ot_move_to_synchro_pos_action.setDisabled(False)
        
    def disable_all_actions(self):
        self.irp6p_move_to_synchro_pos_action.setDisabled(True)
        self.irp6ot_move_to_synchro_pos_action.setDisabled(True)

    def irp6p_done_callback(self,state, result):
        print "self Done callback"
        self.conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorMotor','Irp6pmSplineTrajectoryGeneratorJoint','Irp6pmPoseInt','Irp6pmForceControlLaw','Irp6pmForceTransformation'], True)
        self.motion_in_progress_state = False
        self.enable_post_synchro_actions()
        
    def irp6ot_done_callback(self,state, result):
        print "self Done callback"
        self.conmanSwitch([], ['Irp6otmSplineTrajectoryGeneratorMotor','Irp6otmSplineTrajectoryGeneratorJoint','Irp6otmPoseInt','Irp6otmForceControlLaw','Irp6otmForceTransformation'], True)
        self.motion_in_progress_state = False
        self.enable_post_synchro_actions()

        
    def set_ok(self):
        self.update_state(0)

    def set_warn(self):
        self.update_state(1)

    def set_error(self):
        self.update_state(2)

    def set_stale(self):
        self.update_state(3)

    def irp6p_move_to_synchro_pos(self):
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        self.conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorMotor','Irp6pmSplineTrajectoryGeneratorJoint','Irp6pmPoseInt','Irp6pmForceControlLaw','Irp6pmForceTransformation'], True)
  
  
        self.conmanSwitch(['Irp6pmSplineTrajectoryGeneratorMotor'], [], True)
    
        self.client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_motor', FollowJointTrajectoryAction)
        self.client.wait_for_server()

        print 'server ok'

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        goal.trajectory.points.append(JointTrajectoryPoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)))
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

        self.client.send_goal(goal, self.irp6p_done_callback)
        
    
        print 'za send goal'
        self.motion_in_progress_state = True
        self.disable_all_actions()
        # client.wait_for_result()
        # command_result = client.get_result()
  
        # conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorJoint'], True)
        # time.sleep(20)
        # print "Irp6p move to synchro pos finish"


    def irp6ot_move_to_synchro_pos(self):
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        self.conmanSwitch([], ['Irp6otmSplineTrajectoryGeneratorMotor','Irp6otmSplineTrajectoryGeneratorJoint','Irp6otmPoseInt','Irp6otmForceControlLaw','Irp6otmForceTransformation'], True)
  
  
        self.conmanSwitch(['Irp6otmSplineTrajectoryGeneratorMotor'], [], True)
    
        self.client = actionlib.SimpleActionClient('/irp6ot_arm/spline_trajectory_action_motor', FollowJointTrajectoryAction)
        self.client.wait_for_server()

        print 'server ok'

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        goal.trajectory.points.append(JointTrajectoryPoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)))
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

        self.client.send_goal(goal, self.irp6ot_done_callback)
  
        print 'za send goal'

        self.motion_in_progress_state = True
        self.disable_all_actions()
        # client.wait_for_result()
        # command_result = client.get_result()
  
        # conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorJoint'], True)
        # time.sleep(20)
        # print "Irp6ot move to synchro pos finish"

    def on_motion_one(self):
        QMessageBox.information(self, 'Caution', 
                     'Irpos motion one', QMessageBox.Ok)

        motor_trajectory = [JointTrajectoryPoint([0.4, -1.5418065817051163, 0.0, 1.57, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)), JointTrajectoryPoint([10.0, 10.0, 0.0, 10.57, 10.57, -20.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(12.0))]
        self.irpos.move_along_motor_trajectory(motor_trajectory)

        joint_trajectory = [JointTrajectoryPoint([0.4, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(3.0)),JointTrajectoryPoint([0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(6.0))]
        self.irpos.move_along_joint_trajectory(joint_trajectory)

        rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.18029263241))
        rot2 = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.3, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
        cartesianTrajectory = [CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()), CartesianTrajectoryPoint(rospy.Duration(6.0), pm.toMsg(rot), Twist()),CartesianTrajectoryPoint(rospy.Duration(9.0), pm.toMsg(rot2), Twist())]
        self.irpos.move_along_cartesian_trajectory(cartesianTrajectory)

        toolParams = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        self.irpos.set_tool_geometry_params(toolParams)

        rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
        cartesianTrajectory = [CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()),
        CartesianTrajectoryPoint(rospy.Duration(6.0), pm.toMsg(rot), Twist()),CartesianTrajectoryPoint(rospy.Duration(9.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.63691, 0.096783, 0.75634, -0.11369)), Twist())]
        self.irpos.move_along_cartesian_trajectory(cartesianTrajectory)

        toolParams = Pose(Point(0.0, 0.0, 0.25), Quaternion(0.0, 0.0, 0.0, 1.0))
        self.irpos.set_tool_geometry_params(toolParams)

        print "Irp6p 'multi_trajectory' test completed"
