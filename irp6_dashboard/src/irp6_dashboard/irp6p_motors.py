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

from std_msgs.msg import *
from irpos import *
from .irp6_motor_status import Irp6MotorStatus

class Irp6pMotors(MenuDashWidget):
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

        super(Irp6pMotors, self).__init__('Irp6pMotors', icons)
        
        self.irp6p_move_to_synchro_pos_action = self.add_action('Irp6p move to synchro pos', self.irp6p_move_to_synchro_pos)
        self.irp6p_move_to_front_pos_action = self.add_action('Irp6p move to front pos', self.irp6p_move_to_front_pos)
        self.synchronise_action = self.add_action('Synchronise', self.synchronise)

        
        self.status = Irp6MotorStatus(self)
        self.previous_status = Irp6MotorStatus(self)
        
        self.change_motors_widget_state()


    def set_ok(self):
        self.update_state(0)


    def set_warn(self):
        self.update_state(1)


    def set_error(self):
        self.update_state(2)


    def set_stale(self):
        self.update_state(3)


    def enable_post_synchro_actions(self):
        self.irp6p_move_to_synchro_pos_action.setDisabled(False)
        self.irp6p_move_to_front_pos_action.setDisabled(False)
        self.synchronise_action.setDisabled(True)
         
         
    def enable_pre_synchro_actions(self):
        self.irp6p_move_to_synchro_pos_action.setDisabled(True)
        self.irp6p_move_to_front_pos_action.setDisabled(True)
        self.synchronise_action.setDisabled(False)


    def disable_all_actions(self):
        self.irp6p_move_to_synchro_pos_action.setDisabled(True)
        self.irp6p_move_to_front_pos_action.setDisabled(True)
        self.synchronise_action.setDisabled(True)


    def irp6p_done_callback(self,state, result):
        self.conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorMotor','Irp6pmSplineTrajectoryGeneratorJoint','Irp6pmPoseInt','Irp6pmForceControlLaw','Irp6pmForceTransformation'], True)
        self.status.motion_in_progress = False


    def init_conman(self):
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)


    def irp6p_move_to_synchro_pos(self):
        self.init_conman()
        self.conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorMotor','Irp6pmSplineTrajectoryGeneratorJoint','Irp6pmPoseInt','Irp6pmForceControlLaw','Irp6pmForceTransformation'], True)
  
        self.conmanSwitch(['Irp6pmSplineTrajectoryGeneratorMotor'], [], True)
    
        self.client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_motor', FollowJointTrajectoryAction)
        self.client.wait_for_server()

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        goal.trajectory.points.append(JointTrajectoryPoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)))
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

        self.client.send_goal(goal, self.irp6p_done_callback)

        self.status.motion_in_progress = True


    def irp6p_move_to_front_pos(self):
        self.init_conman()
        self.conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorMotor','Irp6pmSplineTrajectoryGeneratorJoint','Irp6pmPoseInt','Irp6pmForceControlLaw','Irp6pmForceTransformation'], True)
  
        self.conmanSwitch(['Irp6pmSplineTrajectoryGeneratorJoint'], [], True)
    
        self.client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
        self.client.wait_for_server()

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        goal.trajectory.points.append(JointTrajectoryPoint([0.0, -1.57, 0.0, 1.5, 1.57, -1.57], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)))
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
        
        self.client.send_goal(goal, self.irp6p_done_callback)
        self.status.motion_in_progress = True


    def synchronise(self):
        pub = rospy.Publisher('/irp6p_hardware_interface/do_synchro_in', std_msgs.msg.Bool, queue_size=0)
        rospy.sleep(0.5)
        goal = std_msgs.msg.Bool()
        goal.data = True
        pub.publish(goal)
        self.status.synchro_in_progress = True


    def change_motors_widget_state(self):
        # print "change_motors_widget_state"
        if self.status.is_emergency_stop_activated == True:
            self.set_error()
            self.disable_all_actions()
            self.setToolTip(self.tr("Irp6p Motors: Hardware Panic, Check emergency stop, Restart hardware and deployer"))
        elif self.status.is_synchronised == False:
            if self.status.synchro_in_progress == False:
                self.set_warn()
                self.enable_pre_synchro_actions()
                self.setToolTip(self.tr("Irp6p Motors: Robot not synchronised, Execute synchronisation and wait for operation finish"))
            else:
                self.set_stale()
                self.disable_all_actions()
                self.setToolTip(self.tr("Irp6p Motors: Synchronisation in progress"))
        else:
            if self.status.motion_in_progress == True:
                self.set_stale()
                self.disable_all_actions()
                self.setToolTip(self.tr("Irp6p Motors: Robot in motion"))
            else:
                self.set_ok()
                self.enable_post_synchro_actions()
                self.setToolTip(self.tr("Irp6p Motors: Robot synchronised and waiting for command"))
                
        
        self.previous_status.assign(self.status)
