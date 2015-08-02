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

from python_qt_binding.QtGui import QMessageBox

import actionlib
import rospy
from rqt_robot_dashboard.widgets import MenuDashWidget
import std_srvs.srv
import datetime, threading, time

from std_msgs.msg import *
from irpos import *


class Irp6MotorStatus():
    def __init__(self, context):
        self.is_busy = False
        self.is_responding = False
        self.is_synchronised = False
        self.motion_in_progress = False
        self.synchro_in_progress = False
        self.is_emergency_stop_activated  = False


    def is_eq(self,s1):
        if ((self.is_busy == s1.is_busy)
        and(self.is_responding == s1.is_responding)
        and (self.is_synchronised == s1.is_synchronised)
        and (self.motion_in_progress == s1.motion_in_progress)
        and (self.synchro_in_progress == s1.synchro_in_progress)
        and (self.is_emergency_stop_activated == s1.is_emergency_stop_activated)):
            return True
        else:
            return False


    def assign(self,s1):
        self.is_busy = s1.is_busy
        self.is_responding = s1.is_responding
        self.is_synchronised = s1.is_synchronised
        self.motion_in_progress = s1.motion_in_progress
        self.synchro_in_progress = s1.synchro_in_progress
        self.is_emergency_stop_activated = s1.is_emergency_stop_activated



class Irp6Motors(MenuDashWidget):
    """
    Dashboard widget to display motor state and allow interaction.
    """
    def __init__(self, name_):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        :param reset_callback: calback for the "reset" action
        :type reset_callback: function
        :param halt_callback: calback for the "reset" action
        :type halt_callback: function
        """
        
        self.name = name_
        
        ok_icon = ['bg-green.svg', 'ic-motors.svg']
        warn_icon = ['bg-yellow.svg', 'ic-motors.svg', 'ol-warn-badge.svg']
        err_icon = ['bg-red.svg', 'ic-motors.svg', 'ol-err-badge.svg']
        stale_icon = ['bg-grey.svg', 'ic-motors.svg', 'ol-stale-badge.svg']

        icons = [ok_icon, warn_icon, err_icon, stale_icon]
        
        super(Irp6Motors, self).__init__(self.name, icons)
        
        self.status = Irp6MotorStatus(self)
        self.previous_status = Irp6MotorStatus(self)
        
        self.diagnostic_messages_number = 0


    def monitor_robot_activity(self):
        next_call = time.time()
        previous_diagnostic_messages_number = 0
        while True:
            if (self.diagnostic_messages_number == previous_diagnostic_messages_number):
                self.status.is_responding = False
            else:
                self.status.is_responding = True
                if (self.previous_status.is_responding == False):
                    self.status.motion_in_progress = False
                    self.status.synchro_in_progress = False
                previous_diagnostic_messages_number = self.diagnostic_messages_number
            if (not self.status.is_eq(self.previous_status)):
                self.change_motors_widget_state()
            next_call = next_call+0.5; # wariant dla /diagnostics z poszczegolnych robotow
            # next_call = next_call+1.5; # wariant dla /diagnostics_agg 
            time.sleep(next_call - time.time())


    def set_ok(self):
        self.update_state(0)


    def set_warn(self):
        self.update_state(1)


    def set_error(self):
        self.update_state(2)


    def set_stale(self):
        self.update_state(3)
        
        
    def enable_post_synchro_actions(self):
        self.move_to_synchro_pos_action.setDisabled(False)
        self.move_to_front_pos_action.setDisabled(False)
        self.synchronise_action.setDisabled(True)

    def enable_pre_synchro_actions(self):
        self.move_to_synchro_pos_action.setDisabled(True)
        self.move_to_front_pos_action.setDisabled(True)
        self.synchronise_action.setDisabled(False)


    def disable_all_actions(self):
        self.move_to_synchro_pos_action.setDisabled(True)
        self.move_to_front_pos_action.setDisabled(True)
        self.synchronise_action.setDisabled(True)


    def change_motors_widget_state(self):
        # print self.name + ": change_motors_widget_state"
        if self.status.is_responding == False:
            self.set_stale()
            self.disable_all_actions()
            self.setToolTip(self.tr(self.name + ": Not responding"))
        elif self.status.is_emergency_stop_activated == True:
            self.set_error()
            self.disable_all_actions()
            self.setToolTip(self.tr(self.name + ": Hardware Panic, Check emergency stop, Restart hardware and deployer"))
        elif self.status.is_synchronised == False:
            if self.status.synchro_in_progress == False:
                self.set_warn()
                self.enable_pre_synchro_actions()
                self.setToolTip(self.tr(self.name + ": Robot not synchronised, Execute synchronisation and wait for operation finish"))
            else:
                self.set_warn()
                self.disable_all_actions()
                self.setToolTip(self.tr(self.name + ": Synchronisation in progress"))
        else:
            if self.status.is_busy == True:
                self.set_warn()
                self.disable_all_actions()
                if self.status.motion_in_progress == True:
                    self.setToolTip(self.tr(self.name + ": Robot in motion (internal call)"))
                else:
                    self.setToolTip(self.tr(self.name + ": Robot in motion (external call)"))
            else:
                self.set_ok()
                self.enable_post_synchro_actions()
                self.setToolTip(self.tr(self.name + ": Robot synchronised and waiting for command"))
                
        self.previous_status.assign(self.status)


    def interpret_diagnostic_message(self,status):
        self.diagnostic_messages_number += 1
        self.status.is_responding = True 
        for kv in status.values:
            if kv.key == 'Synchro':
                if kv.value == 'TRUE':
                    self.status.is_synchronised = True
                else:
                    self.status.is_synchronised = False
            elif kv.key == 'HardwarePanic':
                if kv.value == 'TRUE':
                    self.status.is_emergency_stop_activated = True
                else:
                    self.status.is_emergency_stop_activated = False
            elif kv.key == 'HardwareBusy':
                if kv.value == 'TRUE':
                    self.status.is_busy = True
                else:
                    self.status.is_busy = False
                    
            if (not self.status.is_eq(self.previous_status)):
                self.change_motors_widget_state()
                
#        print self.name + ": new diag message, synchro: " + str(self.status.is_synchronised) + " hardware panic: " + str(self.status.is_emergency_stop_activated)


