import os
import rospy
from rosgraph import rosenv
import rospkg

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .motors import Motors

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.monitor_dash_widget import MonitorDashWidget
from rqt_robot_dashboard.console_dash_widget import ConsoleDashWidget

class Irp6Dashboard(Dashboard):
    def setup(self, context):
        self.name = 'Irp6 Dashboard (%s)'%rosenv.get_master_uri()

        
        # Diagnostics
        self._monitor = MonitorDashWidget(self.context)

        # Rosout
        self._console = ConsoleDashWidget(self.context, minimal=False)
        
        ## Motors
        self._motors_button = Motors(self.context)
        
        self._agg_sub = rospy.Subscriber('diagnostics_agg', DiagnosticArray, self.new_diagnostic_message)
        
        self.is_synchronised_state = False
        self.is_synchronised_state_previous = False
        self.is_emergency_stop_activated_state = False
        self.is_emergency_stop_activated_state_previous = False
        
        self.change_motors_widget_state()


    def get_widgets(self):
        return [
                [self._monitor, self._console, self._motors_button]
               ]


    def shutdown_dashboard(self):
        self._agg_sub.unregister()


    def new_diagnostic_message(self, msg):
        """
        callback to process dashboard_agg messages
        """
        self._dashboard_message = msg
        for status in msg.status:
            if status.name == 'Hardware Interface':
                for kv in status.values:
                     if kv.key == 'Synchro':
                         if kv.value == 'TRUE':
                             self.is_synchronised_state = True
                         else:
                             self.is_synchronised_state = False
                     elif kv.key == 'HardwarePanic':
                         if kv.value == 'TRUE':
                             self.is_emergency_stop_activated_state = True
                         else:
                             self.is_emergency_stop_activated_state = False
        
        if ((self._motors_button.motion_in_progress_state != self._motors_button.motion_in_progress_state_previous)
        or (self.is_emergency_stop_activated_state != self.is_emergency_stop_activated_state_previous)
        or (self.is_synchronised_state != self.is_synchronised_state_previous)
        or (self._motors_button.synchro_in_progress_state != self._motors_button.synchro_in_progress_state_previous)):
            self.change_motors_widget_state()
        
        self._motors_button.motion_in_progress_state_previous = self._motors_button.motion_in_progress_state
        self.is_emergency_stop_activated_state_previous = self.is_emergency_stop_activated_state
        self.is_synchronised_state_previous = self.is_synchronised_state
        self._motors_button.synchro_in_progress_state_previous = self._motors_button.synchro_in_progress_state


    def change_motors_widget_state(self):
        # print "change_motors_widget_state"
        if self.is_emergency_stop_activated_state == True:
            self._motors_button.set_error()
            self._motors_button.disable_all_actions()
            self._motors_button.setToolTip(self.tr("Motors: Hardware Panic, Check emergency stop, Restart hardware, deployer and rqt"))
        elif self.is_synchronised_state == False:
            if self._motors_button.synchro_in_progress_state == False:
                self._motors_button.set_warn()
                self._motors_button.enable_pre_synchro_actions()
                self._motors_button.setToolTip(self.tr("Motors: Robot not synchronised, Execute synchronisation and wait for operation finish"))
            else:
                self._motors_button.set_stale()
                self._motors_button.disable_all_actions()
                self._motors_button.setToolTip(self.tr("Motors: Synchronisation in progress"))
        elif self._motors_button.motion_in_progress_state == True:
            self._motors_button.set_stale()
            self._motors_button.disable_all_actions()
            self._motors_button.setToolTip(self.tr("Motors: Motion in progress, wait for execution finish"))
            self._motors_button.synchro_in_progress_state = False
        elif self._motors_button.motion_in_progress_state == False:
            self._motors_button.set_ok()
            self._motors_button.enable_post_synchro_actions()
            self._motors_button.setToolTip(self.tr("Motors: Robot synchronised and waiting for command"))
            self._motors_button.synchro_in_progress_state = False
            
	