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

class DashboardStatus():
    def __init__(self, context):
        self.is_synchronised = False
        self.is_synchronised_previous = False
        self.is_emergency_stop_activated = False
        self.is_emergency_stop_activated_previous = False


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
        
        self.status = DashboardStatus(self)
        
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
                             self.status.is_synchronised = True
                         else:
                             self.status.is_synchronised = False
                     elif kv.key == 'HardwarePanic':
                         if kv.value == 'TRUE':
                             self.status.is_emergency_stop_activated = True
                         else:
                             self.status.is_emergency_stop_activated = False
        
        if ((self._motors_button.status.motion_in_progress != self._motors_button.status.motion_in_progress_previous)
        or (self.status.is_emergency_stop_activated != self.status.is_emergency_stop_activated_previous)
        or (self.status.is_synchronised != self.status.is_synchronised_previous)
        or (self._motors_button.status.synchro_in_progress != self._motors_button.status.synchro_in_progress_previous)):
            self.change_motors_widget_state()
        
        self._motors_button.status.motion_in_progress_previous = self._motors_button.status.motion_in_progress
        self.status.is_emergency_stop_activated_previous = self.status.is_emergency_stop_activated
        self.status.is_synchronised_previous = self.status.is_synchronised
        self._motors_button.status.synchro_in_progress_previous = self._motors_button.status.synchro_in_progress


    def change_motors_widget_state(self):
        # print "change_motors_widget_state"
        if self.status.is_emergency_stop_activated == True:
            self._motors_button.set_error()
            self._motors_button.disable_all_actions()
            self._motors_button.setToolTip(self.tr("Motors: Hardware Panic, Check emergency stop, Restart hardware, deployer and rqt"))
        elif self.status.is_synchronised == False:
            if self._motors_button.status.synchro_in_progress == False:
                self._motors_button.set_warn()
                self._motors_button.enable_pre_synchro_actions()
                self._motors_button.setToolTip(self.tr("Motors: Robot not synchronised, Execute synchronisation and wait for operation finish"))
            else:
                self._motors_button.set_stale()
                self._motors_button.disable_all_actions()
                self._motors_button.setToolTip(self.tr("Motors: Synchronisation in progress"))
        elif self._motors_button.status.motion_in_progress == True:
            self._motors_button.set_stale()
            self._motors_button.disable_all_actions()
            self._motors_button.setToolTip(self.tr("Motors: Motion in progress, wait for execution finish"))
            self._motors_button.status.synchro_in_progress = False
        elif self._motors_button.status.motion_in_progress == False:
            self._motors_button.set_ok()
            self._motors_button.enable_post_synchro_actions()
            self._motors_button.setToolTip(self.tr("Motors: Robot synchronised and waiting for command"))
            self._motors_button.status.synchro_in_progress = False
            
	