import os
import rospy
from rosgraph import rosenv
import rospkg

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .hi_motors import HiMotors
from .irp6p_motors import Irp6pMotors
from .irp6ot_motors import Irp6otMotors

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
        self._hi_motors_button = HiMotors(self.context)
        self._irp6p_motors_button = Irp6pMotors(self.context)
        self._irp6ot_motors_button = Irp6otMotors(self.context)
        
        self._agg_sub = rospy.Subscriber('diagnostics_agg', DiagnosticArray, self.new_diagnostic_message)
        
        self.status = DashboardStatus(self)
        
        self.change_motors_widget_state()


    def get_widgets(self):
        return [
                [self._monitor, self._console, self._hi_motors_button, self._irp6p_motors_button, self._irp6ot_motors_button]
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
        
        if ((self._irp6p_motors_button.status.motion_in_progress != self._irp6p_motors_button.status.motion_in_progress_previous)
        or (self._irp6ot_motors_button.status.motion_in_progress != self._irp6ot_motors_button.status.motion_in_progress_previous)
        or (self.status.is_emergency_stop_activated != self.status.is_emergency_stop_activated_previous)
        or (self.status.is_synchronised != self.status.is_synchronised_previous)
        or (self._hi_motors_button.status.synchro_in_progress != self._hi_motors_button.status.synchro_in_progress_previous)):
            self.change_motors_widget_state()
        
        self._hi_motors_button.status.synchro_in_progress_previous = self._hi_motors_button.status.synchro_in_progress
        self._irp6p_motors_button.status.motion_in_progress_previous = self._irp6p_motors_button.status.motion_in_progress
        self._irp6ot_motors_button.status.motion_in_progress_previous = self._irp6ot_motors_button.status.motion_in_progress
        self.status.is_emergency_stop_activated_previous = self.status.is_emergency_stop_activated
        self.status.is_synchronised_previous = self.status.is_synchronised


    def emergency_stop_activated_ui_state(self):
        self._hi_motors_button.set_error()
        self._hi_motors_button.disable_all_actions()
        self._hi_motors_button.setToolTip(self.tr("Hi Motors: Hardware Panic, Check emergency stop, Restart hardware, deployer and rqt"))
        self._irp6p_motors_button.set_error()
        self._irp6p_motors_button.disable_all_actions()
        self._irp6p_motors_button.setToolTip(self.tr("Irp6p Motors: Hardware Panic, Check emergency stop, Restart hardware, deployer and rqt"))
        self._irp6ot_motors_button.set_error()
        self._irp6ot_motors_button.disable_all_actions()
        self._irp6ot_motors_button.setToolTip(self.tr("Irp6ot Motors: Hardware Panic, Check emergency stop, Restart hardware, deployer and rqt"))


    def not_synchronised_no_motion_ui_state(self):
        self._hi_motors_button.set_warn()
        self._hi_motors_button.enable_pre_synchro_actions()
        self._hi_motors_button.setToolTip(self.tr("Hi Motors: Robot not synchronised, Execute synchronisation and wait for operation finish"))
        self._irp6p_motors_button.set_warn()
        self._irp6p_motors_button.enable_pre_synchro_actions()
        self._irp6p_motors_button.setToolTip(self.tr("Irp6p Motors: Robot not synchronised, Execute synchronisation and wait for operation finish"))
        self._irp6ot_motors_button.set_warn()
        self._irp6ot_motors_button.enable_pre_synchro_actions()
        self._irp6ot_motors_button.setToolTip(self.tr("Irp6ot Motors: Robot not synchronised, Execute synchronisation and wait for operation finish"))


    def synchro_in_progress_ui_state(self):
        self._hi_motors_button.set_stale()
        self._hi_motors_button.disable_all_actions()
        self._hi_motors_button.setToolTip(self.tr("Hi Motors: Synchronisation in progress"))
        self._irp6p_motors_button.set_stale()
        self._irp6p_motors_button.disable_all_actions()
        self._irp6p_motors_button.setToolTip(self.tr("Irp6p Motors: Synchronisation in progress"))
        self._irp6ot_motors_button.set_stale()
        self._irp6ot_motors_button.disable_all_actions()
        self._irp6ot_motors_button.setToolTip(self.tr("Irp6ot Motors: Synchronisation in progress"))


    def robot_synchronised_ui_state(self):
        self._hi_motors_button.set_stale()
        self._hi_motors_button.disable_all_actions()
        self._hi_motors_button.setToolTip(self.tr("Hi Motors: Robot synchronised"))
        self._hi_motors_button.status.synchro_in_progress = False
        if self._irp6p_motors_button.status.motion_in_progress == True:
            self._irp6p_motors_button.set_stale()
            self._irp6p_motors_button.disable_all_actions()
            self._irp6p_motors_button.setToolTip(self.tr("Irp6p Motors: Robot in motion"))
        else:
            self._irp6p_motors_button.set_ok()
            self._irp6p_motors_button.enable_post_synchro_actions()
            self._irp6p_motors_button.setToolTip(self.tr("Irp6p Motors: Robot synchronised and waiting for command"))
        if self._irp6ot_motors_button.status.motion_in_progress == True:
            self._irp6ot_motors_button.set_stale()
            self._irp6ot_motors_button.disable_all_actions()
            self._irp6ot_motors_button.setToolTip(self.tr("Irp6ot Motors: Robot in motion"))
        else:
            self._irp6ot_motors_button.set_ok()
            self._irp6ot_motors_button.enable_post_synchro_actions()
            self._irp6ot_motors_button.setToolTip(self.tr("Irp6ot Motors: Robot synchronised and waiting for command"))


    def change_motors_widget_state(self):
        # print "change_motors_widget_state"
        if self.status.is_emergency_stop_activated == True:
            self.emergency_stop_activated_ui_state()
        elif self.status.is_synchronised == False:
            if self._hi_motors_button.status.synchro_in_progress == False:
                self.not_synchronised_no_motion_ui_state()
            else:
                self.synchro_in_progress_ui_state()
        else:
            self.robot_synchronised_ui_state()


