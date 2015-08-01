import os
import rospy
from rosgraph import rosenv
import rospkg


from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .irp6p_motors import Irp6pMotors
from .irp6ot_motors import Irp6otMotors
from .conveyor_motors import ConveyorMotors

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
        self._irp6p_motors_button = Irp6pMotors(self.context)
        self._irp6ot_motors_button = Irp6otMotors(self.context)
        self._conveyor_motors_button = ConveyorMotors(self.context)
        
        self._agg_sub = rospy.Subscriber('diagnostics', DiagnosticArray, self.new_diagnostic_message_diagnostics)
        # self._agg_sub = rospy.Subscriber('diagnostics_agg', DiagnosticArray, self.new_diagnostic_message_diagnostics_agg)
        






    def get_widgets(self):
        return [
                [self._monitor, self._console, self._irp6ot_motors_button, self._irp6p_motors_button, self._conveyor_motors_button]
               ]


    def shutdown_dashboard(self):
        self._agg_sub.unregister()


    def new_diagnostic_message_diagnostics(self, msg):
        """
        callback to process dashboard_agg messages
        """
        
        self._dashboard_message = msg
        for status in msg.status:
            if status.name == 'Irp6p Hardware Interface':
                self._irp6p_motors_button.interpret_diagnostic_message(status)
            elif status.name == 'Irp6ot Hardware Interface':
                self._irp6ot_motors_button.interpret_diagnostic_message(status)
            elif status.name == 'Conveyor Hardware Interface':
                self._conveyor_motors_button.interpret_diagnostic_message(status)


    def new_diagnostic_message_diagnostics_agg(self, msg):
        """
        callback to process dashboard_agg messages
        """
        
        self._dashboard_message = msg
        for status in msg.status:
            if status.name == '/HardwareInterfaces/Irp6p/Irp6p Hardware Interface':
                if (status.level != 3):
                    self._irp6p_motors_button.interpret_diagnostic_message(status)
            elif status.name == '/HardwareInterfaces/Irp6ot/Irp6ot Hardware Interface':
                if (status.level != 3):
                    self._irp6ot_motors_button.interpret_diagnostic_message(status)
            elif status.name == '/HardwareInterfaces/Conveyor/Conveyor Hardware Interface':
                if (status.level != 3):
                    self._conveyor_motors_button.interpret_diagnostic_message(status)


    def save_settings(self, plugin_settings, instance_settings):
        self._console.save_settings(plugin_settings, instance_settings)
        self._monitor.save_settings(plugin_settings, instance_settings)


    def restore_settings(self, plugin_settings, instance_settings):
        self._console.restore_settings(plugin_settings, instance_settings)
        self._monitor.restore_settings(plugin_settings, instance_settings)


