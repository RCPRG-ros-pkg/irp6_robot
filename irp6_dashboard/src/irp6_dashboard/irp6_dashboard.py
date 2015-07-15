import os
import rospy
from rosgraph import rosenv
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget


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

    def get_widgets(self):
        return [
                [self._monitor, self._console]
               ]

    def shutdown_dashboard(self):
        pass
	