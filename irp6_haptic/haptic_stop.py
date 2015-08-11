#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Robot Control and Pattern Recognition Group, Warsaw University of Technology
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of the <organization> nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import tf
import actionlib

from controller_manager_msgs.srv import *
from std_msgs.msg import *
from diagnostic_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from force_control_msgs.msg import *
from tf.transformations import *

#from PyKDL import *
import PyKDL
import tf_conversions.posemath as pm

if __name__ == '__main__':
  rospy.init_node('haptic_stop')
  rospy.wait_for_service('/irp6p_manager/switch_controller')
  conmanSwitchIrp6p = rospy.ServiceProxy('/irp6p_manager/switch_controller', SwitchController)
  
  print "servers irp6p ok"
  
  rospy.wait_for_service('/irp6ot_manager/switch_controller')
  conmanSwitchIrp6ot = rospy.ServiceProxy('/irp6ot_manager/switch_controller', SwitchController)
  
  print "servers irp6ot ok"
  
  rospy.wait_for_service('/haptic_manager/switch_controller')
  conmanSwitchHaptic = rospy.ServiceProxy('/haptic_manager/switch_controller', SwitchController)
  
  print "haptic server ok"
  
  conmanSwitchIrp6p([], ['Irp6pmForceTransformation','Irp6pmForceControlLaw'], True)
  conmanSwitchIrp6ot([], ['Irp6otmForceTransformation','Irp6otmForceControlLaw'], True)
  conmanSwitchHaptic([], ['Irp6Haptic'], True)
      
  print 'finish'
  
  
  
  
  