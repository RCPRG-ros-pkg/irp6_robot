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


class Irp6MotorStatus():
    def __init__(self, context):
        self.is_responding = False
        self.is_synchronised = False
        self.motion_in_progress = False
        self.synchro_in_progress = False
        self.is_emergency_stop_activated  = False


    def is_eq(self,s1):
        if ((self.is_responding == s1.is_responding)
        and (self.is_synchronised == s1.is_synchronised)
        and (self.motion_in_progress == s1.motion_in_progress)
        and (self.synchro_in_progress == s1.synchro_in_progress)
        and (self.is_emergency_stop_activated == s1.is_emergency_stop_activated)):
            return True
        else:
            return False


    def assign(self,s1):
        self.is_responding = s1.is_responding
        self.is_synchronised = s1.is_synchronised
        self.motion_in_progress = s1.motion_in_progress
        self.synchro_in_progress = s1.synchro_in_progress
        self.is_emergency_stop_activated = s1.is_emergency_stop_activated


