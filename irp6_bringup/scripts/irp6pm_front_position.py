#!/usr/bin/env python

from irpos import *
import sys
import math

if __name__ == '__main__':
	
	irpos = IRPOS("irp6pm_front_position", "Irp6p", 6)
	half_pi = math.pi/2
	
	if sys.argv[1] == '1':
		print "Robot patrzy w dol"
		# reczne ustawienie: [7.412760409739285e-06, -1.4791420483915523, -0.16172125899257106, 0.07006621675194569, 4.712388138719054, -1.5707949127454675]
		irpos.move_to_joint_position([0, -half_pi, 0, 0, 3*half_pi, -half_pi], 10.00)
		
	elif sys.argv[1] == '2':
		print "Robot patrzy na nas"
		# reczne ustawienie: [-0.011544899465765387, -1.7562378162659877, 0.19518125616433443, 1.4450377613150052, 1.5245715307209733, -1.5808215788580826]
		irpos.move_to_joint_position([0, -half_pi, 0, half_pi, half_pi, -half_pi], 10.00)
		
	else:
		print "Invalid argument"
