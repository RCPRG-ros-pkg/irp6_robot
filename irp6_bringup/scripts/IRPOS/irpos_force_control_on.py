#!/usr/bin/env python
from IRPOS import *

if __name__ == '__main__':
	irpos = IRPOS("IRpOS")

	irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156))
	irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.002, 0.002, 0.002), Vector3(0.05, 0.05, 0.05)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))

	print "Force control ON"
