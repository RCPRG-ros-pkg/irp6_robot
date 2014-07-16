#!/usr/bin/env python
from IRPOS import *

if __name__ == '__main__':
	irpos = IRPOS("IRpOS")

	irpos.stop_force_controller()

	print "Force control OFF"
