#!/usr/bin/env python
from IRPOS import *

if __name__ == '__main__':
	irpos = IRPOS("IRpOS")

	irpos.move_to_synchro_position(10.0)

	print "Test compleated"
