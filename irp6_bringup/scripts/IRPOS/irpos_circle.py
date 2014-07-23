#!/usr/bin/env python
from IRPOS import *

if __name__ == '__main__':
	irpos = IRPOS("IRpOS", "Irp6p")
	
	irpos.move_to_joint_position([0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], 5.0)

	cartesianTrajectory = [CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.855438961242, -0.1208864692291, 1.281029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist())]
	irpos.move_along_cartesian_trajectory(cartesianTrajectory)

	P1 = Pose(Point(0.855438961242, -0.1208864692291, 1.281029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928))
	P2 = Pose(Point(0.755438961242, 0.0, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928))
	P3 = Pose(Point(0.855438961242, 0.1208864692291, 1.081029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928))

	irpos.move_along_cartesian_circle(P1, P2, P3, 30.0)

	print "Test compleated"
