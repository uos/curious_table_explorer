#!/usr/bin/env python
#
# Curious Table Explorer - Surround Point
#
# Copyright (C) 2015 Michael 'v4hn' Goerner
# This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
# This is free software, and you are welcome to redistribute it
# under certain conditions; see LICENSE file for details

import sys
import rospy
import actionlib

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped

from curious_table_explorer.msg import SurroundPointAction, SurroundPointGoal

if __name__ == '__main__':
	rospy.init_node('surround_client')

	if len(sys.argv) != 3 and len(sys.argv) != 4:
		print "usage: surround_client.py <x> <y> [frame_id] [delta]"
		print "will surround the point (x,y,0) in /map or [frame_id] with delta(rad) between two poses"
		exit(1)

	x= float(sys.argv[1])
	y= float(sys.argv[2])

	frame_id= "/map"
	if len(sys.argv) > 3:
		frame_id= sys.argv[3]

	delta= 0.5
	if len(sys.argv) > 4:
		delta= float(sys.argv[4])

	client= actionlib.SimpleActionClient('/surround_point', SurroundPointAction)
	client.wait_for_server()
	client.send_goal(
		SurroundPointGoal(
			point= PointStamped(
				Header(frame_id= frame_id),
				Point(x,y,0.0)
			),
			delta= delta
		)
	)
	client.wait_for_result()
