#!/usr/bin/env python

import sys
import rospy
import actionlib

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped

from curious_table_explorer.msg import SurroundPointAction, SurroundPointGoal

if __name__ == '__main__':
	rospy.init_node('surround_client')

	if len(sys.argv) != 3 and len(sys.argv) != 4:
		print "usage: surround_client.py <x> <y> [frame_id]"
		print "will surround the point (x,y,0) in /map or [frame_id]"
		exit(1)

	x= float(sys.argv[1])
	y= float(sys.argv[2])

	frame_id= "/map"
	if len(sys.argv) == 4:
		frame_id= sys.argv[3]

	client= actionlib.SimpleActionClient('/surround_point', SurroundPointAction)
	client.wait_for_server()
	client.send_goal(
		SurroundPointGoal(
			point= PointStamped(
				Header(frame_id= frame_id),
				Point(x,y,0.0)
			)
		)
	)
	client.wait_for_result()
