#!/usr/bin/env python
#
# Curious Table Explorer - Table Patrol
#
# Copyright (C) 2015 Michael 'v4hn' Goerner
# This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
# This is free software, and you are welcome to redistribute it
# under certain conditions; see LICENSE file for details

import sys
import rospy
import actionlib

import tf

from std_msgs.msg import Header, String
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Quaternion
from object_recognition_msgs.msg import Table

from curious_table_explorer.msg import SurroundPointAction, SurroundPointGoal
from curious_table_explorer.srv import FinalizeTable

class TablePatrol:
	def __init__(self):
		self.tables= []

		self.tf_listener= tf.TransformListener()

		self.client= actionlib.SimpleActionClient('/surround_point', SurroundPointAction)
		self.client.wait_for_server()

#		self.finalize_service= rospy.ServiceProxy('finalize_table', FinalizeTable)
		self.finalize_pub= rospy.Publisher('/finalize_table', String, queue_size= 1)

	def finalize_table(self):
#		self.finalize_service()
		self.finalize_pub.publish(String("finalize"))

	def add_table(self, table):
		self.tf_listener.waitForTransform('/map', table.header.frame_id, table.header.stamp, rospy.Duration(3.0))
		p= self.tf_listener.transformPose('/map', PoseStamped( header= table.header, pose= table.pose) )
		table.header= p.header
		table.pose= p.pose
		table.convex_hull= [self.tf_listener.transformPoint('/map', PointStamped(header= table.header, point= p)).point for p in table.convex_hull]

		self.tables.insert(0, table)

	def patrol_once(self):
		table= self.tables.pop(0)
		self.tables.append(table)

		self.client.send_goal(
			SurroundPointGoal(
				point= PointStamped(
					table.header,
					table.pose.position
				)
			)
		)
		self.client.wait_for_result()
		rospy.sleep(rospy.Duration(30))
		self.finalize_table()

if __name__ == '__main__':
	rospy.init_node('table_patrol')

	patrol= TablePatrol()

#	patrol.add_table( Table(header= Header(frame_id='/map'), pose= Pose( Point(1,0,1), Quaternion(1,0,0,1))) )
	patrol.add_table( Table(header= Header(frame_id='/map'), pose= Pose( Point(-.5,-3.25,1), Quaternion(0,0,0,1))) )

	while not rospy.is_shutdown():
		patrol.patrol_once()
		rospy.sleep(rospy.Duration(30))
