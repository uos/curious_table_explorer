#!/usr/bin/env python

import sys
import rospy
import actionlib

import tf

from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Quaternion
from object_recognition_msgs.msg import Table

from curious_table_explorer.msg import SurroundPointAction, SurroundPointGoal

class TablePatrol:
	def __init__(self):
		self.tables= []

		self.tf_listener= tf.TransformListener()

		self.client= actionlib.SimpleActionClient('/surround_point', SurroundPointAction)
		self.client.wait_for_server()

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

if __name__ == '__main__':
	rospy.init_node('table_patrol')

	patrol= TablePatrol()

	patrol.add_table( Table(header= Header(frame_id='/map'), pose= Pose( Point(1.75,-1.75,1), Quaternion(0,0,0,1))) )
	patrol.add_table( Table(header= Header(frame_id='/map'), pose= Pose( Point(1,1,1), Quaternion(0,0,0,1))) )

	while not rospy.is_shutdown():
		patrol.patrol_once()
