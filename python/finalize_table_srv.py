#!/usr/bin/env python
#
# Curious Table Explorer - Finalize Table
#
# This module listens for Strings on topic 'finalize_table'
# and resets the caches of table observations with each incoming message
#
# Copyright (C) 2015 Michael 'v4hn' Goerner
# This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
# This is free software, and you are welcome to redistribute it
# under certain conditions; see LICENSE file for details

import sys
import rospy
import actionlib

from std_msgs.msg import Header, String

from curious_table_explorer.srv import FinalizeTable

class FinalizeTableServer:
	def __init__(self):
		self.finalize_object_views= rospy.ServiceProxy('finalize_table', FinalizeTable)
		try:
			from transparent_object_reconstruction.srv import HoleIntersectorReset
			self.finalize_transparent_views= rospy.ServiceProxy('transObjRec/HoleIntersector_reset', FinalizeTable)
		except:
			self.finalize_transparent_views= None
		self.finalize_msg_sub= rospy.Subscriber('finalize_table', String, self.msg_cb, queue_size= 10)

	def msg_cb(self, s):
		rospy.loginfo('Resetting table view caches')
		try:
			self.finalize_object_views()
			if self.finalize_transparent_views != None:
				self.finalize_transparent_views()
		except Exception as e:
			rospy.logwarn('Could not reset table view caches: '+ str(e))

if __name__ == '__main__':
	rospy.init_node('finalize_table')
	ft= FinalizeTableServer()
	rospy.spin()
