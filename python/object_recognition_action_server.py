#!/usr/bin/env python
#
# Curious Table Explorer - Object Recognition Action Server
#
# Copyright (C) 2014-2015 Michael 'v4hn' Goerner
# This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
# This is free software, and you are welcome to redistribute it
# under certain conditions; see LICENSE file for details

import rospy

import actionlib

from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionResult, RecognizedObjectArray
from sensor_msgs.msg import PointCloud2

import sys
import time
import threading

class ObjectRecognitionActionServer:
	def __init__(self):
		self.sub_table_view= rospy.Subscriber( 'input_cloud', PointCloud2, self.callback_table_view, queue_size= 10 )
		self.new_view= threading.Event()
		self.table_view= None

		self.pub_table_view= rospy.Publisher( 'pipeline_input', PointCloud2, queue_size= 10 )

		self.sub_output= rospy.Subscriber( 'pipeline_output', RecognizedObjectArray, self.callback_output, queue_size= 10 )
		self.new_output= threading.Event()
		self.output= None

		self.server= actionlib.SimpleActionServer(rospy.resolve_name('recognize_objects'), ObjectRecognitionAction, self.execute, False)

		deadline= time.time() + rospy.get_param('~startup_timeout', 15.0)
		while (not rospy.is_shutdown()) and (self.pub_table_view.get_num_connections() == 0) and (time.time() < deadline):
			time.sleep(0.05)
		if rospy.is_shutdown():
			exit()
		if self.pub_table_view.get_num_connections() < 1:
			rospy.logfatal("Object recognition pipeline did not connect to topic " + rospy.resolve_name("input_cloud") + ". This server only wraps a pipeline to use as an action. It is useless without a functional recognition pipeline.")
			exit()

		self.server.start()
		rospy.loginfo('started object recognition action server')

	def callback_table_view(self, data):
		self.table_view= data
		self.new_view.set()

	def callback_output(self, data):
		self.output= data
		self.new_output.set()

	def execute(self, goal):
		now= rospy.Time.now()
		rospy.loginfo("object recognition requested at time " + str(now))
		while self.table_view == None or self.table_view.header.stamp < now:
			self.new_view.wait()
			self.new_view.clear()
		self.new_output.clear()
		self.pub_table_view.publish(self.table_view)

		timeout= rospy.get_param("~detection_timeout",15.0)
		if self.new_output.wait(timeout):
			self.new_output.clear()
			self.server.set_succeeded( ObjectRecognitionResult(self.output) )
			self.output= None
			rospy.loginfo('succeeded')
		else:
			rospy.logerr("object recognition pipeline didn't return a result in time(timeout: "+str(timeout)+"s")
			self.server.set_aborted()

if __name__ == '__main__':
	rospy.init_node("object_recognition_action_server")
	ttserver= ObjectRecognitionActionServer()
	rospy.spin()
