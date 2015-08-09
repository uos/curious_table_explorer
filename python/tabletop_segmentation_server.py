#!/usr/bin/env python
#
# Curious Table Explorer - Tabletop Segmentation Server
#
# Copyright (C) 2014-2015 Michael 'v4hn' Goerner
# This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
# This is free software, and you are welcome to redistribute it
# under certain conditions; see LICENSE file for details

import sys

import rospy

import actionlib
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionResult, RecognizedObjectArray

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros
from ecto_ros import ecto_sensor_msgs

from uos_ecto_cells import uos_ecto_cells, ecto_object_recognition_msgs

#import transparent_object_reconstruction.hole_detection as hole_detection
#import transparent_object_reconstruction.ecto_transparent_object_reconstruction as ecto_transparent_object_reconstruction

class TableTopSegmentationServer:
	def __init__(self):
		self.create_plasm()
		self.plasm.configure_all()

		self.output= None
		rospy.Subscriber( '/recognized_object_array', RecognizedObjectArray, self.callback_recognized_object_array )

		self.server= actionlib.SimpleActionServer('recognize_objects', ObjectRecognitionAction, self.execute, False)
		self.server.start()
		rospy.loginfo('started tabletop segmentation server')

	def callback_recognized_object_array(self, data):
		self.output= data

	def execute(self, goal):
		rospy.loginfo('received request - running plasm once')
		if self.plasm.execute(niter= 1):
			# this is a crude hack to make the result accessable to this action server
			# it's pretty hard to support direct conversion for RecognizedObjectArray
			while self.output is None:
				rospy.sleep(.1)
			self.server.set_succeeded( ObjectRecognitionResult(self.output) )
			self.output= None
			rospy.loginfo('succeeded')
		else:
			self.server.set_aborted()

	def create_plasm(self):
		self.plasm= ecto.Plasm()
		graph= []

		cloud_sub= ecto_sensor_msgs.Subscriber_PointCloud2(topic_name= '/kinect/depth_registered/points', queue_size= 1)
		msg2cloud= ecto_pcl_ros.Message2PointCloud()
		cloud_pub= ecto_sensor_msgs.Publisher_PointCloud2(topic_name= '/segmented_view', queue_size= 5)
		graph.extend([
			cloud_sub[:] >> msg2cloud[:],
			cloud_sub[:] >> cloud_pub[:]
		])

		cloud_to_map= uos_ecto_cells.CloudReframer(target_frame= '/map', timeout= 0.2, tf_cache_time= 60.0)
		floor_cropper= ecto_pcl.PassThroughIndices(filter_field_name= "z", filter_limit_min= .20)
		extract_indices_floor= ecto_pcl.ExtractIndices(keep_organized= False)
		graph.extend([
			msg2cloud[:] >> cloud_to_map[:],
			cloud_to_map[:] >> floor_cropper["input"],
			floor_cropper[:] >> extract_indices_floor["indices"],
			msg2cloud[:] >> extract_indices_floor["input"]
		])

		normals= ecto_pcl.NormalEstimation(radius_search= .02, k_search= 0)
		graph.extend([
			extract_indices_floor[:] >> normals[:]
		])

		planar_segmentation= ecto_pcl.SACSegmentationFromNormals(
			# ecto_pcl.SACMODEL_NORMAL_PLANE would support normals, but these are too noisy, so disabled for now
			model_type= ecto_pcl.SACMODEL_PLANE,
			distance_threshold=.02,
			max_iterations= 125)
		extract_table_indices= ecto_pcl.ExtractIndices(negative= False, keep_organized= False)
		subtract_table_indices= ecto_pcl.ExtractIndices(negative= True, keep_organized= False)
		graph.extend([
			extract_indices_floor[:] >> planar_segmentation["input"],
			normals[:] >> planar_segmentation["normals"],
			planar_segmentation["inliers"] >> extract_table_indices["indices"],
			extract_indices_floor[:] >> extract_table_indices["input"],
			planar_segmentation["inliers"] >> subtract_table_indices["indices"],
			extract_indices_floor[:] >> subtract_table_indices["input"]
		])

		extract_table_clusters= ecto_pcl.EuclideanClusterExtraction(cluster_tolerance= .03)
		extract_largest_table_cluster= uos_ecto_cells.ExtractLargestClusterIndices()
		# this should keep the table inlier organized, but ConvexHull would fail
		# TODO: this is required for hole detection
		extract_largest_table_cluster_indices= ecto_pcl.ExtractIndices(keep_organized= False)
		graph.extend([
			extract_table_indices[:] >> extract_table_clusters["input"],
			extract_table_clusters[:] >> extract_largest_table_cluster["clusters"],
			extract_largest_table_cluster[:] >> extract_largest_table_cluster_indices["indices"],
			extract_table_indices[:] >> extract_largest_table_cluster_indices["input"]
		])

		table_inlier2msg= ecto_pcl_ros.PointCloud2Message()
		inlier_pub= ecto_sensor_msgs.Publisher_PointCloud2(topic_name= '/table_inliers', queue_size=1)
		graph.extend([
			extract_largest_table_cluster_indices[:] >> table_inlier2msg[:],
			table_inlier2msg[:] >> inlier_pub[:]
		])

		convex_table= uos_ecto_cells.ConvexHull(dimensionality= 2)
		convex2tables= uos_ecto_cells.ConvexHull2Table()
		table_pub= ecto_object_recognition_msgs.Publisher_TableArray(topic_name= '/table')
		graph.extend([
			extract_largest_table_cluster_indices[:] >> convex_table[:],
			convex_table["output"] >> convex2tables[:],
			convex2tables[:] >> table_pub[:]
		])

		# TODO: include these cells
		#hole_detector= hole_detection.HoleDetector()
		#hole_publisher= ecto_transparent_object_reconstruction.Publisher_Holes(topic_name= '/table_holes')
		#graph.extend([
		#	msg2cloud[:] >> hole_detector["input"],
		#	convex_table[:] >> hole_detector["hull_indices"],
		#	planar_segmentation["model"] >> hole_detector["model"],

		#	hole_detector["holes"] >> hole_publisher[:]
		#])

		extract_table_content= ecto_pcl.ExtractPolygonalPrismData(height_min= 0.0, height_max= .5)
		graph.extend([
			convex_table["output"] >> extract_table_content["planar_hull"],
			#hole_detector["output"] >> extract_table_content["input"]
			subtract_table_indices[:] >> extract_table_content["input"]
		])

		cluster_table_content= ecto_pcl.EuclideanClusterExtraction(cluster_tolerance= .05, min_cluster_size= 20)
		graph.extend([
			subtract_table_indices[:] >> cluster_table_content["input"],
			extract_table_content[:] >> cluster_table_content["indices"]
		])

		clusters2recognized_objects= uos_ecto_cells.Clusters2RecognizedObjectArray()
		recognized_objects_pub= ecto_object_recognition_msgs.Publisher_RecognizedObjectArray(topic_name= '/recognized_object_array')
		graph.extend([
			cluster_table_content[:] >> clusters2recognized_objects["indices"],
			subtract_table_indices[:] >> clusters2recognized_objects["input"],
			clusters2recognized_objects[:] >> recognized_objects_pub[:]
		])

		colorize_clusters= ecto_pcl.ColorizeClusters()
		clusters2cloudmsg= ecto_pcl_ros.PointCloud2Message()
		table_content_cloud_pub= ecto_sensor_msgs.Publisher_PointCloud2(topic_name= '/table_content')
		graph.extend([
			cluster_table_content[:] >> colorize_clusters["clusters"],
			subtract_table_indices[:] >> colorize_clusters["input"],
			colorize_clusters[:] >> clusters2cloudmsg[:],
			clusters2cloudmsg[:] >> table_content_cloud_pub[:]
		])

		self.plasm.connect(graph)

if __name__ == '__main__':
	node_name= "tabletop_segmentation"
	rospy.init_node(node_name)

	node_name= rospy.get_name()
	try:
		node_name= node_name[(node_name.rindex('/')+1):]
	except ValueError:
		pass
	# this filter is necessary to ensure the python & cpp node
	# get different names when this is called from a launch file
	ecto_ros.init([s for s in sys.argv if not s.startswith('__name:=')], node_name+'_ecto', anonymous= True)
	ttserver= TableTopSegmentationServer()
	rospy.spin()
