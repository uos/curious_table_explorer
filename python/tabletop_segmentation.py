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

from object_recognition_msgs.msg import RecognizedObjectArray

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros
from ecto_ros import ecto_sensor_msgs

from uos_ecto_cells import uos_ecto_cells, ecto_object_recognition_msgs

class CloudPublisher(ecto.BlackBox):
	@staticmethod
	def declare_cells(_p):
		cells= {}
		cells['m2c']= ecto.BlackBoxCellInfo(ecto_pcl_ros.PointCloud2Message, name= 'm2c')
		cells['pub']= ecto.BlackBoxCellInfo(ecto_sensor_msgs.Publisher_PointCloud2, name= 'pub')
		return cells
	@staticmethod
	def declare_forwards(_p):
		p= {'pub': [ecto.BlackBoxForward('latched'), ecto.BlackBoxForward('queue_size'), ecto.BlackBoxForward('topic_name')] }
		i= {'m2c': [ecto.BlackBoxForward('input', new_doc="Cloud to publish.")] }
		o= {'pub': [ecto.BlackBoxForward('has_subscribers')]}
		return (p,i,o)
	def configure(self, _p, _i, _o):
		self.m2c.configure()
		self.pub.configure()
	def connections(self, _p):
		return [ self.m2c[:] >> self.pub[:] ]


class TableTopSegmentation:
	def __init__(self):
		ecto_ros.init(sys.argv, 'tabletop_segmentation', anonymous= False)
		self.create_plasm()
		self.plasm.configure_all()

		rospy.loginfo('started tabletop segmentation')

	def run(self):
		self.plasm.execute(niter= 0)

	def create_plasm(self):
		self.plasm= ecto.Plasm()
		graph= []

		cloud_sub= ecto_sensor_msgs.Subscriber_PointCloud2(topic_name= 'table_view', queue_size= 10)
		msg2cloud= ecto_pcl_ros.Message2PointCloud()
		graph.extend([
			cloud_sub[:] >> msg2cloud[:]
		])

		cloud_to_map= uos_ecto_cells.CloudReframer(target_frame= 'map', timeout= 0.2, tf_cache_time= 60.0)
		floor_cropper= ecto_pcl.PassThroughIndices(filter_field_name= "z", filter_limit_min= .20)
		graph.extend([
			msg2cloud[:] >> cloud_to_map[:],
			cloud_to_map[:] >> floor_cropper["input"],
		])

		# ecto_pcl.SACSegmentationFromNormals could be used, but the interpolated normals are too noisy..
		planar_segmentation= ecto_pcl.SACSegmentation(
			model_type= ecto_pcl.SACMODEL_PLANE,
			distance_threshold=.02,
			max_iterations= 125)
		graph.extend([
			msg2cloud[:] >> planar_segmentation["input"],
			floor_cropper[:] >> planar_segmentation["indices"],
		])

		extract_table_clusters= ecto_pcl.EuclideanClusterExtraction(cluster_tolerance= .03)
		extract_largest_table_cluster= uos_ecto_cells.ExtractLargestClusterIndices()
		graph.extend([
			msg2cloud[:] >> extract_table_clusters["input"],
			planar_segmentation["inliers"] >> extract_table_clusters["indices"],
			extract_table_clusters[:] >> extract_largest_table_cluster["clusters"],
		])

		extract_largest_table_cluster_indices= ecto_pcl.ExtractIndices(keep_organized= False)
		inlier_pub= CloudPublisher(topic_name= 'table_inliers', queue_size=1)
		graph.extend([
			extract_largest_table_cluster[:] >> extract_largest_table_cluster_indices["indices"],
			msg2cloud[:] >> extract_largest_table_cluster_indices["input"],
			extract_largest_table_cluster_indices[:] >> inlier_pub[:]
		])

		inlier_projection= uos_ecto_cells.ProjectPlaneInliersPerspectively()
		graph.extend([
			msg2cloud[:] >> inlier_projection["input"],
			extract_largest_table_cluster[:] >> inlier_projection["indices"],
			planar_segmentation["model"] >> inlier_projection["model"]
		])

		convex_table= uos_ecto_cells.ConvexHull(dimensionality= 2)
		convex2tables= uos_ecto_cells.ConvexHull2Table()
		table_pub= ecto_object_recognition_msgs.Publisher_TableArray(topic_name= 'table')
		graph.extend([
			inlier_projection[:] >> convex_table["input"],
			extract_largest_table_cluster[:] >> convex_table["indices"],
			convex_table["output"] >> convex2tables[:],
			convex2tables[:] >> table_pub[:]
		])

		import transparent_object_reconstruction.hole_detection as hole_detection
		import transparent_object_reconstruction.ecto_transparent_object_reconstruction as ecto_transparent_object_reconstruction

		hole_detector= hole_detection.HoleDetector(plane_dist_threshold= 0.02)
		hole_publisher= ecto_transparent_object_reconstruction.Publisher_Holes(topic_name= 'table_holes')
		graph.extend([
			msg2cloud[:] >> hole_detector["input"],
			convex_table["output_indices"] >> hole_detector["hull_indices"],
			planar_segmentation["model"] >> hole_detector["model"],

			hole_detector["holes"] >> hole_publisher[:]
		])

		extract_indices_floor= ecto_pcl.ExtractIndices(keep_organized= True)
		subtract_table_indices= ecto_pcl.ExtractIndices(negative= True, keep_organized= True)
		graph.extend([
			floor_cropper[:] >> extract_indices_floor["indices"],
			msg2cloud[:] >> extract_indices_floor["input"],
			extract_largest_table_cluster[:] >> subtract_table_indices["indices"],
			extract_indices_floor[:] >> subtract_table_indices["input"]
		])

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
		table_content_cloud_pub= CloudPublisher(topic_name= 'table_content', queue_size= 1)
		graph.extend([
			cluster_table_content[:] >> colorize_clusters["clusters"],
			subtract_table_indices[:] >> colorize_clusters["input"],
			colorize_clusters[:] >> table_content_cloud_pub[:]
		])

		self.plasm.connect(graph)

if __name__ == '__main__':
	ttserver= TableTopSegmentation()
	ttserver.run()
