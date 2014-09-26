#!/usr/bin/env python

import sys

import rospy

import actionlib
from object_recognition_msgs.msg import ObjectRecognitionAction

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros
from ecto_ros import ecto_sensor_msgs

class TableTopSegmentationServer:
	def __init__(self):
		self.create_plasm()
		self.plasm.configure_all()

		self.server= actionlib.SimpleActionServer('recognize_objects', ObjectRecognitionAction, self.execute, False)
		self.server.start()
		rospy.loginfo('started tts-server')

	def execute(self, goal):
		rospy.loginfo('received goal: ' + str(goal))
		self.plasm.execute(niter= 1)
		self.server.set_succeeded()
		
	def create_plasm(self):
		self.plasm= ecto.Plasm()
		graph= []

		cloud_sub= ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub", topic_name= '/kinect/depth_registered/points', queue_size= 1)
		msg2cloud= ecto_pcl_ros.Message2PointCloud("msg2cloud")
		voxel_grid= ecto_pcl.VoxelGrid("voxel_grid", leaf_size=.008)
		normals= ecto_pcl.NormalEstimation("normals", radius_search= .02, k_search= 0)
		planar_segmentation= ecto_pcl.SACSegmentationFromNormals("planar_segmentation",
			model_type= ecto_pcl.SACMODEL_NORMAL_PLANE,
			eps_angle=.06,
			distance_threshold=.02)
		inlier_projection= ecto_pcl.ProjectInliers("inlier_projection", model_type= ecto_pcl.SACMODEL_NORMAL_PLANE)
		convex_table= ecto_pcl.ConvexHull("convex_table", dimensionality= 2)
		extract_table_content= ecto_pcl.ExtractPolygonalPrismData("extract_table_content", height_min=.025, height_max=.5)
		extract_table_content_indices= ecto_pcl.ExtractIndices("extract_table_content_indices", negative= False)
		cloud2msg= ecto_pcl_ros.PointCloud2Message()
		table_content_cloud_pub= ecto_sensor_msgs.Publisher_PointCloud2("table_content_cloud_pub", topic_name= '/table_content')

		graph+= [
			cloud_sub[:] >> msg2cloud[:],
			msg2cloud[:] >> voxel_grid[:],
			voxel_grid[:] >> normals[:],
			voxel_grid[:] >> planar_segmentation["input"],
			normals[:]    >> planar_segmentation["normals"],
			voxel_grid[:] >> inlier_projection["input"],
			planar_segmentation["model"] >> inlier_projection["model"],
			inlier_projection[:] >> convex_table[:],
			msg2cloud[:] >> extract_table_content["input"],
			convex_table[:] >> extract_table_content["planar_hull"],
			extract_table_content[:] >> extract_table_content_indices["indices"],
			msg2cloud[:] >> extract_table_content_indices["input"],
			extract_table_content_indices[:] >> cloud2msg[:],
			cloud2msg[:] >> table_content_cloud_pub[:]
		]

		self.plasm.connect(graph)

if __name__ == '__main__':
	rospy.init_node('tabletop_segmentation')
	ecto_ros.init(sys.argv, 'tabletop_segmentation_ecto')
	ttserver= TableTopSegmentationServer()
	rospy.spin()
